"""
Continuous-time model for an output LC filter.

The space vector model is implemented in stator coordinates. 

"""
from types import SimpleNamespace

from motulator.common.model import Subsystem
from motulator.common.utils import complex2abc, FilterPars


# %%
# TODO: remove this file as LCFilter class has moved to
# motulator.common.model._ac_filter.py
class LCFilter(Subsystem):
    """
    LC-filter model.

    Parameters
    ----------
    filter_pars : FilterPars
        Filter parameters. Machine drive LC-filter uses the following parameters:
    
            filter_pars.L_fc : float
                Converter-side inductance of the filter (H).
            filter_pars.C_f : float
                Filter capacitance (F).
            filter_pars.G_f : float, optional
                Filter conductance (S). The default is 0.
            filter_pars.R_fc : float, optional
                Converter-side series resistance (Ω). The default is 0.
   
    """

    def __init__(self, filter_pars: FilterPars):
        super().__init__()
        self.par = SimpleNamespace(
            L_fc=filter_pars.L_fc,
            C_f=filter_pars.C_f,
            R_fc=filter_pars.R_fc,
            G_f=filter_pars.G_f)
        self.state = SimpleNamespace(i_cs=0, u_fs=0)
        self.sol_states = SimpleNamespace(i_cs=[], u_fs=[])

    def set_outputs(self, _):
        """Set output variables."""
        state, out = self.state, self.out
        out.i_cs, out.u_fs = state.i_cs, state.u_fs

    def rhs(self):
        """Compute state derivatives."""
        state, inp, par = self.state, self.inp, self.par
        d_i_cs = (inp.u_cs - state.u_fs - par.R_fc*state.i_cs)/par.L_fc
        d_u_fs = (state.i_cs - inp.i_fs - par.G_f*state.u_fs)/par.C_f

        return [d_i_cs, d_u_fs]

    def meas_currents(self):
        """Measure the converter phase currents."""
        i_c_abc = complex2abc(self.state.i_cs)
        return i_c_abc

    def meas_cap_voltage(self):
        """Measure the capacitor phase voltages."""
        u_f_abc = complex2abc(self.state.u_fs)
        return u_f_abc
