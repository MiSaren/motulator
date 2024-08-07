"""
Grid and AC filter impedance models.

This module contains continuous-time models for subsystems comprising an AC 
filter and a grid impedance between the converter and grid voltage sources. The 
models are implemented with space vectors in stationary coordinates.

"""
from types import SimpleNamespace

from motulator.common.utils._utils import complex2abc, FilterPars
from motulator.common.model import Subsystem
from motulator.grid.utils import GridPars


# %%
# TODO: remove this file as LFilter and LCLFilter classes have moved to
# motulator.common.model._ac_filter.py
class LFilter(Subsystem):
    """
    Dynamic model for an inductive L filter and an inductive-resistive grid.

    An L filter and an inductive-resistive grid impedance, between the
    converter and grid voltage sources, are modeled combining their inductances
    and series resistances in a state equation. The grid current is used as a
    state variable. The point-of-common-coupling (PCC) voltage between the L
    filter and the grid impedance is separately calculated.

    Parameters
    ----------
    grid_par : GridPars
        Grid model parameters. Needed to set the initial value of PCC voltage.
    filter_par : FilterPars
        Filter model parameters.
        L-Filter model uses only following FilterPars parameters:

            filter_par.L_fc : float
                Converter-side inductance of the filter (H).
            filter_par.R_fc : float (optional)
                Converter-side series resistance (Ω). The default is 0.

    """

    def __init__(self, grid_par:GridPars, filter_par:FilterPars):
        super().__init__()
        self.par = SimpleNamespace(L_f=filter_par.L_fc, R_f=filter_par.R_fc, L_g=grid_par.L_g, R_g=grid_par.R_g)
        self.inp = SimpleNamespace(u_cs=0+0j, e_gs=grid_par.u_gN+0j)
        self.out = SimpleNamespace(u_gs=grid_par.u_gN+0j)  # Needed for direct feedthrough
        self.state = SimpleNamespace(i_gs=0+0j)
        self.sol_states = SimpleNamespace(i_gs=[])

    def set_outputs(self, _):
        """Set output variables."""
        state, par, inp, out = self.state, self.par, self.inp, self.out
        u_gs = (par.L_g*inp.u_cs + par.L_f*inp.e_gs +
            (par.R_g*par.L_f - par.R_f*par.L_g)*
            state.i_gs)/(par.L_g+par.L_f)
        out.i_gs, out.i_cs, out.u_gs = state.i_gs, state.i_gs, u_gs

    def rhs(self):
        """
        Compute the state derivatives.

        Returns
        -------
        complex list, length 1
            Time derivative of the complex state vector, [d_i_gs].

        """
        state, inp, par = self.state, self.inp, self.par
        L_t = par.L_f + par.L_g
        R_t = par.R_f + par.R_g
        d_i_gs = (inp.u_cs - inp.e_gs - R_t*state.i_gs)/L_t
        return [d_i_gs]

    def meas_currents(self):
        """
        Measure the phase currents.

        Returns
        -------
        i_g_abc : 3-tuple of floats
            Grid phase currents (A).

        """
        # Grid phase currents from the corresponding space vector
        i_g_abc = complex2abc(self.state.i_gs)
        return i_g_abc

    def meas_pcc_voltage(self):
        """
        Measure the phase voltages at the point of common coupling (PCC).

        Returns
        -------
        u_g_abc : 3-tuple of floats
            Phase voltages at the PCC (V).

        """
        # PCC phase voltages from the corresponding space vector
        u_g_abc = complex2abc(self.out.u_gs)
        return u_g_abc

    def post_process_states(self):
        """Post-process data."""
        self.data.i_cs=self.data.i_gs

    def post_process_with_inputs(self):
        """Post-process data with inputs."""
        data = self.data
        data.u_gs = (self.par.L_g*data.u_cs + self.par.L_f*data.e_gs +
            (self.par.R_g*self.par.L_f - self.par.R_f*self.par.L_g)*
            data.i_gs)/(self.par.L_g+self.par.L_f)


# %%
class LCLFilter(Subsystem):
    """
    Dynamic model for an inductive-capacitive-inductive (LCL) filter and grid.

    An LCL filter and an inductive-resistive grid impedance, between the 
    converter and grid voltage sources, are modeled using converter current, 
    LCL-filter capacitor voltage and grid current as state variables. The grid 
    inductance and resistance are included in the state equation of the grid 
    current. The point-of-common-coupling (PCC) voltage between the LCL filter 
    and the grid impedance is separately calculated.

    Parameters
    ----------
    grid_par : GridPars
        Grid model parameters. Needed to set the initial value of PCC voltage.
    filter_par : FilterPars
        Filter model parameters.
    """


    def __init__(self, grid_par:GridPars, filter_par:FilterPars):
        super().__init__()
        self.par = SimpleNamespace(
            L_fc = filter_par.L_fc,
            R_fc = filter_par.R_fc,
            L_fg = filter_par.L_fg,
            R_fg = filter_par.R_fg,
            C_f = filter_par.C_f,
            G_f = filter_par.G_f,
            L_g = grid_par.L_g,
            R_g = grid_par.R_g
            )
        self.inp = SimpleNamespace(u_cs=0+0j, e_gs=grid_par.u_gN+0j)
        self.out = SimpleNamespace(u_gs=grid_par.u_gN+0j)
        self.state = SimpleNamespace(i_cs=0+0j, u_fs=grid_par.u_gN+0j, i_gs=0+0j)
        self.sol_states = SimpleNamespace(i_cs=[], u_fs=[], i_gs=[])

    def set_outputs(self, _):
        """Set output variables."""
        state, par, inp, out = self.state, self.par, self.inp, self.out
        u_gs = (par.L_fg*inp.e_gs + par.L_g*state.u_fs + (par.R_g*par.L_fg -
                par.R_fg*par.L_g)*state.i_gs)/(par.L_g+par.L_fg)
        out.i_cs, out.u_fs, out.i_gs, out.u_gs = (state.i_cs, state.u_fs,
                                                state.i_gs, u_gs)

    def rhs(self):
        """
        Compute the state derivatives.

        Returns
        -------
        complex list, length 3
            Time derivative of the complex state vector,
            [d_i_cs, d_u_fs, d_i_gs].

        """
        state, par, inp = self.state, self.par, self.inp
        # Converter current dynamics
        d_i_cs = (inp.u_cs - state.u_fs - par.R_fc*state.i_cs)/par.L_fc
        # Capacitor voltage dynamics
        d_u_fs = (state.i_cs - state.i_gs - par.G_f*state.u_fs)/par.C_f
        # Calculation of the total grid-side impedance
        L_t = par.L_fg + par.L_g
        R_t = par.R_fg + par.R_g
        # Grid current dynamics
        d_i_gs = (state.u_fs - inp.e_gs - R_t*state.i_gs)/L_t

        return [d_i_cs, d_u_fs, d_i_gs]

    def meas_currents(self):
        """
        Measure the converter phase currents.

        Returns
        -------
        i_c_abc : 3-tuple of floats
            Converter phase currents (A).

        """
        # Converter phase currents from the corresponding space vector
        i_c_abc = complex2abc(self.state.i_cs)

        return i_c_abc

    def meas_grid_currents(self):
        """
        Measure the grid phase currents.

        Returns
        -------
        i_g_abc : 3-tuple of floats
            Grid phase currents (A).

        """
        # Grid phase currents from the corresponding space vector
        i_g_abc = complex2abc(self.state.i_gs)
        return i_g_abc

    def meas_cap_voltage(self):
        """
        Measure the capacitor phase voltages.

        Returns
        -------
        u_f_abc : 3-tuple of floats
            Phase voltages of the LCL filter capacitor (V).

        """
        # Capacitor phase voltages from the corresponding space vector
        u_f_abc = complex2abc(self.state.u_fs)
        return u_f_abc

    def meas_pcc_voltage(self):
        """
        Measure the phase voltages at the point of common coupling (PCC).

        Returns
        -------
        u_g_abc : 3-tuple of floats
            Phase voltages at the PCC (V).

        """
        # PCC phase voltages from the corresponding space vector
        u_g_abc = complex2abc(self.out.u_gs)
        return u_g_abc

    def post_process_with_inputs(self):
        """Post-process data with inputs."""
        data, par = self.data, self.par
        data.u_gs = (par.L_fg*data.e_gs+par.L_g*data.u_fs+(par.R_g*par.L_fg-
                    par.R_fg*par.L_g)*data.i_gs)/(par.L_g+par.L_fg)
