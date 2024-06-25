"""
Grid and AC filter impedance models.

This module contains continuous-time models for subsystems comprising an AC 
filter and a grid impedance between the converter and grid voltage sources. The 
models are implemented with space vectors in stationary coordinates.

"""
from types import SimpleNamespace

from motulator.common.utils._utils import complex2abc
from motulator.common.model import Subsystem


# %%
# TODO: move filter models to motulator/common along with LCFilter from drive,
# if feasible
class LFilter(Subsystem):
    """
    Dynamic model for an inductive L filter and an inductive-resistive grid.

    An L filter and an inductive-resistive grid impedance, between the converter 
    and grid voltage sources, are modeled combining their inductances and series
    resistances in a state equation. The grid current is used as a state 
    variable. The point-of-common-coupling (PCC) voltage between the L filter 
    and the grid impedance is separately calculated.

    Parameters
    ----------
    L_f : float
        Filter inductance (H)
    R_f : float
        Filter series resistance (Ω)
    L_g : float
        Grid inductance (H)
    R_g : float
        Grid resistance (Ω)

    """
    def __init__(self, U_gN, L_f, R_f=0, L_g=0, R_g=0):
        super().__init__()
        self.par = SimpleNamespace(L_f=L_f, R_f=R_f, L_g=L_g, R_g=R_g)
        self.inp = SimpleNamespace(u_cs=0+0j, e_gs=U_gN+0j)
        self.out = SimpleNamespace(u_gs=U_gN+0j)  # Needed for direct feedthrough
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
        # pylint: disable=R0913
        """
        Compute the state derivatives.

        Returns
        -------
        complex list, length 1
            Time derivative of the complex state vector, [di_gs]

        """
        state, inp, par = self.state, self.inp, self.par
        L_t = par.L_f + par.L_g
        R_t = par.R_f + par.R_g
        di_gs = (inp.u_cs - inp.e_gs - R_t*state.i_gs)/L_t
        return [di_gs]

    def meas_currents(self):
        """
        Measure the phase currents at the end of the sampling period.

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
        Measure the phase voltages at PCC at the end of the sampling period.

        Returns
        -------
        u_g_abc : 3-tuple of floats
            Phase voltage at the point of common coupling (V).

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
    Dynamic model for an inductive-capacitive-inductive (LCL) filter and a grid.

    An LCL filter and an inductive-resistive grid impedance, between the 
    converter and grid voltage sources, are modeled using converter current, 
    LCL-filter capacitor voltage and grid current as state variables. The grid 
    inductance and resistance are included in the state equation of the grid 
    current. The point-of-common-coupling (PCC) voltage between the LCL filter 
    and the grid impedance is separately calculated.

    Parameters
    ----------
    L_fc : float
        Converter-side inductance of the LCL filter (H)
    R_fc : float
        Converter-side series resistance (Ω)
    L_fg : float
        Grid-side inductance of the LCL filter (H)
    R_fg : float
        Grid-side series resistance (Ω)
    C_f : float
        Capacitance of the LCL Filter (F)
    G_f : float
        Conductance of a resistor in parallel with the LCL-filter capacitor (S)
    L_g : float
        Grid inductance (H)
    R_g : float
        Grid resistance (Ω)


    """
    def __init__(
            self, U_gN, L_fc=6e-3, R_fc=0, L_fg=3e-3,
            R_fg=0, C_f=10e-6, G_f=0, L_g=0, R_g=0):
        super().__init__()
        self.par = SimpleNamespace(L_fc=L_fc, R_fc=R_fc, L_fg=L_fg, R_fg=R_fg,
                                    C_f=C_f, G_f=G_f, L_g=L_g, R_g=R_g)
        self.inp = SimpleNamespace(u_cs=0+0j, e_gs=U_gN+0j)
        self.out = SimpleNamespace(u_gs=U_gN+0j)
        self.state = SimpleNamespace(i_cs=0+0j, u_fs=U_gN+0j, i_gs=0+0j)
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
            Time derivative of the complex state vector, [di_cs, du_fs, di_gs]

        """
        state, par, inp = self.state, self.par, self.inp
        # Converter current dynamics
        di_cs = (inp.u_cs - state.u_fs - par.R_fc*state.i_cs)/par.L_fc
        # Capacitor voltage dynamics
        du_fs = (state.i_cs - state.i_gs - par.G_f*state.u_fs)/par.C_f
        # Calculation of the total grid-side impedance
        L_t = par.L_fg + par.L_g
        R_t = par.R_fg + par.R_g
        # Grid current dynamics
        di_gs = (state.u_fs - inp.e_gs - R_t*state.i_gs)/L_t

        return [di_cs, du_fs, di_gs]

    def meas_currents(self):
        """
        Measure the converter phase currents at the end of the sampling period.

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
        Measure the grid phase currents at the end of the sampling period.

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
        Measure the capacitor phase voltages at the end of the sampling period.

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
        Measure the PCC voltages at the end of the sampling period.

        Returns
        -------
        u_g_abc : 3-tuple of floats
            Phase voltages at the point of common coupling (V).

        """
        # PCC phase voltages from the corresponding space vector
        u_g_abc = complex2abc(self.out.u_gs)
        return u_g_abc

    def post_process_with_inputs(self):
        """Post-process data with inputs."""
        data, par = self.data, self.par
        data.u_gs = (par.L_fg*data.e_gs+par.L_g*data.u_fs+(par.R_g*par.L_fg-
                    par.R_fg*par.L_g)*data.i_gs)/(par.L_g+par.L_fg)
