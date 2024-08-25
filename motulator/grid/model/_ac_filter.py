"""
Grid and AC filter impedance models.

This module contains continuous-time models for subsystems comprising an AC 
filter and a grid impedance between the converter and grid voltage sources. The 
models are implemented with space vectors in stationary coordinates.

"""
from types import SimpleNamespace

from motulator.common.model import Subsystem
from motulator.common.utils import complex2abc
from motulator.grid.utils import FilterPars


# %%
class ACFilter(Subsystem):
    """
    Base class for AC-side filters.

    This provides a base class and wrapper for converter AC-side filters
    (`LFilter`, `LCLFilter`) and grid impedance. Calling this class returns the
    corresponding filter object depending on if a value for the filter
    capacitance `C_f` is given.

    Parameters
    ----------
    filter_par : FilterPars
        Filter model parameters.
    grid_par : GridPars
        Grid model parameters.
        
    """

    def __new__(cls, filter_par: FilterPars, _):
        if filter_par.C_f > 0:
            if filter_par.L_fg > 0:
                return super().__new__(LCLFilter)
            raise ValueError("L_fg must be specified for LCL filter.")
        return super().__new__(LFilter)

    def meas_currents(self):
        """
        Measure the converter phase currents.

        Returns
        -------
        i_c_abc : 3-tuple of floats
            Converter phase currents (A).

        """
        i_c_abc = complex2abc(self.state.i_cs)

        return i_c_abc

    def meas_pcc_voltages(self):
        """
        Measure the phase voltages at the point of common coupling (PCC).

        Returns
        -------
        u_g_abc : 3-tuple of floats
            Phase voltages at the PCC (V).

        """
        u_g_abc = complex2abc(self.out.u_gs)
        return u_g_abc


# %%
class LFilter(ACFilter):
    """
    Model of an L filter and an inductive-resistive grid.

    An L filter and an inductive-resistive grid, between the converter and grid 
    voltage sources, are modeled combining their inductances and series 
    resistances. The point-of-common-coupling (PCC) voltage between the L 
    filter and the grid impedance is calculated.

    Parameters
    ----------
    grid_par : GridPars
        Grid model parameters. The following parameters are needed:

            L_g : float
                Grid inductance (H).
            R_g : float, optional
                Series resistance (Ω). The default is 0.

    filter_par : FilterPars
        Filter model parameters. The following parameters are needed:

            L_fc : float
                Filter inductance (H).
            R_fc : float, optional
                Series resistance (Ω). The default is 0.

    """

    def __init__(self, filter_par, grid_par):
        super().__init__()
        self.par = SimpleNamespace(
            L_f=filter_par.L_fc,
            R_f=filter_par.R_fc,
            L_g=grid_par.L_g,
            R_g=grid_par.R_g)
        self.inp = SimpleNamespace(u_cs=0j, e_gs=grid_par.u_gN + 0j)
        # For direct feedthrough
        self.out = SimpleNamespace(u_gs=grid_par.u_gN + 0j)
        self.state = SimpleNamespace(i_cs=0j)
        self.sol_states = SimpleNamespace(i_cs=[])

    def set_outputs(self, _):
        """Set output variables."""
        state, par, inp, out = self.state, self.par, self.inp, self.out
        u_gs = (
            par.L_g*inp.u_cs + par.L_f*inp.e_gs +
            (par.R_g*par.L_f - par.R_f*par.L_g)*state.i_cs)/(
                par.L_g + par.L_f)
        out.i_cs, out.i_gs, out.u_gs = state.i_cs, state.i_cs, u_gs

    def rhs(self):
        """Compute the state derivatives."""
        state, inp, par = self.state, self.inp, self.par
        L_t = par.L_f + par.L_g
        R_t = par.R_f + par.R_g
        d_i_cs = (inp.u_cs - inp.e_gs - R_t*state.i_cs)/L_t
        return [d_i_cs]

    def post_process_states(self):
        """Post-process data."""
        self.data.i_gs = self.data.i_cs

    def post_process_with_inputs(self):
        """Post-process data with inputs."""
        data = self.data
        data.u_gs = (
            self.par.L_g*data.u_cs + self.par.L_f*data.e_gs +
            (self.par.R_g*self.par.L_f - self.par.R_f*self.par.L_g)*
            data.i_cs)/(self.par.L_g + self.par.L_f)


# %%
class LCLFilter(ACFilter):
    """
    Model of an LCL filter and an inductive-resistive grid.

    An LCL filter and an inductive-resistive grid impedance, between the 
    converter and grid voltage sources, are modeled. The point-of-common-
    coupling (PCC) voltage between the LCL filter and the grid impedance is 
    also calculated.

    Parameters
    ----------
    grid_par : GridPars
        Grid model parameters. 
    filter_par : FilterPars
        Filter model parameters.

    """

    def __init__(self, filter_par, grid_par):
        super().__init__()
        self.par = SimpleNamespace(
            L_fc=filter_par.L_fc,
            R_fc=filter_par.R_fc,
            L_fg=filter_par.L_fg,
            R_fg=filter_par.R_fg,
            C_f=filter_par.C_f,
            L_g=grid_par.L_g,
            R_g=grid_par.R_g,
        )
        self.inp = SimpleNamespace(u_cs=0 + 0j, e_gs=grid_par.u_gN + 0j)
        self.out = SimpleNamespace(u_gs=grid_par.u_gN + 0j)
        self.state = SimpleNamespace(
            i_cs=0j,
            u_fs=grid_par.u_gN + 0j,
            i_gs=0j,
        )
        self.sol_states = SimpleNamespace(i_cs=[], u_fs=[], i_gs=[])

    def set_outputs(self, _):
        """Set output variables."""
        state, par, inp, out = self.state, self.par, self.inp, self.out
        u_gs = (
            par.L_fg*inp.e_gs + par.L_g*state.u_fs +
            (par.R_g*par.L_fg - par.R_fg*par.L_g)*state.i_gs)/(
                par.L_g + par.L_fg)
        out.i_cs, out.u_fs, out.i_gs, out.u_gs = (
            state.i_cs, state.u_fs, state.i_gs, u_gs)

    def rhs(self):
        """Compute the state derivatives."""
        state, par, inp = self.state, self.par, self.inp
        # Total inductance and resistance
        L_t = par.L_fg + par.L_g
        R_t = par.R_fg + par.R_g
        # State equations
        d_i_cs = (inp.u_cs - state.u_fs - par.R_fc*state.i_cs)/par.L_fc
        d_u_fs = (state.i_cs - state.i_gs)/par.C_f
        d_i_gs = (state.u_fs - inp.e_gs - R_t*state.i_gs)/L_t

        return [d_i_cs, d_u_fs, d_i_gs]

    def meas_grid_currents(self):
        """
        Measure the grid phase currents.

        Returns
        -------
        i_g_abc : 3-tuple of floats
            Grid phase currents (A).

        """
        i_g_abc = complex2abc(self.state.i_gs)
        return i_g_abc

    def meas_capacitor_voltages(self):
        """
        Measure the capacitor phase voltages.

        Returns
        -------
        u_f_abc : 3-tuple of floats
            Phase voltages of the filter capacitor (V).

        """
        u_f_abc = complex2abc(self.state.u_fs)
        return u_f_abc

    def post_process_with_inputs(self):
        """Post-process data with inputs."""
        data, par = self.data, self.par
        data.u_gs = (
            par.L_fg*data.e_gs + par.L_g*data.u_fs +
            (par.R_g*par.L_fg - par.R_fg*par.L_g)*data.i_gs)/(
                par.L_g + par.L_fg)
