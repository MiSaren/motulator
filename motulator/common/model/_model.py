"""Common functions and classes for continuous-time system models."""

from abc import ABC, abstractmethod
from types import SimpleNamespace

import numpy as np

from motulator.common.utils import abc2complex, complex2abc, DCBusPars, FilterPars
from motulator.grid.utils import GridPars

# TODO: divide contents of this file to multiple files, e.g., _converter.py
# and _ac_filter.py. Before doing so, a way to resolve Sphinx cyclic imports
# should be found, however.
# %%
class Delay:
    """
    Computational delay modeled as a ring buffer.

    Parameters
    ----------
    length : int, optional
        Length of the buffer in samples. The default is 1.

    """

    def __init__(self, length=1, elem=3):
        self.data = length*[elem*[0]]  # Creates a zero list

    def __call__(self, u):
        """
        Delay the input.

        Parameters
        ----------
        u : array_like, shape (elem,)
            Input array.

        Returns
        -------
        array_like, shape (elem,)
            Output array.

        """
        # Add the latest value to the end of the list
        self.data.append(u)
        # Pop the first element and return it
        return self.data.pop(0)


# %%
class CarrierComparison:
    """
    Carrier comparison.

    This computes the the switching states and their durations based on the
    duty ratios. Instead of searching for zero crossings, the switching
    instants are explicitly computed in the beginning of each sampling period,
    allowing faster simulations.

    Parameters
    ----------
    N : int, optional
        Amount of the counter quantization levels. The default is 2**12.
    return_complex : bool, optional
        Complex switching state space vectors are returned if True. Otherwise
        phase switching states are returned. The default is True.

    Examples
    --------
    >>> from motulator.common.model import CarrierComparison
    >>> carrier_cmp = CarrierComparison(return_complex=False)
    >>> # First call gives rising edges
    >>> t_steps, q_c_abc = carrier_cmp(1e-3, [.4, .2, .8])
    >>> # Durations of the switching states
    >>> t_steps
    array([0.00019995, 0.00040015, 0.00019995, 0.00019995])
    >>> # Switching states
    >>> q_c_abc
    array([[0, 0, 0],
           [0, 0, 1],
           [1, 0, 1],
           [1, 1, 1]])
    >>> # Second call gives falling edges
    >>> t_steps, q_c_abc = carrier_cmp(.001, [.4, .2, .8])
    >>> t_steps
    array([0.00019995, 0.00019995, 0.00040015, 0.00019995])
    >>> q_c_abc
    array([[1, 1, 1],
           [1, 0, 1],
           [0, 0, 1],
           [0, 0, 0]])
    >>> # Sum of the step times equals T_s
    >>> np.sum(t_steps)
    0.001
    >>> # 50% duty ratios in all phases
    >>> t_steps, q_c_abc = carrier_cmp(1e-3, [.5, .5, .5])
    >>> t_steps
    array([0.0005, 0.    , 0.    , 0.0005])
    >>> q_c_abc
    array([[0, 0, 0],
           [0, 0, 0],
           [0, 0, 0],
           [1, 1, 1]])

    """

    def __init__(self, N=2**12, return_complex=True):
        self.N = N
        self.return_complex = return_complex
        self._rising_edge = True  # Stores the carrier direction

    def __call__(self, T_s, d_c_abc):
        """
        Compute the switching state durations and vectors.

        Parameters
        ----------
        T_s : float
            Half carrier period (s).
        d_c_abc : array_like of floats, shape (3,)
            Duty ratios in the range [0, 1].

        Returns
        -------
        t_steps : ndarray, shape (4,)
            Switching state durations (s), `[t0, t1, t2, t3]`.
        q_cs : complex ndarray, shape (4,)
            Switching state vectors, `[q0, q1, q2, q3]`, where `q1` and `q2`
            are active vectors.

        Notes
        -----
        No switching (e.g. `d_a == 0` or `d_a == 1`) or simultaneous switching
        (e.g. `d_a == d_b`) lead to zeroes in `t_steps`.

        """
        # Quantize the duty ratios to N levels
        d_c_abc = np.round(self.N*np.asarray(d_c_abc))/self.N

        # Assume falling edge and compute the normalized switching instants:
        t_n = np.append(0, np.sort(d_c_abc))
        # Compute the corresponding switching states:
        q_c_abc = (t_n[:, np.newaxis] < d_c_abc).astype(int)

        # Durations of switching states
        t_steps = T_s*np.diff(t_n, append=1)

        # Flip the sequence if rising edge
        if self._rising_edge:
            t_steps = np.flip(t_steps)
            q_c_abc = np.flipud(q_c_abc)

        # Change the carrier direction for the next call
        self._rising_edge = not self._rising_edge

        return ((t_steps, abc2complex(q_c_abc.T)) if self.return_complex else
                (t_steps, q_c_abc))


# %%
def zoh(T_s, d_c_abc):
    """
    Zero-order hold of the duty ratios over the sampling period.

    Parameters
    ----------
    T_s : float
        Sampling period.
    d_c_abc : array_like of floats, shape (3,)
        Duty ratios in the range [0, 1].

    Returns
    -------
    t_steps : ndarray, shape (1,)
        Sampling period as an array compatible with the solver.
    q_cs : complex ndarray, shape (1,)
        Duty ratio vector as an array compatible with the solver.

    """
    # Shape the output arrays to be compatible with the solver
    t_steps = np.array([T_s])
    q_cs = np.array([abc2complex(d_c_abc)])
    return t_steps, q_cs


# %%
class Model(ABC):
    """
    Base class for continuous-time system models.

    This base class is a template for a system model that interconnects the 
    subsystems and provides an interface to the solver. 

    Parameters
    ----------
    pwm : zoh | CarrierComparison, optional
        Zero-order hold of duty ratios or carrier comparison. If None, the 
        default is `zoh`.
    delay : int, optional
        Amount of computational delays. The default is 1.

    """

    def __init__(self, pwm=None, delay=1):
        self.delay = Delay(delay)
        self.pwm = zoh if pwm is None else pwm
        self.t0 = 0
        self.converter = None
        self.subsystems = []  # Contains the list of subsystems
        self.sol_t = []

    def get_initial_values(self):
        """Get initial values of all subsystems before the solver."""
        state0 = []
        for subsystem in self.subsystems:
            if hasattr(subsystem, "state"):
                state0 += list(vars(subsystem.state).values())

        return state0

    def set_states(self, state_list):
        """Set the states in all subsystems."""
        index = 0
        for subsystem in self.subsystems:
            if hasattr(subsystem, "state"):
                for attr in vars(subsystem.state):
                    setattr(subsystem.state, attr, state_list[index])
                    index += 1

    # TODO: set_outputs() and set_inputs() could be combined into a single method
    def set_outputs(self, t):
        """Compute the output variables."""
        for subsystem in self.subsystems:
            if hasattr(subsystem, "set_outputs"):
                subsystem.set_outputs(t)

    def set_inputs(self, t):
        """Compute the input variables."""
        for subsystem in self.subsystems:
            if hasattr(subsystem, "set_inputs"):
                subsystem.set_inputs(t)

    @abstractmethod
    def interconnect(self, t):
        """Interconnect the subsystems."""

    def rhs(self, t, state_list):
        """Compute the complete state derivative list for the solver."""
        # Get the states from the list and set them to the subsystems
        self.set_states(state_list)

        # Set the outputs for the interconnections and for the rhs
        self.set_outputs(t)

        # Set the inputs for the interconnections and for the rhs
        self.set_inputs(t)

        # Interconnections
        self.interconnect(t)

        # State derivatives
        rhs_list = []
        for subsystem in self.subsystems:
            if hasattr(subsystem, "rhs"):
                subsystem_rhs = subsystem.rhs()
                if subsystem_rhs is not None:
                    rhs_list += subsystem_rhs

        # List of state derivatives
        return rhs_list

    def save(self, sol):
        """Save the solution."""
        self.sol_t.extend(sol.t)
        self.converter.sol_q_cs.extend(sol.q_cs)

        index = 0
        for subsystem in self.subsystems:
            if hasattr(subsystem, "sol_states"):
                for attr in vars(subsystem.sol_states):
                    subsystem.sol_states.__dict__[attr].extend(sol.y[index])
                    index += 1

    def post_process_states(self):
        """Transform the lists to the ndarray format and post-process them."""
        self.converter.data.q_cs = np.asarray(self.converter.sol_q_cs)

        for subsystem in self.subsystems:
            subsystem.data.t = np.asarray(self.sol_t)
            if hasattr(subsystem, "sol_states"):
                for key, value in vars(subsystem.sol_states).items():
                    setattr(subsystem.data, key, np.asarray(value))

            if hasattr(subsystem, "post_process_states"):
                subsystem.post_process_states()

    def post_process_with_inputs(self):
        """Post-process after the inputs have been added."""
        for subsystem in self.subsystems:
            if hasattr(subsystem, "post_process_with_inputs"):
                subsystem.post_process_with_inputs()


# %%
class Subsystem(ABC):
    """Base class for subsystems."""

    def __init__(self):
        # States, inputs, and outputs
        self.state = SimpleNamespace()
        self.inp = SimpleNamespace()
        self.out = SimpleNamespace()
        # Save the solution in these lists
        self.sol_states = SimpleNamespace()
        # For post-processed data
        self.data = SimpleNamespace()


# %%
class Inverter(Subsystem):
    """
    Lossless three-phase voltage source inverter.

    Capacitive DC-bus dynamics are modeled if C_dc is given as
    a parameter. In this case, the capacitor voltage u_dc is used as
    a state variable. Otherwise the DC voltage is constant or a function of
    time depending on the type of parameter u_dc.

    Parameters
    ----------
    dc_bus_par : DCBusPars
        DC-bus parameters.
    i_ext : callable, optional
        External DC current, seen as disturbance, `i_ext(t)`. Default is zero,
        ``lambda t: 0``.

    Inventer model uses following fields of the DCbusPas object:
    
    dc_bus_par.u_dc : float | callable
        DC-bus voltage (V).
    dc_bus_par.C_dc : float, optional
        DC-bus capacitance (F). Default is None.
    dc_bus_par.G_dc : float, optional
        Parallel conductance of the DC-bus capacitor (S). Default value is 0.
    
    """

    def __init__(self, dc_bus_par : DCBusPars, i_ext=lambda t: 0):
        super().__init__()
        self.i_ext = i_ext
        self.par = SimpleNamespace(u_dc=dc_bus_par.u_dc, C_dc=dc_bus_par.C_dc, G_dc=dc_bus_par.G_dc)
        # Initial values
        self.u_dc0 = self.par.u_dc(0) if callable(self.par.u_dc) else self.par.u_dc
        if self.par.C_dc is not None: # Only initialize states if dynamic DC model is used
            self.state = SimpleNamespace(u_dc = self.u_dc0)
            self.sol_states = SimpleNamespace(u_dc = [])
        self.inp = SimpleNamespace(
            u_dc=self.u_dc0, i_ext=i_ext(0), q_cs=None, i_cs=0j)
        self.sol_q_cs = []

    @property
    def u_dc(self):
        """DC-bus voltage (V)."""
        if self.par.C_dc is not None:
            return self.state.u_dc.real
        return self.inp.u_dc

    @property
    def u_cs(self):
        """AC-side voltage (V)."""
        return self.inp.q_cs*self.u_dc

    @property
    def i_dc(self):
        """DC-side current (A)."""
        return 1.5*np.real(self.inp.q_cs*np.conj(self.inp.i_cs))

    def set_outputs(self, _):
        """Set output variables."""
        self.out.u_cs = self.u_cs
        self.out.u_dc = self.u_dc

    def set_inputs(self, t):
        """Set input variables."""
        self.inp.u_dc = self.par.u_dc(t) if callable(
            self.par.u_dc) else self.par.u_dc
        self.inp.i_ext = self.i_ext(t)

    def rhs(self):
        """
        Compute the state derivatives.

        Returns
        -------
        complex list, length 1
            Time derivative of the complex state vector, [d_u_dc].

        """
        state, inp, par = self.state, self.inp, self.par
        if par.C_dc is None: # Check whether dynamic DC model is used
            return None
        d_u_dc = (inp.i_ext - self.i_dc - par.G_dc*state.u_dc)/par.C_dc
        return [d_u_dc]

    def meas_dc_voltage(self):
        """
        Measure the converter DC-bus voltage.

        Returns
        -------
        u_dc : float
            DC-bus voltage (V).

        """
        return self.u_dc

    def post_process_states(self):
        """Post-process data."""
        data = self.data
        if self.par.C_dc is None:
            if callable(self.u_dc):
                self.data.u_dc = self.u_dc(self.data.t)
            else:
                self.data.u_dc = np.full(np.size(self.data.t), self.u_dc)
        else:
            data.u_dc = data.u_dc.real
        data.u_cs = data.q_cs*data.u_dc


# %%
class DiodeBridge(Subsystem):
    """
    Three-phase diode bridge.

    A three-phase diode bridge rectifier with a DC-side inductor is modeled.
    The inductor current i_L is used as a state variable.

    Parameters
    ----------
    dc_bus_par : DCBusPars
        DC-bus parameters. Using on the following field:

            dc_bus_par.L_dc : float
                DC-bus inductance (H).

    """

    def __init__(self, dc_bus_par : DCBusPars):
        super().__init__()
        self.par = SimpleNamespace(L=dc_bus_par.L_dc)
        self.state = SimpleNamespace(i_L=0)
        self.sol_states = SimpleNamespace(i_L=[])

    def set_outputs(self, _):
        """Set output variables."""
        self.out.i_L = self.state.i_L.real

    def rhs(self):
        """
        Compute the state derivatives.

        Returns
        -------
        complex list, length 1
            Time derivative of the complex state vector, [d_i_L].

        """
        state, inp, par = self.state, self.inp, self.par
        # Output voltage of the diode bridge
        u_g_abc = complex2abc(inp.u_gs)
        u_di = np.amax(u_g_abc, axis=0) - np.amin(u_g_abc, axis=0)
        # State derivatives
        d_i_L = (u_di - inp.u_dc)/par.L
        # The inductor current cannot be negative due to the diode bridge
        if state.i_L < 0 and d_i_L < 0:
            d_i_L = 0

        return [d_i_L]

    def post_process_states(self):
        """Post-process data."""
        self.data.i_L = self.data.i_L.real

    def post_process_with_inputs(self):
        """Post-process data with inputs."""
        data = self.data
        data.u_g_abc = complex2abc(data.u_gs)
        # Voltage at the output of the diode bridge
        data.u_di = (
            np.amax(data.u_g_abc, axis=0) - np.amin(data.u_g_abc, axis=0))
        # Diode bridge switching states (-1, 0, 1)
        data.q_g_abc = (
            (np.amax(data.u_g_abc, axis=0) == data.u_g_abc).astype(int) -
            (np.amin(data.u_g_abc, axis=0) == data.u_g_abc).astype(int))
        # Grid current space vector
        data.i_gs = abc2complex(data.q_g_abc)*data.i_L


# %%
class ACFilter(Subsystem):
    """
    Base class for converter AC-side filters.

    This provides a base class and wrapper for converter AC-side filters
    (LFilter, LCLFilter, LCFilter). Calling this class returns one of the three
    subclasses depending on whether values for filter capacitance C_f and
    filter grid-side inductance L_fg are included in the FilterPars object.

    Parameters
    ----------
    filter_par : FilterPars
        Filter model parameters.
    grid_par : GridPars, optional
        Grid model parameters.

    """

    def __new__(cls, filter_par:FilterPars, grid_par:GridPars=None):
        if filter_par.C_f != 0:
            if filter_par.L_fg != 0:
                return super().__new__(LCLFilter)
            return super().__new__(LCFilter)
        return super().__new__(LFilter)

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
            Phase voltages of the filter capacitor (V).

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


# %%
class LFilter(ACFilter):
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
        L-Filter model uses only the following FilterPars parameters:

            L_fc : float
                Converter-side inductance of the filter (H).
            R_fc : float (optional)
                Converter-side series resistance (Ω). The default is 0.

    """

    def __init__(self, filter_par:FilterPars, grid_par:GridPars):
        super().__init__()
        self.par = SimpleNamespace(
            L_f=filter_par.L_fc,
            R_f=filter_par.R_fc,
            L_g=grid_par.L_g,
            R_g=grid_par.R_g
            )
        self.inp = SimpleNamespace(u_cs=0+0j, e_gs=grid_par.u_gN+0j)
        self.out = SimpleNamespace(u_gs=grid_par.u_gN+0j)  # Needed for direct feedthrough
        self.state = SimpleNamespace(i_cs=0+0j)
        self.sol_states = SimpleNamespace(i_cs=[])

    def set_outputs(self, _):
        """Set output variables."""
        state, par, inp, out = self.state, self.par, self.inp, self.out
        u_gs = (par.L_g*inp.u_cs + par.L_f*inp.e_gs +
            (par.R_g*par.L_f - par.R_f*par.L_g)*
            state.i_cs)/(par.L_g+par.L_f)
        out.i_cs, out.i_gs, out.u_gs = state.i_cs, state.i_cs, u_gs

    def rhs(self):
        """
        Compute the state derivatives.

        Returns
        -------
        complex list, length 1
            Time derivative of the complex state vector, [d_i_cs].

        """
        state, inp, par = self.state, self.inp, self.par
        L_t = par.L_f + par.L_g
        R_t = par.R_f + par.R_g
        d_i_cs = (inp.u_cs - inp.e_gs - R_t*state.i_cs)/L_t
        return [d_i_cs]

    def post_process_states(self):
        """Post-process data."""
        self.data.i_gs=self.data.i_cs

    def post_process_with_inputs(self):
        """Post-process data with inputs."""
        data = self.data
        data.u_gs = (self.par.L_g*data.u_cs + self.par.L_f*data.e_gs +
            (self.par.R_g*self.par.L_f - self.par.R_f*self.par.L_g)*
            data.i_cs)/(self.par.L_g+self.par.L_f)


# %%
class LCLFilter(ACFilter):
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


    def __init__(self, filter_par:FilterPars, grid_par:GridPars):
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

    def post_process_with_inputs(self):
        """Post-process data with inputs."""
        data, par = self.data, self.par
        data.u_gs = (par.L_fg*data.e_gs+par.L_g*data.u_fs+(par.R_g*par.L_fg-
                    par.R_fg*par.L_g)*data.i_gs)/(par.L_g+par.L_fg)


# %%
class LCFilter(ACFilter):
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

    def __init__(self, filter_par: FilterPars):
        super().__init__()
        self.par = SimpleNamespace(
            L_fc=filter_par.L_fc,
            C_f=filter_par.C_f,
            R_fc=filter_par.R_fc,
            G_f=filter_par.G_f)
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
