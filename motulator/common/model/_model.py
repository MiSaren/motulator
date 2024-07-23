"""Common functions and classes for continuous-time system models."""

from abc import ABC, abstractmethod
from types import SimpleNamespace

import numpy as np

from motulator.common.utils import abc2complex, complex2abc


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
    u_dc : float | callable
        DC-bus voltage (V).
    C_dc : float, optional
        DC-bus capacitance (F). Default is None.
    G_dc : float, optional
        Parallel conductance of the DC-bus capacitor (S). Default value is 0.
    i_ext : callable, optional
        External DC current, seen as disturbance, `i_ext(t)`. Default is zero,
        ``lambda t: 0``.

    """

    def __init__(self, u_dc, C_dc=None, G_dc=0, i_ext=lambda t: 0):
        super().__init__()
        self.i_ext = i_ext
        self.par = SimpleNamespace(u_dc=u_dc, C_dc=C_dc, G_dc=G_dc)
        # Initial values
        self.u_dc0 = u_dc(0) if callable(u_dc) else u_dc
        if C_dc is not None: # Only initialize states if dynamic DC model is used
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
        data.i_dc = 1.5*np.real(data.q_cs*np.conj(data.i_cs))


# %%
# TODO: implement diode bridge as a separate subsystem and remove this class
class FrequencyConverter(Subsystem):
    """
    Frequency converter.

    This extends the Inverter class with models for a strong grid, a
    three-phase diode-bridge rectifier, an LC filter.

    Parameters
    ----------
    L : float
        DC-bus inductance (H).
    C : float
        DC-bus capacitance (F).
    U_g : float
        Grid voltage (V, line-line, rms).
    f_g : float
        Grid frequency (Hz).

    """

    def __init__(self, L, C, U_g, f_g):
        super().__init__()
        self.par = SimpleNamespace(
            L=L, C=C, w_g=2*np.pi*f_g, u_g=np.sqrt(2/3)*U_g)
        self.state = SimpleNamespace(u_dc=np.sqrt(2)*U_g, i_L=0)
        self.inp = SimpleNamespace(q_cs=None, i_cs=0j)
        self.sol_states = SimpleNamespace(u_dc=[], i_L=[])
        self.sol_q_cs = []

    @property
    def u_dc(self):
        """DC-bus voltage."""
        return self.state.u_dc.real

    @property
    def u_cs(self):
        """AC-side voltage (V)."""
        return self.inp.q_cs*self.u_dc

    @property
    def i_dc(self):
        """DC-side current (A)."""
        return 1.5*np.real(self.inp.q_cs*np.conj(self.inp.i_cs))

    def grid_voltages(self, t):
        """
        Compute three-phase grid voltages.

        Parameters
        ----------
        t : float
            Time (s).

        Returns
        -------
        u_g_abc : ndarray of floats, shape (3,)
            Phase voltages (V).

        """
        theta_g = self.par.w_g*t
        u_g_abc = self.par.u_g*np.array([
            np.cos(theta_g),
            np.cos(theta_g - 2*np.pi/3),
            np.cos(theta_g - 4*np.pi/3)
        ])
        return u_g_abc

    def set_outputs(self, t):
        """Set output variables."""
        self.out.u_cs = self.u_cs
        self.out.u_dc, self.out.i_L = self.state.u_dc.real, self.state.i_L.real
        self.out.i_dc = self.i_dc.real
        # Grid phase voltages
        self.out.u_g_abc = self.grid_voltages(t)

    def rhs(self):
        """
        Compute the state derivatives.

        Returns
        -------
        complex list, length 2
            Time derivative of the complex state vector, [d_u_dc, d_i_L].

        """
        state, out, par = self.state, self.out, self.par
        # Output voltage of the diode bridge
        u_di = np.amax(out.u_g_abc, axis=0) - np.amin(out.u_g_abc, axis=0)
        # State derivatives
        d_u_dc = (state.i_L - self.i_dc)/par.C
        d_i_L = (u_di - self.u_dc)/par.L
        # The inductor current cannot be negative due to the diode bridge
        if state.i_L < 0 and d_i_L < 0:
            d_i_L = 0

        return [d_u_dc, d_i_L]

    def meas_dc_voltage(self):
        """Measure the DC-bus voltage."""
        return self.u_dc

    def post_process_states(self):
        """Post-process data."""
        data = self.data
        data.u_dc, data.i_L = data.u_dc.real, data.i_L.real
        data.u_cs = data.q_cs*data.u_dc

    def post_process_with_inputs(self):
        """Post-process data with inputs."""
        data = self.data
        data.i_dc = 1.5*np.real(data.q_cs*np.conj(data.i_cs))
        data.u_g_abc = self.grid_voltages(data.t)
        data.u_g = abc2complex(data.u_g_abc)
        # Voltage at the output of the diode bridge
        data.u_di = (
            np.amax(data.u_g_abc, axis=0) - np.amin(data.u_g_abc, axis=0))
        # Diode bridge switching states (-1, 0, 1)
        data.q_g_abc = (
            (np.amax(data.u_g_abc, axis=0) == data.u_g_abc).astype(int) -
            (np.amin(data.u_g_abc, axis=0) == data.u_g_abc).astype(int))
        # Grid current space vector
        data.i_g = abc2complex(data.q_g_abc)*data.i_L


class DiodeBridge(Subsystem):
    """
    Three-phase diode bridge.

    A three-phase diode bridge rectifier with a DC-side inductor is modeled.
    The inductor current i_L is used as a state variable.

    Parameters
    ----------
    L : float
        DC-bus inductance (H).

    """

    def __init__(self, L):
        super().__init__()
        self.par = SimpleNamespace(L=L)
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
        state, inp, out, par = self.state, self.inp, self.out, self.par
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
