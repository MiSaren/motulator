"""
Power converter models.

An inverter with constant DC-bus voltage or optional dynamic DC model and
a frequency converter with a diode front-end rectifier are modeled. Complex
space vectors are used also for duty ratios and switching states, wherever
applicable. 

"""
from types import SimpleNamespace

import numpy as np

from motulator.common.model import Subsystem
from motulator.common.utils import abc2complex


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
        if self.par.C_dc is None:
            self.data.u_dc = self.par.u_dc(self.data.t) if callable(
                self.par.u_dc) else self.par.u_dc
        else:
            self.data.u_dc = self.data.u_dc.real
        self.data.u_cs = self.data.q_cs*self.data.u_dc


# %%
# TODO: not used anymore, can be removed
class InverterWithVariableDC(Subsystem):
    """
    Lossless three-phase inverter with variable DC-bus voltage. This extends the
    Inverter class

    Parameters
    ----------
    u_dc0 : float
        DC-bus initial voltage (V).
    """


    def __init__(self):
        super().__init__()
        self.inp = SimpleNamespace(u_dc=0, q_cs=None, i_cs=0j)
        self.sol_q_cs = []

    @property
    def i_dc(self):
        """DC-side current (A)."""
        return 1.5*np.real(self.inp.q_cs*np.conj(self.inp.i_cs))
    
    @property
    def u_cs(self):
        """AC-side voltage (V)."""
        return self.inp.q_cs*self.inp.u_dc

    def set_outputs(self, t):
        """Set output variables."""
        self.out.u_cs = self.u_cs
        self.out.i_dc = self.i_dc

    def meas_dc_voltage(self):
        """Measure the DC-bus voltage."""
        return self.inp.u_dc

    def ac_voltage(self, q_cs, u_dc):
        """Compute the AC voltage."""
        return q_cs*u_dc

    def dc_current(self, q_cs, i_cs):
        """Compute the DC current."""
        return 1.5*np.real(q_cs*np.conj(i_cs))


# %%
# TODO: implement diode bridge as a separate subsystem
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
