"""
Continuous-time models for converters.

A three-phase voltage-source inverter with optional DC-bus dynamics is
modelled, along with a six-pulse diode bridge rectifier supplied from a stiff
grid. Complex space vectors are used also for duty ratios and switching states,
wherever applicable.

"""
from types import SimpleNamespace

import numpy as np

from motulator.common.model import Subsystem
from motulator.common.utils import abc2complex, complex2abc


# %%
class VoltageSourceConverter(Subsystem):
    """
    Lossless three-phase voltage-source converter.

    Parameters
    ----------
    u_dc : float
        DC-bus voltage (V). If the DC-bus capacitor is modeled, this value is
        used as the initial condition.
    C_dc : float, optional
        DC-bus capacitance (F). The default is None.
    i_dc : callable, optional
        External current (A) fed to the DC bus. Needed if `C_dc` is not None.

    """

    def __init__(self, u_dc, C_dc=None, i_dc=lambda t: None):
        super().__init__()
        self.par = SimpleNamespace(u_dc=u_dc, C_dc=C_dc)
        self.inp = SimpleNamespace(q_cs=None, i_cs=0j)
        if C_dc is not None:
            self.state = SimpleNamespace(u_dc=u_dc)
            self.sol_states = SimpleNamespace(u_dc=[])
            self.i_dc = i_dc
            self.inp.i_dc = i_dc(0)
        self.sol_q_cs = []

    @property
    def u_dc(self):
        """DC-bus voltage (V)."""
        if self.par.C_dc is not None:
            return self.state.u_dc.real
        return self.par.u_dc

    @property
    def u_cs(self):
        """AC-side voltage (V)."""
        return self.inp.q_cs*self.u_dc

    @property
    def i_dc_int(self):
        """Converter-side DC current (A)."""
        return 1.5*np.real(self.inp.q_cs*np.conj(self.inp.i_cs))

    def set_outputs(self, _):
        """Set output variables."""
        self.out.u_cs = self.u_cs
        self.out.u_dc = self.u_dc

    def set_inputs(self, t):
        """Set input variables."""
        if self.par.C_dc is not None:
            self.inp.i_dc = self.i_dc(t)

    def rhs(self):
        """Compute the state derivatives."""
        if self.par.C_dc is not None:
            d_u_dc = (self.inp.i_dc - self.i_dc_int)/self.par.C_dc
            return [d_u_dc]
        return []

    def meas_dc_voltage(self):
        """Measure the converter DC-bus voltage (V)."""
        return self.u_dc

    def post_process_states(self):
        """Post-process data."""
        data = self.data
        if self.par.C_dc is not None:
            data.u_dc = data.u_dc.real
        else:
            # TODO: fix (u_dc is property, not attribute)
            if callable(self.u_dc):
                self.data.u_dc = self.u_dc(self.data.t)
            else:
                self.data.u_dc = np.full(np.size(self.data.t), self.u_dc)
        data.u_cs = data.q_cs*data.u_dc

    def post_process_with_inputs(self):
        """Post-process data with inputs."""
        data = self.data
        data.i_dc_int = 1.5*np.real(data.q_cs*np.conj(data.i_cs))


class ThreeLevelConverter(VoltageSourceConverter):
    """
    Lossless three-phase, three-level voltage-source converter.

    Parameters
    ----------
    u_dc : float
        DC-bus voltage (V). If the DC-bus capacitors are modeled, this value is
        used as the initial condition.
    C_dc1 : float, optional
        DC-bus capacitance (F), positive side. The default is None.
    C_dc2 : float, optional
        DC-bus capacitance (F), negative side. The default is None.
    i_dc : callable, optional
        External current (A) fed to the DC bus. The default is 0.
    G_dc : float, optional
        DC-bus conductance (S). The default is 0.

    """

    def __init__(self, u_dc, C_dc1=None, C_dc2=None, i_dc=lambda t: 0, G_dc=0):
        super().__init__(u_dc)
        self.par.C_dc1 = C_dc1
        self.par.C_dc2 = C_dc2
        self.par.G_dc = G_dc
        if not (C_dc1 is None or C_dc2 is None):
            self.state = SimpleNamespace(u_dc1=u_dc/2, u_dc2=u_dc/2)
            self.sol_states = SimpleNamespace(u_dc1=[], u_dc2=[])
            self.inp.i_dc = i_dc(0)
        self.i_dc = i_dc
        self.inp.q_P = [0, 0, 0]
        self.inp.q_o = [0, 0, 0]

    @property
    def u_dc(self):
        """DC-bus voltage (V)."""
        if not (self.par.C_dc1 is None or self.par.C_dc2 is None):
            return self.state.u_dc1.real + self.state.u_dc2.real
        return self.par.u_dc

    @property
    def u_cs(self):
        """AC-side voltage (V)."""
        #TODO: remove q_P, q_o and do comparison to inp.q_cs directly?
        inp = self.inp
        if not (self.par.C_dc1 is None or self.par.C_dc2 is None):
            u_dc1 = self.state.u_dc1
            u_dc2 = self.state.u_dc2
        else:
            u_dc1 = u_dc2 = self.par.u_dc/2
        u_aN = ((inp.q_P[0]) + (inp.q_o[0]))*u_dc1 + inp.q_o[0]*u_dc2
        u_bN = ((inp.q_P[1]) + (inp.q_o[1]))*u_dc1 + inp.q_o[1]*u_dc2
        u_cN = ((inp.q_P[2]) + (inp.q_o[2]))*u_dc1 + inp.q_o[2]*u_dc2
        return abc2complex([u_aN, u_bN, u_cN])

    @property
    def i_P(self):
        """Converter-side positive DC current (A)."""
        i_abc = complex2abc(self.inp.i_cs)
        q_P = self.inp.q_P
        return q_P[0]*i_abc[0] + q_P[1]*i_abc[1] + q_P[2]*i_abc[2]

    @property
    def i_o(self):
        """Converter-side neutral-point current (A)."""
        i_abc = complex2abc(self.inp.i_cs)
        q_o = self.inp.q_o
        return q_o[0]*i_abc[0] + q_o[1]*i_abc[1] + q_o[2]*i_abc[2]

    def set_outputs(self, _):
        """Set output variables."""
        self.out.u_cs = self.u_cs
        self.out.u_dc = self.u_dc

    def set_inputs(self, t):
        """Set input variables."""
        # External DC-bus current
        self.inp.i_dc = self.i_dc(t)
        # Switching state vectors for DC-bus positive and neutral points
        q_abc = self.inp.q_cs
        self.inp.q_P = [q_abc[0] == 1, q_abc[1] == 1, q_abc[2] == 1]
        self.inp.q_o = [q_abc[0] == .5, q_abc[1] == .5, q_abc[2] == .5]

    def rhs(self):
        """Compute the state derivatives."""
        if not (self.par.C_dc1 is None or self.par.C_dc2 is None):
            d_u_dc1 = (
                self.inp.i_dc - self.i_P -
                self.par.G_dc*self.state.u_dc1)/self.par.C_dc1
            d_u_dc2 = (
                self.inp.i_dc - self.i_P - self.i_o -
                self.par.G_dc*self.u_dc)/self.par.C_dc2
            return [d_u_dc1, d_u_dc2]
        return []

    def post_process_states(self):
        """Post-process data."""
        data = self.data
        if not (self.par.C_dc1 is None or self.par.C_dc2 is None):
            data.u_dc1, data.u_dc2 = data.u_dc1.real, data.u_dc2.real
            data.u_dc = data.u_dc1 + data.u_dc2
        else:
            data.u_dc = np.full(np.size(self.data.t), self.u_dc)
            data.u_dc1 = data.u_dc2 = data.u_dc*0.5
        u_aN = ((data.q_cs[:, 0] == 1) +
                (data.q_cs[:, 0] == .5))*data.u_dc1 + (
                    data.q_cs[:, 0] == .5)*data.u_dc2
        u_bN = ((data.q_cs[:, 1] == 1) +
                (data.q_cs[:, 1] == .5))*data.u_dc1 + (
                    data.q_cs[:, 1] == .5)*data.u_dc2
        u_cN = ((data.q_cs[:, 2] == 1) +
                (data.q_cs[:, 2] == .5))*data.u_dc1 + (
                    data.q_cs[:, 2] == .5)*data.u_dc2
        data.u_cs = abc2complex([u_aN, u_bN, u_cN])

    def post_process_with_inputs(self):
        """Post-process data with inputs."""
        data = self.data
        i_abc = complex2abc(data.i_cs)
        data.i_P = (data.q_cs[:, 0] == 1)*i_abc[0] + (
            data.q_cs[:, 1] == 1)*i_abc[1] + (data.q_cs[:, 2] == 1)*i_abc[2]
        data.i_o = (data.q_cs[:, 0] == .5)*i_abc[0] + (
            data.q_cs[:, 1] == .5)*i_abc[1] + (data.q_cs[:, 2] == .5)*i_abc[2]


# %%
class FrequencyConverter(VoltageSourceConverter):
    """
    Frequency converter with a six-pulse diode bridge.

    A three-phase diode bridge rectifier with a DC-bus inductor is modeled. The
    diode bridge is connected to the voltage-source inverter. The inductance of
    the grid is omitted.

    Parameters
    ----------
    C_dc : float
        DC-bus capacitance (F).
    L_dc : float
        DC-bus inductance (H).
    U_g : float
        Grid voltage (V, line-line, rms).
    f_g : float
        Grid frequency (Hz).

    """

    def __init__(self, C_dc, L_dc, U_g, f_g):
        super().__init__(np.sqrt(2)*U_g, C_dc)
        self.par = SimpleNamespace(
            L_dc=L_dc, C_dc=C_dc, w_g=2*np.pi*f_g, u_g=np.sqrt(2/3)*U_g)
        self.state.i_L = 0
        self.state.exp_j_theta_g = complex(1)
        self.sol_states.i_L, self.sol_states.exp_j_theta_g = [], []

    def set_outputs(self, t):
        """Set output variables."""
        super().set_outputs(t)
        self.out.i_L = self.state.i_L.real

    def set_inputs(self, t):
        """Set output variables."""
        self.inp.i_dc = self.out.i_L
        self.inp.u_dc = self.out.u_dc

    def rhs(self):
        """Compute the state derivatives."""
        # Grid voltage
        u_gs = self.par.u_g*self.state.exp_j_theta_g
        u_g_abc = complex2abc(u_gs)
        # Output voltage of the diode bridge
        u_di = np.amax(u_g_abc, axis=0) - np.amin(u_g_abc, axis=0)
        # State derivatives
        d_exp_j_theta_g = 1j*self.par.w_g*self.state.exp_j_theta_g
        d_i_L = (u_di - self.inp.u_dc)/self.par.L_dc
        # The inductor current cannot be negative due to the diode bridge
        if self.state.i_L < 0 and d_i_L < 0:
            d_i_L = 0
        return super().rhs() + [d_i_L, d_exp_j_theta_g]

    def post_process_states(self):
        """Post-process data."""
        super().post_process_states()
        self.data.i_L = self.data.i_L.real

    def post_process_with_inputs(self):
        """Post-process data with inputs."""
        super().post_process_with_inputs()
        data = self.data
        data.u_gs = self.par.u_g*data.exp_j_theta_g
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


class DCPowerSource(Subsystem):
    """
    Constant DC power source.

    A constant DC power source is modeled as a voltage-controlled current
    source. The output power p_dc=u_dc*i_dc is kept constant.

    Parameters
    ----------
    p_dc : float | callable
        Output power (W).
    u_dc : float
        DC-bus voltage (V). Needed to set the initial value of current.
    
    """

    def __init__(self, p_dc, u_dc):
        super().__init__()
        self.p_dc = p_dc
        self.inp.p_dc = p_dc(0) if callable(p_dc) else p_dc
        self.inp.u_dc = u_dc

    def set_inputs(self, t):
        """Set input variables."""
        self.inp.p_dc = self.p_dc(t) if callable(self.p_dc) else self.p_dc

    def set_outputs(self, _):
        """Set output variables."""
        self.out.i_dc = self.inp.p_dc/self.inp.u_dc

    def post_process_direct_feedthrough(self):
        """Post-process direct feedthrough data."""
        if callable(self.p_dc):
            self.data.p_dc = self.p_dc(self.data.t)
        else:
            self.data.p_dc = np.full(np.size(self.data.t), self.p_dc)
        self.data.i_dc = self.data.p_dc/self.data.u_dc
