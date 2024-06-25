"""Common control functions and classes."""
from abc import ABC
from types import SimpleNamespace

import numpy as np
from motulator.common.control import (ControlSystem, PIController)
from motulator.common.utils import (abc2complex)


# %%
class DCBusVoltageController(PIController):
    """
    PI DC-bus voltage controller.

    This provides an interface for a DC-bus controller. The gains are
    initialized based on the desired closed-loop bandwidth and the DC-bus
    capacitance estimate. The PI controller is designed to control the energy
    of the DC-bus capacitance and not the DC-bus voltage in order to have a
    linear closed-loop system [#Hur2001]_.

    Parameters
    ----------
    zeta : float
        Damping ratio of the closed-loop system.
    alpha_dc : float
        Closed-loop bandwidth (rad/s). 
    p_max : float, optional
        Maximum converter power (W). The default is inf.
        
    References
    ----------
    .. [#Hur2001] Hur, Jung, Nam, "A Fast Dynamic DC-Link Power-Balancing
       Scheme for a PWM Converter–Inverter System," IEEE Trans. Ind. Electron.,
       2001, https://doi.org/10.1109/41.937412

    """

    def __init__(self, zeta, alpha_dc, p_max=np.inf):
        k_p = -2*zeta*alpha_dc
        k_i = -(alpha_dc**2)
        k_t = k_p
        super().__init__(k_p, k_i, k_t, p_max)


# %%
class GridConverterControlSystem(ControlSystem, ABC):
    """
    Base class for control of grid-connected converters.
    """
    def __init__(self, par, T_s, on_u_dc):
        super().__init__(T_s)
        self.par = par
        self.on_u_dc = on_u_dc
        self.dc_bus_volt_ctrl = None
        self.ref = SimpleNamespace()

    def get_electrical_measurements(self, fbk, mdl):
        """
        Measure the currents and voltages.
        
        Parameters
        ----------
        fbk : SimpleNamespace
            Measured signals are added to this object.
        mdl : Model
            Continuous-time system model.

        Returns
        -------
        fbk : SimpleNamespace
            Measured signals, containing the following fields:

                u_dc : float
                    DC-bus voltage (V).
                i_cs : complex
                    Converter current (A) in stator coordinates.
                u_cs : complex
                    Realized stator voltage (V) in stator coordinates. This
                    signal is obtained from the PWM.
                u_gs : complex
                    PCC voltage (V) in stator coordinates.

        """
        fbk.u_dc = mdl.dc_model.meas_dc_voltage()
        fbk.i_cs = abc2complex(mdl.grid_filter.meas_currents())
        fbk.u_cs = self.pwm.get_realized_voltage()
        fbk.u_gs = abc2complex(mdl.grid_filter.meas_pcc_voltage())

        return fbk

    def get_feedback_signals(self, mdl):
        """Get the feedback signals."""
        fbk = super().get_feedback_signals(mdl)
        fbk = self.get_electrical_measurements(fbk, mdl)

        return fbk

    def get_power_reference(self, fbk, ref):
        """
        Get the active power reference in DC bus voltage control mode.

        Parameters
        ----------
        fbk : SimpleNamespace
            Feedback signals.
        ref : SimpleNamespace
            Reference signals, containing the digital time `t`.

        Returns
        -------
        ref : SimpleNamespace
            Reference signals, containing the following fields:
                
                u_dc : float
                    DC-bus voltage reference (V).
                p_g : float
                    Active power reference (W).
                q_g : float
                    Reactive power reference (VAr).  

        """
        if self.on_u_dc:
            # DC-bus voltage control mode

            # Definition of capacitance energy variables for the DC-bus controller
            ref.u_dc = self.ref.u_dc(ref.t)
            ref_W_dc = 0.5*self.par.C_dc*ref.u_dc**2
            W_dc = 0.5*self.par.C_dc*fbk.u_dc**2
            # Define the active power reference
            ref.p_g = self.dc_bus_volt_ctrl.output(ref_W_dc, W_dc)

        else:
            # Power control mode
            ref.u_dc = None
            ref.p_g = self.ref.p_g(ref.t)

        # Define the reactive power reference
        ref.q_g = self.ref.q_g(ref.t) if callable(self.ref.q_g) else self.ref.q_g

        return ref

    def update(self, fbk, ref):
        """Extend the base class method."""
        super().update(fbk, ref)
        if self.dc_bus_volt_ctrl:
            self.dc_bus_volt_ctrl.update(ref.T_s, ref.p_g)
