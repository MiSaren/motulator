"""Common control functions and classes."""
from abc import ABC
from types import SimpleNamespace

import numpy as np

from motulator.common.control import ControlSystem, PIController
from motulator.common.utils import abc2complex, wrap


# %%
class PLL:
    """
    PLL synchronizing loop.

    Parameters
    ----------
    cfg: GFLControlCfg
        Model and controller configuration parameters.
        
    """

    def __init__(self, cfg):
        """
        Parameters
        ----------
        cfg : GFLControlCfg
           Control parameters.
    
        """
        self.cfg = cfg

        # PLL gains
        self.k_p_pll = 2*cfg.zeta_pll*cfg.w0_pll/cfg.grid_par.u_gN
        self.k_i_pll = cfg.w0_pll*cfg.w0_pll/cfg.grid_par.u_gN

        # Initial states
        self.w_pll = cfg.grid_par.w_gN
        self.theta_c = 0

    def output(self, fbk, ref):
        """
        Compute the estimated frequency and phase angle using the PLL.
    
        Parameters
        ----------
        u_gs : complex
            PCC voltage (in V) in synchronous coordinates.
    
        Returns
        -------
        ref : SimpleNamespace
            References, containing the following fields:

        ref.u_g : complex
            Grid-voltage vector
        ref.u_gq : float
            Error signal (in V, corresponds to the q-axis grid voltage).
        ref.abs_u_g : float
            magnitude of the grid voltage vector (in V).
        """

        # Definition of the grid-voltage vector
        ref.u_g = fbk.u_gs*np.exp(-1j*fbk.theta_c)
        # Definition of the error using the q-axis voltage
        ref.u_gq = np.imag(ref.u_g)
        # Absolute value of the grid-voltage vector
        ref.abs_ug = np.abs(ref.u_g)

        return ref

    def update(self, u_gq):
        """
        Update the integral state.
    
        Parameters
        ----------
        u_gq : real
            Error signal (in V, corresponds to the q-axis grid voltage).
    
        """
        cfg = self.cfg
        # Calculation of the estimated PLL frequency
        w_g_pll = self.k_p_pll*u_gq + self.w_pll

        # Update the integrator state
        self.w_pll = self.w_pll + cfg.T_s*self.k_i_pll*u_gq
        # Update the grid-voltage angle state
        self.theta_c = self.theta_c + cfg.T_s*w_g_pll
        self.theta_c = wrap(self.theta_c)  # Limit to [-pi, pi]


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
    zeta : float, optional
        Damping ratio of the closed-loop system. The default is 1.
    alpha_dc : float, optional
        Closed-loop bandwidth (rad/s). The default is 2*np.pi*30.
    p_max : float, optional
        Maximum converter power (W). The default is `inf`.
        
    References
    ----------
    .. [#Hur2001] Hur, Jung, Nam, "A Fast Dynamic DC-Link Power-Balancing
       Scheme for a PWM Converter–Inverter System," IEEE Trans. Ind. Electron.,
       2001, https://doi.org/10.1109/41.937412

    """

    def __init__(self, zeta=1, alpha_dc=2*np.pi*30, p_max=np.inf):
        k_p = -2*zeta*alpha_dc
        k_i = -(alpha_dc**2)
        k_t = k_p
        super().__init__(k_p, k_i, k_t, p_max)


# %%
class GridConverterControlSystem(ControlSystem, ABC):
    """
    Base class for control of grid-connected converters.
    
    This base class provides typical functionalities for control of
    grid-connected converters. This can be used both in power control and
    DC-bus voltage control modes. 

    Parameters
    ----------
    grid_par : GridPars
        Grid model parameters.
    dc_bus_par : DCBusPars
        DC-bus model parameters.
    T_s : float
        Sampling period (s).
    on_u_cap : bool, optional
        to use the filter capacitance voltage measurement or PCC voltage. 
        The default is False

    Attributes
    ----------
    ref : SimpleNamespace
        References, possibly containing the following fields:

            U : float | callable
                Converter output voltage reference (V). Can be given either as
                a constant or a function of time (s). 
            p_g : callable
                Active power reference (W) as a function of time (s). This
                signal is needed in power control mode.
            q_g : callable
                Reactive power reference (VAr) as a function of time (s). This
                signal is needed if grid-following control is used.
            u_dc : callable
                DC-voltage reference (V) as a function of time (s). This signal
                is needed in DC-bus voltage control mode.
            on_u_cap: bool, optional
                to use the filter capacitance voltage measurement or PCC voltage. 
                The default is False.

    dc_bus_volt_ctrl : DCBusVoltageController | None
        DC-bus voltage controller. The default is None.

    """

    def __init__(self, grid_par, dc_bus_par, T_s, on_u_cap=False):
        super().__init__(T_s)
        self.grid_par = grid_par
        self.dc_bus_par = dc_bus_par
        self.on_u_cap = on_u_cap
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
                    Converter current (A) in stationary coordinates.
                u_cs : complex
                    Realized converter output voltage (V) in stationary
                    coordinates. This signal is obtained from the PWM.
                u_gs : complex
                    PCC voltage (V) in stationary coordinates.

        """
        fbk.u_dc = mdl.converter.meas_dc_voltage()
        fbk.i_cs = abc2complex(mdl.grid_filter.meas_currents())
        fbk.u_cs = self.pwm.get_realized_voltage()
        if self.on_u_cap:
            fbk.u_gs = abc2complex(mdl.grid_filter.meas_cap_voltage())
        else:
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
        if self.dc_bus_volt_ctrl:
            # DC-bus voltage control mode

            # Definition of capacitance energy variables for the DC-bus controller
            ref.u_dc = self.ref.u_dc(ref.t)
            ref_W_dc = 0.5*self.dc_bus_par.C_dc*ref.u_dc**2
            W_dc = 0.5*self.dc_bus_par.C_dc*fbk.u_dc**2
            # Define the active power reference
            ref.p_g = self.dc_bus_volt_ctrl.output(ref_W_dc, W_dc)

        else:
            # Power control mode
            ref.u_dc = None
            ref.p_g = self.ref.p_g(ref.t)

        # Define the reactive power reference
        ref.q_g = self.ref.q_g(ref.t) if callable(
            self.ref.q_g) else self.ref.q_g

        return ref

    def update(self, fbk, ref):
        """Extend the base class method."""
        super().update(fbk, ref)
        if self.dc_bus_volt_ctrl:
            self.dc_bus_volt_ctrl.update(ref.T_s, ref.p_g)
