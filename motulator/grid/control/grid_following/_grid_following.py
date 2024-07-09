"""grid following control methods for grid onverters."""

# %%
from __future__ import annotations
from collections.abc import Callable
from dataclasses import dataclass, field

import numpy as np

from motulator.grid.control import (
    GridConverterControlSystem, DCBusVoltageController)
from motulator.grid.utils import GridModelPars

from motulator.common.control import (ComplexFFPIController)
from motulator.common.utils import wrap

# %%
@dataclass
class GFLControlCfg:
    """Grid-following control configuration
    
    Parameters
    ----------
    par : GridModelPars
        Grid model parameters.
    T_s : float, optional
        Sampling period (s). The default is 1/(16e3).
    on_u_dc : bool, optional
        to activate dc voltage controller. The default is False.
    on_u_cap : bool, optional
        to use the filter capacitance voltage measurement or PCC voltage. The default is False.
    i_max : float, optional
        maximum current modulus in A. The default is 20.
    alpha_c : float, optional
        current controller bandwidth. The default is 2*np.pi*400.
    alpha_ff : float, optional
        low pass filter bandwidth for voltage feedforward term. The default is 2*np.pi*(4*50).
    k_scal : float, optional
        scaling ratio of the abc/dq transformation. The default is 3/2.
        
    Parameters for the Phase Locked Loop (PLL)
    w0_pll : float, optional
        undamped natural frequency of the PLL. The default is 2*np.pi*20.
    zeta : float, optional
        damping ratio of the PLL. The default is 1.

    parameters for the DC-voltage controller
    p_max : float, optional
        maximum power reference in W. The default is 10e3.
    zeta_dc : float, optional
        damping ratio of the DC-voltage controller. The default is 1.
    w_0_dc : float, optional
        controller undamped natural frequency in rad/s. The default is 2*np.pi*30.
    """

    par: GridModelPars
    T_s: float = 1/(16e3)
    on_u_dc: bool = False
    on_u_cap: bool = False
    i_max: float = 20
    alpha_c: float = 2*np.pi*400
    alpha_ff: float = 2*np.pi*(4*50)
    k_scal: float = 3/2

    w0_pll: float = 2*np.pi*20
    zeta: float = 1

    p_max: float = 10e3
    zeta_dc: float = 1
    w_0_dc: float = 2*np.pi*30

    def __post_init__(self):
        self.k_p_i = self.alpha_c*self.par.L_f
        self.k_i_i = np.power(self.alpha_c,2)*self.par.L_f
        self.r_i = self.alpha_c*self.par.L_f
        self.k_p_pll = 2*self.zeta*self.w0_pll/self.par.U_gN
        self.k_i_pll = self.w0_pll*self.w0_pll/self.par.U_gN


# %%
class GFLControl(GridConverterControlSystem):
    """
    Grid-following control for power converters.
    
    Parameters
    ----------
    cfg : GFLControlCfg
        Control configuration.
    
    """

    def __init__(self, cfg):
        super().__init__(cfg.par, cfg.T_s, on_u_dc=cfg.on_u_dc)
        self.cfg = cfg
        self.current_ctrl = CurrentController(cfg)
        # Initialize the states

        self.current_reference = CurrentRefCalc(cfg)
        self.u_filt = cfg.par.U_gN + 1j*0
        self.pll = PLL(cfg)

    def get_feedback_signals(self, mdl):
        fbk = super().get_feedback_signals(mdl)
        fbk.theta_c = self.pll.theta_c
        # Transform the measured current in dq frame
        fbk.u_g = np.exp(-1j*fbk.theta_c)*fbk.u_gs
        fbk.i_c = np.exp(-1j*fbk.theta_c)*fbk.i_cs
        fbk.u_c = np.exp(-1j*fbk.theta_c)*fbk.u_cs

        # Calculating of active and reactive powers
        fbk.p_g = self.cfg.k_scal*np.real(fbk.u_c*np.conj(fbk.i_c))
        fbk.q_g = self.cfg.k_scal*np.imag(fbk.u_c*np.conj(fbk.i_c))

        if self.cfg.on_u_cap:
            fbk.u_g_abc = mdl.grid_filter.meas_cap_voltage()
        else:
            fbk.u_g_abc = mdl.grid_filter.meas_pcc_voltage()

        return fbk
    
    def output(self, fbk):
        """Extend the base class method."""
        par, cfg = self.par, self.cfg
        # Get the reference signals
        ref = super().output(fbk)
        if self.on_u_dc:
            ref.u_dc = self.ref.u_dc(ref.t)
        ref = super().get_power_reference(fbk, ref)
        ref.i_c = self.current_reference.output(ref)
        # Voltage reference generation in synchronous coordinates
        ref.u_c = self.current_ctrl.output(ref.i_c, fbk.i_c, fbk.u_g, par.w_g)

        # Transform the voltage reference into stator coordinates
        ref.u_cs = np.exp(1j*fbk.theta_c)*ref.u_c

        # Use of PLL to bring ugq to zero
        ref.u_g_q, ref.abs_u_g, ref.w_pll, ref.theta_pll = self.pll.output(fbk.u_g_abc)
        # get the duty ratios from the PWM
        ref.d_abc = self.pwm(ref.T_s, ref.u_cs, fbk.u_dc, par.w_g)

        return ref
    
    def update(self, fbk, ref):
        """Extend the base class method."""
        super().update(fbk, ref)
        self.current_ctrl.update(ref.T_s, fbk.u_c)
        self.pll.update(ref.u_g_q)
        if self.on_u_dc:
            self.dc_bus_volt_ctrl.update(ref.T_s, ref.p_g) 


# %%
class PLL:

    """
    PLL synchronizing loop.

    Parameters
    ----------
    u_g_abc : ndarray, shape (3,)
        Phase voltages at the PCC.

    Returns
    -------
    u_g_q : float
        q-axis of the PCC voltage (V)
    abs_u_g : float
        amplitude of the voltage waveform, in V
    theta_pll : float
        estimated phase angle (in rad).
        
    """

    def __init__(self, cfg):

        """
        Parameters
        ----------
        pars : GridFollowingCtrlPars
           Control parameters.
    
        """
        self.T_s = cfg.T_s
        self.w_0_pll = cfg.w0_pll
        self.k_p_pll = cfg.k_p_pll
        self.k_i_pll = cfg.k_i_pll

        # Initial states
        self.w_pll = cfg.par.w_g
        self.theta_c = 0


    def output(self, u_g_abc):

        """
        Compute the estimated frequency and phase angle using the PLL.
    
        Parameters
        ----------
        u_g_abc : ndarray, shape (3,)
            Grid 3-phase voltage.
    
        Returns
        -------
        u_g_q : float
            Error signal (in V, corresponds to the q-axis grid voltage).
        abs_u_g : float
            magnitude of the grid voltage vector (in V).
        w_g_pll : float
            estimated grid frequency (in rad/s).
        theta_pll : float
            estimated phase angle (in rad).
        """

        u_g_ab = u_g_abc[0] - u_g_abc[1] # calculation of phase-to-phase voltages
        u_g_bc = u_g_abc[1] - u_g_abc[2] # calculation of phase-to-phase voltages

        # Calculation of u_g in complex form (stationary coordinates)
        u_g_s = (2/3)*u_g_ab +(1/3)*u_g_bc + 1j*(np.sqrt(3)/(3))*u_g_bc
        # And then in general coordinates
        u_g = u_g_s*np.exp(-1j*self.theta_c)
        # Definition of the error using the q-axis voltage
        u_g_q = np.imag(u_g)

        # Absolute value of the grid-voltage vector
        abs_u_g = np.abs(u_g)

        # Calculation of the estimated PLL frequency
        w_g_pll = self.k_p_pll*u_g_q + self.w_pll

        # Estimated phase angle
        theta_pll = self.theta_c + self.T_s*w_g_pll

        return u_g_q, abs_u_g, w_g_pll, theta_pll


    def update(self, u_g_q):
        """
        Update the integral state.
    
        Parameters
        ----------
        u_g_q : real
            Error signal (in V, corresponds to the q-axis grid voltage).
    
        """

        # Calculation of the estimated PLL frequency
        w_g_pll = self.k_p_pll*u_g_q + self.w_pll

        # Update the integrator state
        self.w_pll = self.w_pll + self.T_s*self.k_i_pll*u_g_q
        # Update the grid-voltage angle state
        self.theta_c = self.theta_c + self.T_s*w_g_pll
        self.theta_c = wrap(self.theta_c)    # Limit to [-pi, pi]


# %%
class CurrentController(ComplexFFPIController):
    """
    2DOF PI current controller for grid converters.

    This class provides an interface for a current controller for grid 
    converters. The gains are initialized based on the desired closed-loop 
    bandwidth and the filter inductance. 

    Parameters
    ----------
    cfg : ModelPars
        Grid converter parameters, contains the filter inductance `L_f` (H)
        and the Closed-loop bandwidth 'alpha_c' (rad/s).

    """

    def __init__(self, cfg):
        k_t = cfg.alpha_c*cfg.par.L_f
        k_i = cfg.alpha_c*k_t
        k_p = 2*k_t
        L_f = cfg.par.L_f
        super().__init__(k_p, k_i, k_t, L_f)

# %%
class CurrentRefCalc:

    """
    Current controller reference generator
    
    This class is used to generate the current references for the current
    controllers based on the active and reactive power references.
    
    """

    def __init__(self, cfg):

        """
        Parameters
        ----------
        pars : GridFollowingCtrlPars
            Control parameters.
    
        """
        self.u_gN = cfg.par.U_gN
        self.u_filt = self.u_gN + 1j*0
        self.i_max = cfg.i_max


    def output(self, ref):

        """
        Current reference genetator.
    
        Parameters
        ----------
        p_g_ref : float
            active power reference
        q_g_ref : float
            reactive power reference
    
        Returns
        -------
        i_c_ref : float
            current reference in the rotationary frame
            
        """

        # Calculation of the current references in the stationary frame:
        i_c_ref = 2*ref.p_g/(3*self.u_gN) -2*1j*ref.q_g/(3*self.u_gN)

        # Calculation of the modulus of current reference
        i_abs = np.abs(i_c_ref)
        i_cd_ref = np.real(i_c_ref)
        i_cq_ref = np.imag(i_c_ref)
    
        # And current limitation algorithm
        if i_abs > 0:
            i_ratio = self.i_max/i_abs
            i_cd_ref = np.sign(i_cd_ref)*np.min(
                [i_ratio*np.abs(i_cd_ref),np.abs(i_cd_ref)])
            i_cq_ref = np.sign(i_cq_ref)*np.min(
                [i_ratio*np.abs(i_cq_ref),np.abs(i_cq_ref)])
            i_c_ref = i_cd_ref + 1j*i_cq_ref

        ref.i_c = i_c_ref

        return ref.i_c
