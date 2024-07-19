"""grid following control methods for grid onverters."""

# %%
from __future__ import annotations
from dataclasses import dataclass

import numpy as np

from motulator.grid.control import (
    GridConverterControlSystem)
from motulator.grid.utils import GridModelPars

from motulator.common.control import (ComplexFFPIController)
from motulator.grid.control._common import PLL
from motulator.common.utils import complex2abc

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
    overmodulation : str, optional
        overmodulation method. The default is Minimum Phase Error "MPE".
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
    overmodulation: str = "MPE"

    def __post_init__(self):
        par = self.par

        # Current controller gains
        self.k_p_i = self.alpha_c*par.L_f
        self.k_i_i = np.power(self.alpha_c,2)*par.L_f
        self.r_i = self.alpha_c*self.par.L_f

        # PLL gains
        self.k_p_pll = 2*self.zeta*self.w0_pll/par.U_gN
        self.k_i_pll = self.w0_pll*self.w0_pll/par.U_gN


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
        super().__init__(cfg.par, cfg.T_s, on_u_dc=cfg.on_u_dc, on_u_cap=cfg.on_u_cap)
        self.cfg = cfg
        self.current_ctrl = CurrentController(cfg)
        self.pll = PLL(cfg)
        self.current_reference = CurrentRefCalc(cfg)

        # Initialize the states
        self.u_filt = cfg.par.U_gN + 1j*0
        

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

        #fbk.u_g_abc = complex2abc(fbk.u_gs)
    
        return fbk
    
    def output(self, fbk):
        """Extend the base class method."""
        par = self.par
        # Get the reference signals
        ref = super().output(fbk)
        if self.on_u_dc:
            ref.u_dc = self.ref.u_dc(ref.t)
        ref = super().get_power_reference(fbk, ref)
        self.current_reference.get_current_reference(ref)

        # Calculation of the modulus of current reference
        i_abs = np.abs(ref.i_c)
        i_cd_ref = np.real(ref.i_c)
        i_cq_ref = np.imag(ref.i_c)

        # And current limitation algorithm
        if i_abs > 0:
            i_ratio = self.cfg.i_max/i_abs
            i_cd_ref = np.sign(i_cd_ref)*np.min(
                [i_ratio*np.abs(i_cd_ref),np.abs(i_cd_ref)])
            i_cq_ref = np.sign(i_cq_ref)*np.min(
                [i_ratio*np.abs(i_cq_ref),np.abs(i_cq_ref)])
            ref.i_c = i_cd_ref + 1j*i_cq_ref
        
        # Low pass filter for the feedforward PCC voltage:
        u_filt = self.u_filt

        # Use of PLL to bring ugq to zero
        self.pll.output(fbk, ref)

        # Voltage reference generation in synchronous coordinates
        ref.u_c = self.current_ctrl.output(ref.i_c, fbk.i_c, u_filt, par.w_gN)

         # Transform the voltage reference into stator coordinates
        ref.u_cs = np.exp(1j*fbk.theta_c)*ref.u_c
        
        # get the duty ratios from the PWM
        ref.d_abc = self.pwm(ref.T_s, ref.u_cs, fbk.u_dc, par.w_gN, self.cfg.overmodulation)

        return ref
    
    def update(self, fbk, ref):
        """Extend the base class method."""
        super().update(fbk, ref)
        self.current_ctrl.update(ref.T_s, fbk.u_c)
        self.pll.update(ref.u_gq)
        self.u_filt = (1 - ref.T_s*self.cfg.alpha_ff)*self.u_filt + (
            ref.T_s*self.cfg.alpha_ff*fbk.u_g)     


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
        cfg : GFLControlCfg
            Model and controller configuration parameters.
    
        """
        self.u_gN = cfg.par.U_gN
        self.i_max = cfg.i_max


    def get_current_reference(self, ref):

        """
        Current reference genetator.
    
        Parameters
        ----------
        p_g_ref : float
            active power reference
        q_g_ref : float
            reactive power reference
        """

        # Calculation of the current references in the stationary frame:
        ref.i_c = 2*ref.p_g/(3*self.u_gN) -2*1j*ref.q_g/(3*self.u_gN)
