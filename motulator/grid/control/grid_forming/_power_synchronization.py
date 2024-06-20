"""Power synchronization control for grid-connected converters"""

# %%
from dataclasses import dataclass, field, InitVar
from types import SimpleNamespace

import numpy as np

from motulator.grid.control import GridConverterControlSystem
from motulator.drive.utils import InductionMachineInvGammaPars
from motulator.common.control import PWM, RateLimiter
from motulator.common.utils import wrap

# %%
@dataclass
class PSCControlCfg:
    """Power synchronization control configuration"""

    par: GridModelPars # TODO: implement
    T_s: float = 1/(16e3)

    def __post_init__(self, R_a, k_scal):
        par = self.par
        k_p_psc = par.w_g*R_a/(k_scal*par.u_gN*par.u_gN)
        self.gain = SimpleNamespace(
            R_a=R_a, k_scal=k_scal, k_p_psc=k_p_psc)


# %%
class PSCControl(GridConverterControlSystem):
    """
    Power synchronization control for grid converters.
    
    This implements the power synchronization control (PSC) method described in
    [#Har2019]_. The alternative reference-feedforward PSC (RFPSC) can also be 
    used and is based on [#Har2020]_.

    Parameters
    ----------
    cfg : PSCControlCfg
        Model and controller configuration parameters.

    References
    ----------
    .. [#Har2019] Harnefors, Hinkkanen, Riaz, Rahman, Zhang, "Robust Analytic
        Design of Power-Synchronization Control," IEEE Trans. Ind. Electron.,
        Aug. 2019, https://doi.org/10.1109/TIE.2018.2874584
        
    .. [#Har2020] Harnefors, Rahman, Hinkkanen, Routimo, "Reference-Feedforward
        Power-Synchronization Control," IEEE Trans. Power Electron., Sep. 2020,
        https://doi.org/10.1109/TPEL.2020.2970991

    """

    def __init__(self, cfg):
        super().__init__(cfg.par, cfg.T_s, on_u_dc=cfg.on_u_dc)
        self.gain = cfg.gain
        self.pwm = PWM()
        # Initialize the states
        self.theta_c = 0

    def get_feedback_signals(self, mdl):
        """Get the feedback signals."""
        fbk = super().get_feedback_signals(mdl)
        fbk.theta_c = self.theta_c
        # Transform the measured values into dq frame
        fbk.u_g = np.exp(-1j*fbk.theta_c)*fbk.u_gs
        fbk.i_c = np.exp(-1j*fbk.theta_c)*fbk.i_cs
        fbk.u_c = np.exp(-1j*fbk.theta_c)*fbk.u_cs

        return fbk

    def output(self, fbk):
        """Extend the base class method."""
        par, gain = self.par, self.gain

        # Define the active power/frequency synchronization
        def power_synch(fbk, ref):
            # Calculation of power droop
            fbk.w_c = par.w_g + (gain.k_p_psc)*(ref.p_g - fbk.p_g)
            # Estimated phase angle
            theta_c = fbk.theta_c + par.T_s*fbk.w_c
            # Limit to [-pi, pi]
            fbk.theta_c = wrap(theta_c)

            return fbk

        # Get the reference signals
        ref = super().output(fbk)
        if self.on_u_dc:
            ref.u_dc = self.ref.u_dc(ref.t)
        ref = super().get_power_reference(fbk, ref)

        # Calculation of active and reactive powers
        fbk.p_g = gain.k_scal*np.real(fbk.u_c*np.conj(fbk.i_c))
        fbk.q_g = gain.k_scal*np.imag(fbk.u_c*np.conj(fbk.i_c))

        # Calculation of converter voltage angle
        fbk = power_synch(fbk, ref)

        return ref

    def update(self, fbk, ref):
        """Extend the base class method."""
        super().update(fbk, ref)


class CurrentController:
    """
    PSC-based current controller.
    
    PSC makes the converter operate as a voltage source, however, this block
    is used to damp the current oscillations and limit the current
    flowing through the converter to avoid physical damages of the device.
    
    It is important to note that this block uses P-type controller and can thus
    encounter steady-state error when the current reference is saturated.
        
    """

    def __init__(self, cfg, w_0_cc, K_cc):
        self.T_s = cfg.T_s
        self.L_f = cfg.L_f
        self.R_a = cfg.gain.R_a
        self.k_scal = cfg.gain.k_scal
        self.w_0_cc = w_0_cc
        self.K_cc = K_cc
        # activation/deactivation of reference feedforward action
        self.on_rf = cfg.on_rf
        # activation/deactivation of PCC voltage control option
        self.on_u_g = cfg.on_u_g
        # Calculated maximum current in A
        self.i_max = cfg.i_max
        #initial states
        self.i_c_filt = 0j 

    def output(self, ):

    def update(self, ):

