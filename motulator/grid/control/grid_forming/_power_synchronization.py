"""Power synchronization control for grid-connected converters"""

# %%
from dataclasses import dataclass, field, InitVar
from types import SimpleNamespace

import numpy as np

from motulator.grid.control import GridConverterControlSystem
from motulator.grid.utils import GridModelPars
from motulator.common.control import PWM
from motulator.common.utils import wrap

# %%
@dataclass
class PSCControlCfg:
    """Power synchronization control configuration"""

    par: GridModelPars
    T_s: float = 1/(16e3)
    on_rf: bool = False
    on_u_dc: bool = False
    on_u_g: bool = False
    i_max: float = 20
    gain: SimpleNamespace = field(init=False)
    R_a: InitVar[float] = 4.6
    k_scal: InitVar[float] = 3/2
    w_0_cc: InitVar[float] = 2*np.pi*5
    K_cc: InitVar[float] = 1

    def __post_init__(self, R_a, k_scal, w_0_cc, K_cc):
        par = self.par
        k_p_psc = par.w_g*R_a/(k_scal*par.U_gN*par.U_gN)
        self.gain = SimpleNamespace(
            R_a=R_a, k_scal=k_scal, k_p_psc=k_p_psc, w_0_cc=w_0_cc, K_cc=K_cc)


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
        self.current_ctrl = CurrentController(cfg)
        self.ref.q_g = 0
        # Initialize the states
        self.theta_c = 0

    def get_feedback_signals(self, mdl):
        """Get the feedback signals."""
        fbk = super().get_feedback_signals(mdl)
        fbk.theta_c = self.theta_c
        # Transform the measured values into synchronous coordinates
        fbk.u_g = np.exp(-1j*fbk.theta_c)*fbk.u_gs
        fbk.i_c = np.exp(-1j*fbk.theta_c)*fbk.i_cs
        fbk.u_c = np.exp(-1j*fbk.theta_c)*fbk.u_cs

        # Calculation of active and reactive powers
        fbk.p_g = self.gain.k_scal*np.real(fbk.u_c*np.conj(fbk.i_c))
        fbk.q_g = self.gain.k_scal*np.imag(fbk.u_c*np.conj(fbk.i_c))

        return fbk

    def output(self, fbk):
        """Extend the base class method."""
        par, gain = self.par, self.gain

        # Define the active power/frequency synchronization
        def power_synch(fbk, ref):
            # Calculation of power droop
            fbk.w_c = par.w_g + (gain.k_p_psc)*(ref.p_g - fbk.p_g)
            # Estimated phase angle
            self.theta_c = fbk.theta_c + ref.T_s*fbk.w_c
            # Limit to [-pi, pi]
            self.theta_c = wrap(self.theta_c)

            return fbk

        # Get the reference signals
        ref = super().output(fbk)
        if self.on_u_dc:
            ref.u_dc = self.ref.u_dc(ref.t)
        ref = super().get_power_reference(fbk, ref)
        # Voltage magnitude reference
        ref.U = self.ref.U(ref.t)

        # Calculation of converter voltage angle
        fbk = power_synch(fbk, ref)

        # Get voltage reference from current controller
        ref = self.current_ctrl.output(fbk, ref, par)

        # Transform voltage reference into stator coordinates
        ref.u_cs = np.exp(1j*fbk.theta_c)*ref.u_c

        # Get duty ratios from PWM
        ref.d_abc = self.pwm(ref.T_s, ref.u_cs, fbk.u_dc, par.w_g)

        return ref

    def update(self, fbk, ref):
        """Extend the base class method."""
        super().update(fbk, ref)
        self.current_ctrl.update(fbk, ref)


class CurrentController:
    """
    PSC-based current controller.
    
    PSC makes the converter operate as a voltage source, however, this block
    is used to damp the current oscillations and limit the current
    flowing through the converter to avoid physical damages of the device.
    
    It is important to note that this block uses P-type controller and can thus
    encounter steady-state error when the current reference is saturated.
        
    """

    def __init__(self, cfg):
        self.gain = cfg.gain
        # activation/deactivation of reference feedforward action
        self.on_rf = cfg.on_rf
        # activation/deactivation of PCC voltage control option
        self.on_u_g = cfg.on_u_g
        # Calculated maximum current in A
        self.i_max = cfg.i_max
        #initial states
        self.i_c_filt = 0j

    def output(self, fbk, ref, par):
        """Compute the converter voltage reference signal."""
        # Low pass filter for the current:
        i_c_filt = self.i_c_filt

        # Use of reference feedforward for d-axis current
        if self.on_rf:
            i_c_ref = ref.p_g/(ref.U*self.gain.k_scal) + 1j*np.imag(i_c_filt)
        else:
            i_c_ref = i_c_filt

        # Calculation of the modulus of current reference
        i_abs = np.abs(i_c_ref)
        i_cd_ref = np.real(i_c_ref)
        i_cq_ref = np.imag(i_c_ref)

        # Current limitation algorithm
        if i_abs > 0:
            i_ratio = self.i_max/i_abs
            i_cd_ref = np.sign(i_cd_ref)*np.min(
                [i_ratio*np.abs(i_cd_ref),np.abs(i_cd_ref)])
            i_cq_ref = np.sign(i_cq_ref)*np.min(
                [i_ratio*np.abs(i_cq_ref),np.abs(i_cq_ref)])
            i_c_ref = i_cd_ref + 1j*i_cq_ref

        ref.i_c = i_c_ref

        # Calculation of converter voltage output (reference sent to PWM)
        ref.u_c = ((ref.U + 0j) + self.gain.R_a*(ref.i_c - fbk.i_c) +
           self.on_u_g*1j*par.L_f*par.w_g*fbk.i_c)

        return ref

    def update(self, fbk, ref):
        """Update the integral state for the current low pass filter."""
        self.i_c_filt = (1 - ref.T_s*self.gain.w_0_cc)*self.i_c_filt + (
            self.gain.K_cc*ref.T_s*self.gain.w_0_cc*fbk.i_c)
