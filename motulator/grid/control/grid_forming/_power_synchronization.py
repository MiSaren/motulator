"""Power synchronization control for grid-connected converters."""

# %%
from dataclasses import dataclass

import numpy as np

from motulator.common.utils import wrap, DCBusPars, FilterPars
from motulator.grid.control import GridConverterControlSystem
from motulator.grid.utils import GridPars


# %%
@dataclass
class PSCControlCfg:
    """
    Power synchronization control configuration.

    Parameters
    ----------
    grid_par : GridPars
        Grid model parameters.
    dc_bus_par : DCBusPars
        DC-bus model parameters.
    filter_par : FilterPars
        Filter model parameters.
    T_s : float, optional
        Sampling period of the controller (s). Default is 1/(16e3).
    on_rf : bool, optional
        Enable reference-feedforward for the control. Default is False.
    on_u_dc : bool, optional
        Enable DC-bus voltage control mode. Default is False.
    on_u_g : bool, optional
        Enable control of PCC voltage. Default is False (converter output
        voltage is controlled).
    i_max : float, optional
        Maximum current modulus (A). Default is 20.
    R_a : float, optional
        Damping resistance (Ω). Default is 4.6.
    w_0_cc : float, optional
        Current controller undamped natural frequency (rad/s).
        Default is 2*pi*5.
    K_cc : float, optional
        Current controller low-pass filter gain. Default is 1.
    overmodulation : str, optional
        Overmodulation method for the PWM. Default is Minimum Phase Error "MPE".
    """

    grid_par: GridPars
    dc_bus_par: DCBusPars
    filter_par: FilterPars
    T_s: float = 1/(16e3)
    on_rf: bool = False
    on_u_dc: bool = False
    on_u_g: bool = False
    i_max: float = 20
    R_a: float = 4.6
    w_0_cc: float = 2*np.pi*5
    K_cc: float = 1
    overmodulation: str = "MPE"

    def __post_init__(self):
        par = self.grid_par
        self.k_p_psc = par.w_gN*self.R_a/(1.5*par.u_gN*par.u_gN)


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

    Attributes
    ----------
    current_ctrl : PSCCurrentController
        Current controller object.
    

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
        super().__init__(
            cfg.grid_par,
            cfg.dc_bus_par,
            cfg.T_s,
        )
        self.cfg = cfg
        self.current_ctrl = PSCCurrentController(cfg)
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
        fbk.p_g = 1.5*np.real(fbk.u_c*np.conj(fbk.i_c))
        fbk.q_g = 1.5*np.imag(fbk.u_c*np.conj(fbk.i_c))

        return fbk

    def output(self, fbk):
        """Extend the base class method."""
        par, cfg = self.grid_par, self.cfg

        # Get the reference signals
        ref = super().output(fbk)
        ref = super().get_power_reference(fbk, ref)
        # Voltage magnitude reference
        ref.U = self.ref.U(ref.t) if callable(self.ref.U) else self.ref.U

        # Calculation of power droop
        fbk.w_c = par.w_gN + (cfg.k_p_psc)*(ref.p_g - fbk.p_g)

        # Get voltage reference from current controller
        ref = self.current_ctrl.output(fbk, ref, par)

        # Transform voltage reference into stator coordinates
        ref.u_cs = np.exp(1j*fbk.theta_c)*ref.u_c

        # Get duty ratios from PWM
        ref.d_abc = self.pwm(
            ref.T_s,
            ref.u_cs,
            fbk.u_dc,
            par.w_gN,
            cfg.overmodulation,
        )

        return ref

    def update(self, fbk, ref):
        """Extend the base class method."""
        super().update(fbk, ref)
        # Estimated phase angle
        self.theta_c = fbk.theta_c + ref.T_s*fbk.w_c
        # Limit to [-pi, pi]
        self.theta_c = wrap(self.theta_c)
        self.current_ctrl.update(fbk, ref)


class PSCCurrentController:
    """
    PSC-based current controller.
    
    PSC makes the converter operate as a voltage source, however, this block
    is used to damp the current oscillations and limit the current
    flowing through the converter to avoid physical damages of the device.
    
    It is important to note that this block uses P-type controller and can thus
    encounter steady-state error when the current reference is saturated.

    Parameters
    ----------
    cfg : PSCControlCfg
        Model and controller configuration parameters.
      
    """

    def __init__(self, cfg):
        self.cfg = cfg

        #initial states
        self.i_c_filt = 0j

    def output(self, fbk, ref, par):
        """Compute the converter voltage reference signal."""
        cfg = self.cfg

        # Low pass filter for the current:
        i_c_filt = self.i_c_filt

        # Use of reference feedforward for d-axis current
        if cfg.on_rf:
            i_c_ref = ref.p_g/(ref.U*1.5) + 1j*np.imag(i_c_filt)
        else:
            i_c_ref = i_c_filt

        # Calculation of the modulus of current reference
        i_abs = np.abs(i_c_ref)
        i_cd_ref = np.real(i_c_ref)
        i_cq_ref = np.imag(i_c_ref)

        # Current limitation algorithm
        if i_abs > 0:
            i_ratio = cfg.i_max/i_abs
            i_cd_ref = np.sign(i_cd_ref)*np.min(
                [i_ratio*np.abs(i_cd_ref),
                 np.abs(i_cd_ref)])
            i_cq_ref = np.sign(i_cq_ref)*np.min(
                [i_ratio*np.abs(i_cq_ref),
                 np.abs(i_cq_ref)])
            i_c_ref = i_cd_ref + 1j*i_cq_ref

        ref.i_c = i_c_ref

        # Calculation of converter voltage output (reference sent to PWM)
        ref.u_c = ((ref.U + 0j) + cfg.R_a*(ref.i_c - fbk.i_c) +
                   cfg.on_u_g*1j*self.cfg.filter_par.L_fc*par.w_gN*fbk.i_c)

        return ref

    def update(self, fbk, ref):
        """Update the integral state for the current low pass filter."""
        cfg = self.cfg

        self.i_c_filt = (1 - ref.T_s*cfg.w_0_cc)*self.i_c_filt + (
            cfg.K_cc*ref.T_s*cfg.w_0_cc*fbk.i_c)
