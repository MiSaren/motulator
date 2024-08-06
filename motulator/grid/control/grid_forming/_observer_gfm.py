"""Disturbance observer-based grid-forming control for grid converters."""

# %%
from dataclasses import dataclass

import numpy as np

from motulator.common.utils import wrap, DCBusPars, FilterPars
from motulator.grid.control import GridConverterControlSystem
from motulator.grid.utils import GridPars


# %%
@dataclass
class ObserverBasedGFMControlCfg:
    """
    Observer GFM control configuration.

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
    i_max : float, optional
        Maximum current modulus (A). Default is 20.
    R_a : float, optional
        Active resistance (Ω). Default is 4.6.
    k_v : float, optional
        Voltage gain. Default is 1.
    alpha_c : float, optional
        Current control bandwidth (rad/s). Default is 2*pi*400.
    alpha_o : float, optional
        Observer gain (rad/s). Default is 2*pi*50.
    overmodulation : str, optional
        Overmodulation method for the PWM. Default is Minimum Phase Error "MPE".
    
    """

    grid_par: GridPars
    dc_bus_par: DCBusPars
    filter_par: FilterPars
    T_s: float = 1/(16e3)
    i_max: float = 20
    R_a: float = 4.6
    k_v: float = 1
    alpha_c: float = 2*np.pi*400
    alpha_o: float = 2*np.pi*50
    overmodulation: str = "MPE"

    def __post_init__(self):
        par = self.filter_par
        self.L = par.L_fc + par.L_fg + self.grid_par.L_g  # Total inductance
        self.k_c = self.alpha_c*self.L  # Current control gain
        self.k_scal = 3/2  # Space vector scaling constant, peak-value scaling


# %%
class ObserverBasedGFMControl(GridConverterControlSystem):
    """
    Disturbance observer-based grid-forming control for grid converters.
    
    This implements the disturbance observer-based control method described in
    [#Nur2024]_. More specifically, the grid-forming mode using RFPSC-type
    gains and the current control mode are implemented.

    Parameters
    ----------
    cfg : ObserverBasedGFMControlCfg
        Model and controller configuration parameters.

    Attributes
    ----------
    observer : DisturbanceObserver
        Disturbance observer object.
    
    References
    ----------
    .. [#Nur2024] Nurminen, Mourouvin, Hinkkanen, Kukkola, "Multifunctional
        Grid-Forming Converter Control Based on a Disturbance Observer, "IEEE
        Trans. Power Electron., Jul. 2024,
        https://doi.org/10.1109/TPEL.2024.3433503

    """

    def __init__(self, cfg):
        super().__init__(cfg.grid_par, cfg.dc_bus_par, cfg.T_s, on_u_dc=False)
        self.cfg = cfg
        self.observer = DisturbanceObserver(cfg)
        self.ref.q_g = 0

    def get_feedback_signals(self, mdl):
        """Get the feedback signals."""
        fbk = super().get_feedback_signals(mdl)
        # Transform the measured values into synchronous coordinates
        fbk.i_c = np.exp(-1j*fbk.theta_c)*fbk.i_cs

        return fbk

    def output(self, fbk):
        """Extend the base class method."""
        par, cfg = self.grid_par, self.cfg

        # Get the reference signals
        ref = super().output(fbk)
        if self.dc_bus_volt_ctrl:
            ref.u_dc = self.ref.u_dc(ref.t)
        ref = super().get_power_reference(fbk, ref)
        # Converter voltage magnitude reference
        ref.v_c = self.ref.v_c(ref.t) if callable(
            self.ref.v_c) else self.ref.v_c

        # Get estimates from the observer
        fbk = self.observer.output(fbk)

        # Calculation of complex gains (grid-forming)
        abs_k_p = cfg.R_a/(cfg.k_scal*ref.v_c)
        abs_v_c = np.abs(fbk.v_c)
        k_p = abs_k_p*fbk.v_c/abs_v_c if abs_v_c > 0 else 0
        k_v = (1 - cfg.k_v*1j)*fbk.v_c/abs_v_c if abs_v_c > 0 else 0

        # Calculation of feedback correction term (grid-forming)
        fbk.e_c = k_p*(ref.p_g - fbk.p_g) + k_v*(ref.v_c - abs_v_c)

        # Current limitation
        ref.i_c = fbk.i_c + fbk.e_c/cfg.k_c
        if np.abs(ref.i_c) > cfg.i_max:
            ref.i_c_lim = ref.i_c/np.abs(ref.i_c)*cfg.i_max
            fbk.e_c = cfg.k_c*(ref.i_c_lim - fbk.i_c)
            #self.k_o = 2*np.pi*200 + 1j*self.w_g*0

        # Calculation of voltage reference
        ref.u_c = fbk.e_c + fbk.v_c

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
        self.observer.update(fbk, ref)


class DisturbanceObserver:
    """
    Disturbance observer.
    
    This implements a disturbance observer for grid converters.

    Parameters
    ----------
    cfg : ObserverBasedGFMControlCfg
        Model and controller configuration parameters.
      
    """

    def __init__(self, cfg):
        self.cfg = cfg
        self.w_c = cfg.grid_par.w_gN
        self.k_o = cfg.alpha_o - 1j*cfg.grid_par.w_gN
        # Initial states
        self.v_cp = cfg.grid_par.U_gN
        self.theta_c = 0

    def output(self, fbk):
        """Compute the signal estimates."""
        # Quasi-static converter voltage
        fbk.v_c = self.v_cp - self.k_o*self.cfg.L*fbk.i_c
        # Grid voltage
        fbk.u_g = fbk.v_c - 1j*self.cfg.grid_par.w_gN*self.cfg.L*fbk.i_c
        # Active and reactive power
        fbk.p_g = self.cfg.k_scal*np.real(fbk.v_c*np.conj(fbk.i_c))
        fbk.q_g = self.cfg.k_scal*np.imag(fbk.u_g*np.conj(fbk.i_c))

        return fbk

    def update(self, fbk, ref):
        """Update the observer integral states."""
        self.v_cp += ref.T_s*(self.k_o + 1j*self.w_c)*fbk.e_c + 1j*(
            self.cfg.grid_par.w_gN - self.w_c)*self.v_cp
        self.theta_c += ref.T_s*self.w_c
        # Limit to [-pi, pi]
        self.theta_c = wrap(self.theta_c)
