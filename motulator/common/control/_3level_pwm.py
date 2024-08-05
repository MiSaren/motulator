"""Common functions and classes for controls."""

from abc import ABC, abstractmethod
from types import SimpleNamespace

import numpy as np

from motulator.common.utils import abc2complex, complex2abc


class ThreeLevelPWM_NTV:
    """
    3-level PWM modulation.

    Parameters
    ----------
    cfg: PSCControlCfg
        Model and controller configuration parameters.
        
    """

    def __init__(self, cfg):
        """
        Parameters
        ----------
        cfg : PSCControlCfg
           Control parameters.
    
        """
        self.cfg = cfg
        self.sqrt3 = np.sqrt(3)

    def find_sector(self, ref_u_cs):
        """
        Find the sector of the reference voltage.

        Parameters
        ----------
        ref_u_cs : complex
            Voltage reference in the stator coordinates.

        Returns
        -------
        sector : int
            Sector of the reference voltage.
        """

        # Normalize the angle to be within the range [0, 2*pi)
        theta = np.angle(ref_u_cs) % (2*np.pi)

        # Define the boundary values
        boundary1 = np.pi/3
        boundary2 = 2*np.pi/3
        boundary3 = np.pi
        boundary4 = 4*np.pi/3
        boundary5 = 5*np.pi/3
        boundary6 = 2*np.pi

        # Determine the sector
        if 0 <= theta < boundary1:
            return 1
        elif boundary1 <= theta < boundary2:
            return 2
        elif boundary2 <= theta < boundary3:
            return 3
        elif boundary3 <= theta < boundary4:
            return 4
        elif boundary4 <= theta < boundary5:
            return 5
        elif boundary5 <= theta < boundary6:
            return 6

    def determine_region(self, V_alpha, V_beta, sector):
        # Normalize V_alpha and V_beta to the range of -1 to 1
        V_alpha_norm = V_alpha/(self.u_dc/2)
        V_beta_norm = V_beta/(self.u_dc/2)

        if sector == 1:
            if V_beta_norm <= (self.sqrt3*V_alpha_norm):
                if V_beta_norm >= 0.5*V_alpha_norm:
                    return 1
                else:
                    return 2
            else:
                if V_beta_norm >= 0.5*V_alpha_norm:
                    return 3
                else:
                    return 4
        elif sector == 2:
            if V_beta_norm >= (self.sqrt3*V_alpha_norm):
                if V_beta_norm <= 0.5*V_alpha_norm:
                    return 1
                else:
                    return 2
            else:
                if V_beta_norm <= 0.5*V_alpha_norm:
                    return 3
                else:
                    return 4
        elif sector == 3:
            if V_beta_norm <= (-self.sqrt3*V_alpha_norm):
                if V_beta_norm >= -0.5*V_alpha_norm:
                    return 1
                else:
                    return 2
            else:
                if V_beta_norm >= -0.5*V_alpha_norm:
                    return 3
                else:
                    return 4
        elif sector == 4:
            if V_beta_norm >= (-self.sqrt3*V_alpha_norm):
                if V_beta_norm <= -0.5*V_alpha_norm:
                    return 1
                else:
                    return 2
            else:
                if V_beta_norm <= -0.5*V_alpha_norm:
                    return 3
                else:
                    return 4
        elif sector == 5:
            if V_beta_norm <= (self.sqrt3*V_alpha_norm):
                if V_beta_norm >= 0.5*V_alpha_norm:
                    return 1
                else:
                    return 2
            else:
                if V_beta_norm >= 0.5*V_alpha_norm:
                    return 3
                else:
                    return 4
        elif sector == 6:
            if V_beta_norm >= (-self.sqrt3*V_alpha_norm):
                if V_beta_norm <= -0.5*V_alpha_norm:
                    return 1
                else:
                    return 2
            else:
                if V_beta_norm <= -0.5*V_alpha_norm:
                    return 3
                else:
                    return 4
