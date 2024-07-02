"""Helper functions and classes."""

# %%
from abc import ABC
from dataclasses import dataclass


# %%
@dataclass
class GridModelPars(ABC):
    """
    Class for grid parameters
    
    Parameters
    ----------
    U_gN : float
        Nominal grid voltage, phase-to-ground peak value (V).
    w_g : float
        Nominal grid angular frequency (rad/s).
    L_f : float
        Filter inductance (H).
    C_dc : float
        DC bus capacitance (F).
    
    """
    U_gN: float = None
    w_g: float = None
    L_f: float = None
    C_dc: float = None
