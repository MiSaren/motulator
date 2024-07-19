"""Common dataclasses usable in models and control of grid converters."""

# %%
from abc import ABC
from dataclasses import dataclass


# %%
# TODO: extend class to include parameters for LCL filter, grid impedance,
# optionally there could be a parent class which is inherited (subclasses could
# be LFilterModelPars and LCLFilterModelPars?)
@dataclass
class GridModelPars(ABC):
    """
    Class for grid parameters
    
    Parameters
    ----------
    U_gN : float
        Nominal grid voltage, phase-to-ground peak value (V).
    w_gN : float
        Nominal grid angular frequency (rad/s).
    L_f : float
        Filter inductance (H).
    C_dc : float
        DC bus capacitance (F).
    
    """
    U_gN: float = None
    w_gN: float = None
    L_f: float = None
    C_dc: float = None
