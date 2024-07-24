"""Common dataclasses usable in models and control of grid converters."""

# %%
from abc import ABC
from dataclasses import dataclass


# %%
# TODO: extend class to include parameters for LCL filter, grid impedance,
# optionally there could be a parent class which is inherited (subclasses could
# be LFilterModelPars and LCLFilterModelPars?)
@dataclass
class GridConverterPars(ABC):
    """
    Class for grid and grid converter parameters
    
    Parameters
    ----------
    u_gN : float
        Nominal grid voltage, phase-to-ground peak value (V).
    w_gN : float
        Nominal grid angular frequency (rad/s).
    L_f : float
        Filter inductance (H).
    C_dc : float
        DC bus capacitance (F).
    
    """
    u_gN: float = None
    w_gN: float = None
    L_f: float = None
    C_dc: float = None

@dataclass
class GridPars(ABC):
    """
    Class for grid parameters

    Parameters
    ----------
    u_gN : float
        Nominal grid voltage, phase-to-ground peak value (V).
    w_gN : float
        Nominal grid angular frequency (rad/s).
    L_g : float, optional
        Grid inductance (H). The default is 0.
    R_g : float, optional
        Grid resistance (Ω). The default is 0.

    """
    u_gN: float = None
    w_gN: float = None
    L_g: float = 0
    R_g: float = 0

@dataclass
class FilterPars(ABC):
    """
    Class for grid filter parameters

    Parameters
    ----------
    L_fc : float
        Converter-side inductance of the filter (H).
    L_fg : float, optional
        Grid-side inductance of the filter (H). The default is 0.
    C_f : float, optional
        Filter capacitance (F). The default is 0.
    R_fc : float, optional
        Converter-side series resistance (Ω). The default is 0.
    R_fg : float, optional
        Grid-side series resistance (Ω). The default is 0.
    G_f : float, optional
        Conductance of a resistor in parallel with the filter capacitor (S).
        The default is 0.

    """
    L_fc: float
    L_fg: float = 0
    C_f: float = 0
    R_fc: float = 0
    R_fg: float = 0
    G_f: float = 0

@dataclass
class DCBusPars(ABC):
    """
    Class for DC bus parameters

    Parameters
    ----------
    u_dc : float | callable
        DC bus voltage (V).
    C_dc : float, optional
        DC bus capacitance (F). The default is None.
    L_f : float, optional
        Filter inductance (H). The default is 0.
    R_dc : float, optional
        DC bus series resistance (Ω). The default is 0.
    G_dc : float, optional
        DC bus conductance (S). The default is 0.

    """
    u_dc: float
    C_dc: float = None
    L_f: float = 0
    R_dc: float = 0
    G_dc: float = 0
