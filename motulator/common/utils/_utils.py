"""Common dataclasses."""

# %%
from dataclasses import dataclass

import numpy as np


# %%
@dataclass
class NominalValues:
    """
    Nominal values.

    Parameters
    ----------
    U : float
        Voltage (V, rms, line-line).
    I : float
        Current (A, rms).
    f : float
        Frequency (Hz).
    P : float
        Power (W).
    tau : float, optional
        Torque (Nm). Default value is None.

    """

    U: float
    I: float
    f: float
    P: float
    tau: float = None


# %%
@dataclass
class BaseValues:
    # pylint: disable=too-many-instance-attributes
    """
    Base values.

    Parameters
    ----------
    u : float
        Voltage (V, peak, line-neutral).
    i : float
        Current (A, peak).
    w : float
        Angular frequency (rad/s).
    psi : float
        Flux linkage (Vs).
    p : float
        Power (W).
    Z : float
        Impedance (Ω).
    L : float
        Inductance (H).
    C : float
        Capacitance (F).
    tau : float, optional
        Torque (Nm). Default is None.
    n_p : int, optional
        Number of pole pairs. Default is None.

    """
    u: float
    i: float
    w: float
    psi: float
    p: float
    Z: float
    L: float
    C: float
    tau: float = None
    n_p: int = None

    @classmethod
    def from_nominal(cls, nom, n_p=None):
        """
        Compute base values from nominal values.

        Parameters
        ----------
        nom : NominalValues
            Nominal values containing the following fields:

                U : float
                    Voltage (V, rms, line-line).
                I : float
                    Current (A, rms).
                f : float
                    Frequency (Hz).

        n_p : int, optional
            Number of pole pairs. If not given it is assumed that base values
            for a grid converter are calculated. Default is None.

        Returns
        -------
        BaseValues
            Base values.

        Notes
        -----
        Notice that the nominal torque is larger than the base torque due to 
        the power factor and efficiency being less than unity.

        """
        u = np.sqrt(2/3)*nom.U
        i = np.sqrt(2)*nom.I
        w = 2*np.pi*nom.f
        psi = u/w
        p = 1.5*u*i
        Z = u/i
        L = Z/w
        C = 1/(Z*w)

        if n_p is not None:
            tau = n_p*p/w
            return cls(
                u=u, i=i, w=w, psi=psi, p=p, Z=Z, L=L, C=C, tau=tau, n_p=n_p)
        return cls(u=u, i=i, w=w, psi=psi, p=p, Z=Z, L=L, C=C)
