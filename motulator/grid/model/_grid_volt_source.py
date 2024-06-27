"""
3-phase AC voltage source models.

Peak-valued complex space vectors are used.

Two voltage sources with variable voltage magnitude, to simulate voltage dips
and symmetrical short circuits are modeled. A stiff model with a constant
frequency and a dynamic model with the electromechanical dynamics of a
synchronous generator are considered. In this module, all space vectors are in
stationary coordinates.

The grid angle theta_g is used 
    as state variables.

        theta_g = w_N * t

"""
from types import SimpleNamespace

import numpy as np

from motulator.common.model import Subsystem
from motulator.common.utils._utils import (complex2abc, abc2complex)

# %%
# TODO: implement simulation of harmonics, negative sequence
class StiffSource(Subsystem):
    """
    3-phase voltage source model.

    This model is a 3-phase voltage source for the AC grid.

    The grid angle theta_g is used as a state variable.

    Parameters
    ----------
    w_N : float
        Grid nominal frequency (rad/s).
    e_g_abs : float | callable
        3-phase grid voltage magnitude, phase-to-ground peak value (V).
    w_g : callable, optional
        Grid frequency (rad/s) as a function of time, `w_g(t)`. If given, w_g
        will be used to compute grid voltage angle instead of w_N.
    """

    def __init__(self, w_N=2*np.pi*50, e_g_abs=400*np.sqrt(2/3), w_g=None):
        super().__init__()
        self.w_g=w_g
        self.par = SimpleNamespace(w_N=w_N, e_g_abs=e_g_abs)
        # states
        self.state = SimpleNamespace(exp_j_theta_g=complex(1))
        # Store the solutions in these lists
        self.sol_states = SimpleNamespace(exp_j_theta_g=[])

    def voltages(self, t, theta_g):
        """
        Compute the grid voltage in stationary frame.
           
        Parameters
        ----------
        t : float
            Time (s).
        theta_g : float
            Grid voltage angle (rad)

        Returns
        -------
        e_gs: complex
            grid complex voltage (V).

        """

        # Calculation of the three-phase voltages
        e_g_abs = self.par.e_g_abs(t) if callable(
            self.par.e_g_abs) else self.par.e_g_abs
        e_g_a = e_g_abs*np.cos(theta_g)
        e_g_b = e_g_abs*np.cos(theta_g-2*np.pi/3)
        e_g_c = e_g_abs*np.cos(theta_g-4*np.pi/3)

        e_gs = abc2complex([e_g_a, e_g_b, e_g_c])
        return e_gs

    def set_outputs(self, t):
        """Set output variables."""
        self.out.e_gs = self.voltages(t, np.angle(self.state.exp_j_theta_g))

    def set_inputs(self, t):
        """Set input variables."""
        self.inp.w_g = self.w_g(t) if callable(self.w_g) else self.par.w_N

    def rhs(self):
        """
        Compute the state derivatives.
        
        Returns
        -------
        list, length 1
            Time derivatives of the state vector.
            
        """
        d_exp_j_theta_g = 1j*self.inp.w_g*self.state.exp_j_theta_g
        return [d_exp_j_theta_g]

    def meas_voltages(self, t):
        """
        Measure the grid phase voltages.
        
        Parameters
        ----------
        t : float
            Time (s).

        Returns
        -------
        e_g_abc : 3-tuple of floats
            Phase voltages (V).

        """
        e_g_abc = complex2abc(self.voltages(t, np.angle(self.state.exp_j_theta_g)))
        return e_g_abc

    def post_process_states(self):
        """Post-process the solution."""
        if callable(self.w_g):
            self.data.w_g = self.w_g(self.data.t)
        else:
            self.data.w_g = np.full(np.size(self.data.t), self.par.w_N)
        self.data.theta_g = np.angle(self.data.exp_j_theta_g)
        self.data.e_gs=self.voltages(self.data.t, self.data.theta_g)
