"""
12.5-kVA grid-forming converter (RFPSC), with electromechanical grid model
==========================================================================
    
This example simulates a grid-forming controlled converter, which uses
reference-feedforward power synchronization control (RFPSC) method connected to
a weak grid. The control system includes a power synchronization loop (PSL) to
synchronize with the grid, and an inner P-type current controller used to damp
the current oscillations enhanced with a reference-feedforward term. The
converter is connected to an AC grid with electromechanical dynamics through an
LCL filter and an inductive impedance.

This example uses a custom model for the 3-phase voltage source of the AC grid,
which takes into account the electromechanical dynamics of a typical grid
generated by synchronous generators. More information about the model can be
found in [#ENT2013]_.

"""

# %%
import time
from types import SimpleNamespace
import numpy as np

from motulator.common.model import Subsystem
from motulator.common.utils import (BaseValues, NominalValues, abc2complex,
                                    complex2abc, wrap)

from motulator.grid import model
import motulator.grid.control.grid_forming as control
from motulator.grid.utils import GridModelPars, plot_grid


# %%
# Define the AC source model which is used in this example.

# sphinx_gallery_start_ignore
# TODO: should the FlexSource class definition be moved to a different file?
# sphinx_gallery_end_ignore

class FlexSource(Subsystem):
    """
    3-phase AC grid including synchronous generator electromechanical dynamics.

    Parameters
    ----------
    T_D : float
        Turbine delay time constant (s).
    T_N : float
        Turbine derivative time constant (s).
    H_g : float
        Grid inertia constant (s).
    r_d : float
        Primary frequency droop control gain (p.u.).
    T_gov : float
        Governor time constant (s).
    w_N : float
        Grid constant frequency (rad/s).
    S_grid : float
        Grid rated power (VA).
    e_g_abs : float | callable
        3-phase grid voltage magnitude, phase-to-ground peak value (V).
    p_m_ref : function
        Mechanical power output reference (W).
    p_e : function
        Electrical power disturbance (W).
        
    """

    def __init__(self, T_D=10, T_N=3, H_g=3, D_g=0, r_d=.05, T_gov=0.5,
                 w_N=2*np.pi*50, S_grid =500e6,
                 e_g_abs=lambda t: 400*np.sqrt(2/3),
                 p_m_ref=lambda t: 0,
                 p_e=lambda t: 0):
        super().__init__()
        self.p_m_ref = p_m_ref
        self.p_e = p_e
        self.par = SimpleNamespace(T_D=T_D, T_N=T_N, H_g=H_g, D_g=D_g,
                                   r_d=r_d*w_N/S_grid, T_gov=T_gov, w_N=w_N,
                                   S_grid=S_grid, e_g_abs=e_g_abs)
        # States
        self.state = SimpleNamespace(err_w_g=0, p_gov=0, x_turb=0,
                                     theta_g=0)
        # Store the solutions in these lists
        self.sol_states = SimpleNamespace(err_w_g=[], p_gov=[],
                                          x_turb=[], theta_g=[])

    def voltages(self, t, theta_g):
        """
        Compute the grid voltage in stationary frame:
           
        Parameters
        ----------
        t : float
            Time.
        theta_g : float
            Grid electrical angle (rad).

        Returns
        -------
        e_gs: complex
            Grid complex voltage (V).

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
        self.out.e_gs = self.voltages(t, wrap(self.state.theta_g.real))

    def set_inputs(self, t):
        """Set input variables."""
        self.inp.p_m_ref = self.p_m_ref(t)
        self.inp.p_e = self.p_e(t)

    def rhs(self):
        """
        Compute the state derivatives.

        Returns
        -------
        Complex list, length 4
            Time derivative of the complex state vector,
            [d_err_w_g, d_p_gov, d_x_turb, d_theta_g].

        """
        par, state, inp = self.par, self.state, self.inp
        err_w_g = state.err_w_g
        p_gov = state.p_gov
        x_turb = state.x_turb

        # calculation of mechanical power from the turbine output
        p_m = (par.T_N/par.T_D)*p_gov + (1 - (par.T_N/par.T_D))*x_turb
        # swing equation
        p_diff = (p_m - inp.p_e)/par.S_grid # in per units
        d_err_w_g = par.w_N*(p_diff - par.D_g*err_w_g)/(2*par.H_g)
        # governor dynamics
        d_p_gov = (inp.p_m_ref - (1/par.r_d)*err_w_g - p_gov)/par.T_gov
        # turbine dynamics (lead-lag)
        d_x_turb = (p_gov - x_turb)/par.T_D
        # integration of the angle
        d_theta_g = par.w_N + err_w_g
        return [d_err_w_g, d_p_gov, d_x_turb, d_theta_g]

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
        e_g_abc = complex2abc(self.voltages(t, self.state.theta_g.real))
        return e_g_abc

    def meas_freq(self):
        """
        Measure the grid frequency.

        Returns
        -------
        w_g : float
            Grid angular frequency (rad/s).

        """
        w_g = self.par.w_N + self.state.err_w_g.real
        return w_g

    def meas_angle(self):
        """
        Measure the grid angle.

        Returns
        -------
        theta_g : float
            Grid electrical angle (rad).

        """
        theta_g = wrap(self.state.theta_g.real)
        return theta_g

    def post_process_states(self):
        """Post-process the solution."""
        self.data.w_g = self.par.w_N + self.data.err_w_g.real
        self.data.theta_g = wrap(self.data.theta_g.real)
        self.data.e_gs=self.voltages(self.data.t, self.data.theta_g)


# %%
# Compute base values based on the nominal values (just for figures).

nom = NominalValues(U=400, I=18, f=50, P=12.5e3)
base = BaseValues.from_nominal(nom)


# %%
# Configure the system model.

mdl_par = GridModelPars(U_gN=400*np.sqrt(2/3), w_g=2*np.pi*50, L_f=3e-3)

grid_filter = model.LCLFilter(U_gN=mdl_par.U_gN, L_fc=mdl_par.L_f,
                              L_fg=3e-3, C_f=10e-6, L_g=20e-3)
grid_model = FlexSource(w_N=2*np.pi*50, S_grid=500e3, H_g=2, r_d = 0.05)
converter = model.Inverter(u_dc=650)

mdl = model.StiffSourceAndGridFilterModel(converter, grid_filter, grid_model)


# %%
# Configure the control system.

ctrl_par = mdl_par

cfg = control.PSCControlCfg(
        ctrl_par,
        T_s = 1/(10e3),
        on_rf=True,
        i_max = 1.5*base.i,
        R_a = .2*base.Z,
        w_0_cc = 2*np.pi*5)

# Create the control system
ctrl = control.PSCControl(cfg)


# %%
# Set the time-dependent reference and disturbance signals.

# Converter output voltage magnitude reference (constant)
ctrl.ref.U = lambda t: mdl_par.U_gN

# Active power reference
ctrl.ref.p_g = lambda t: ((t > .2)*(6.25e3))

# Load disturbance in the AC grid
mdl.grid_model.p_e = lambda t: (t > .4)*50e3

# Mechanical power reference for the electromechanical model
mdl.grid_model.p_m_ref = lambda t: 0


# %%
# Create the simulation object and simulate it.

start_time = time.time()
sim = model.Simulation(mdl, ctrl)
sim.simulate(t_stop = 6)
stop_time = time.time()
print(f"Simulation time: {stop_time-start_time:.2f} s")


# %%
# Plot results in per-unit values. By omitting the argument `base` you can plot
# the results in SI units.

plot_grid(sim, base=base, plot_pcc_voltage=True, plot_w=True)


# %%
# .. rubric:: References
#
# .. [#ENT2013] ENTSO-E, Documentation on Controller Tests in Test Grid
#    Configurations, Technical Report, 26.11.2013.
