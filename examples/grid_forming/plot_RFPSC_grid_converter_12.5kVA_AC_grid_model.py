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

This example uses a model for the 3-phase voltage source of the AC grid
which takes into account the electromechanical dynamics of a typical grid
generated by synchronous generators. More information about the model can be
found in [#ENT2013]_.

"""

# %%
import time
import numpy as np

from motulator.common.model import Simulation, Inverter
from motulator.common.utils import BaseValues, NominalValues

from motulator.grid import model
import motulator.grid.control.grid_forming as control
from motulator.grid.utils import GridConverterPars, plot_grid


# %%
# Compute base values based on the nominal values (just for figures).

nom = NominalValues(U=400, I=18, f=50, P=12.5e3)
base = BaseValues.from_nominal(nom)


# %%
# Configure the system model.

mdl_par = GridConverterPars(
    u_gN=400*np.sqrt(2/3),
    w_gN=2*np.pi*50,
    L_f=3e-3)

grid_filter = model.LCLFilter(u_gN=mdl_par.u_gN, L_fc=mdl_par.L_f,
                              L_fg=3e-3, C_f=10e-6, L_g=20e-3)
grid_model = model.FlexSource(w_gN=2*np.pi*50, e_g_abs=mdl_par.u_gN,
                              S_grid=500e3, H_g=2, r_d=0.05)
converter = Inverter(u_dc=650)

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
ctrl.ref.U = lambda t: mdl_par.u_gN

# Active power reference
ctrl.ref.p_g = lambda t: ((t > .2)*(6.25e3))

# Load disturbance in the AC grid
mdl.grid_model.p_e = lambda t: (t > .4)*50e3

# Mechanical power reference for the electromechanical model
mdl.grid_model.p_m_ref = lambda t: 0


# %%
# Create the simulation object and simulate it.

start_time = time.time()
sim = Simulation(mdl, ctrl)
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
