"""
10-kVA grid-following converter, power control
==============================================
    
This example simulates a grid-following-controlled converter connected to a
strong grid. The control system includes a phase-locked loop (PLL) to
synchronize with the grid, a current reference generator, and a PI-based
current controller.

"""

# %%
# Imports.

import time
import numpy as np

from motulator.common.model import Simulation, Inverter, CarrierComparison
from motulator.common.utils import BaseValues, NominalValues

from motulator.grid import model
import motulator.grid.control.grid_following as control
from motulator.grid.utils import GridConverterPars, plot_grid, plot_voltage_vector


# %%
# Compute base values based on the nominal values (just for figures).

nom = NominalValues(U=400, I=14.5, f=50, P=10e3)
base = BaseValues.from_nominal(nom)

# %%
# Configure the system model.

mdl_par = GridConverterPars(
    u_gN=400*np.sqrt(2/3),
    w_gN=2*np.pi*50,
    L_f=10e-3,
    C_dc=1e-3)
grid_filter = model.LFilter(u_gN=mdl_par.u_gN, L_f=mdl_par.L_f)
# AC grid model with constant voltage magnitude and frequency
grid_model = model.StiffSource(w_gN=mdl_par.w_gN, e_g_abs=mdl_par.u_gN)
# Inverter with constant DC voltage
converter = Inverter(u_dc = 650)

mdl = model.StiffSourceAndGridFilterModel(
    converter, grid_filter, grid_model)

# %%
# Configure the control system.

# Control configuration parameters
cfg = control.GFLControlCfg(
    mdl_par,
    i_max=1.5*base.i,
    p_max=base.p,
)
ctrl = control.GFLControl(cfg)


# %%
# Set the time-dependent reference and disturbance signals.

# Set the active and reactive power references
ctrl.ref.p_g = lambda t: (t > 0.02)*(5e3)
ctrl.ref.q_g = lambda t: (t > 0.04)*(4e3)

# Uncomment the following line to simulate a single-phase voltage dip
#mdl.grid_model.par.e_gb_abs = lambda t: mdl_par.U_gN*(1 - 0.5*(t > 0.05))


# %%
# Create the simulation object and simulate it.

#mdl.pwm = model.CarrierComparison()  # Enable the PWM model
start_time = time.time()
sim = Simulation(mdl, ctrl)
sim.simulate(t_stop = .1)

# Print the execution time
#print('\nExecution time: {:.2f} s'.format((time.time() - start_time)))


# %%
# Plot results per-unit values.

# By omitting the argument `base` you can plot
# the results in SI units.

# Plot the locus of the grid voltage space vector
plot_voltage_vector(sim=sim, base=base)

plot_grid(sim=sim, base=base, plot_pcc_voltage=True)
