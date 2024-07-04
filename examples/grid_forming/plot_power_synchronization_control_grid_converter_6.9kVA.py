"""
6.9-kVA grid-forming converter, power synchronization control (PSC)
===================================================================
    
This example simulates a grid-forming controlled converter connected to a
weak grid. The control system includes a power synchronization loop (PSL) to
synchronize with the grid, and an inner P-type current controller used to damp
the current oscillations.

"""

# %%
import time
import numpy as np

from motulator.common.utils import BaseValues, NominalValues

from motulator.grid import model
import motulator.grid.control.grid_forming as control
from motulator.grid.utils import (plot_grid, GridModelPars)


# %%
# Compute base values based on the nominal values (just for figures).

nom = NominalValues(U=400, I=10, f=50, P=6.9e3)
base = BaseValues.from_nominal(nom)


# %%
# Configure the system model.

mdl_par = GridModelPars(U_gN=400*np.sqrt(2/3), w_g=2*np.pi*50, L_f=8e-3)

grid_filter = model.LFilter(U_gN=mdl_par.U_gN, L_f=mdl_par.L_f, L_g=65.8e-3)

# Grid voltage source with constant frequency and voltage magnitude
grid_model = model.StiffSource(w_N=mdl_par.w_g, e_g_abs = mdl_par.U_gN)

# Inverter with constant DC voltage
converter = model.Inverter(u_dc=650)

# Create system model
mdl = model.StiffSourceAndGridFilterModel(converter, grid_filter, grid_model)

# Uncomment line below to enable the PWM model
#mdl.pwm = model.CarrierComparison()


# %%
# Configure the control system.

# Use the actual model parameters
par = mdl_par

# Set the configuration parameters
cfg = control.PSCControlCfg(
        par,
        T_s = 1/(8e3),
        i_max = 1.5*base.i,
        R_a = .2*base.Z,
        w_0_cc = 2*np.pi*5)

# Create the control system
ctrl = control.PSCControl(cfg)


# %%
# Set the references for converter output voltage magnitude and active power.

# Converter output voltage magnitude reference (constant)
ctrl.ref.U = lambda t: mdl_par.U_gN

# Active power reference
ctrl.ref.p_g = lambda t: ((t > .2)*(2.3e3) + (t > .5)*(2.3e3) +
    (t > .8)*(2.3e3) - (t > 1.2)*(6.9e3))


# %%
# Create the simulation object and simulate it.

start_time = time.time()
sim = model.Simulation(mdl, ctrl)
sim.simulate(t_stop = 1.5)
stop_time = time.time()
print(f"Simulation time: {stop_time-start_time:.2f} s")


# %%
# Plot results in per-unit values. By omitting the argument `base` you can plot
# the results in SI units.

plot_grid(sim=sim, base=base, plot_pcc_voltage=True)
