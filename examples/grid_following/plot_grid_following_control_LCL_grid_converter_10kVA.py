"""
10-kVA grid-following converter with LCL filter, power control
==============================================================
    
This example simulates a grid-following-controlled converter connected to a
strong grid through an LCL filter. The control system includes a phase-locked
loop (PLL) to synchronize with the grid, a current reference generator, and a
PI-based current controller.

"""

# %%
# Imports.

import time

from motulator.common.model import (Simulation, Inverter, ACFilter,
                                    CarrierComparison)
from motulator.common.utils import BaseValues, NominalValues, FilterPars, DCBusPars

from motulator.grid import model
import motulator.grid.control.grid_following as control
from motulator.grid.utils import GridPars, plot_grid

# To check the computation time of the program

start_time = time.time()

# %%
# Compute base values based on the nominal values (just for figures).

nom = NominalValues(U=400, I=14.5, f=50, P=10e3)
base = BaseValues.from_nominal(nom)


# %%
# Configure the system model.

grid_par = GridPars(
    u_gN = base.u,
    w_gN = base.w)

filter_par = FilterPars(
    L_fc = 3.7e-3,
    L_fg = 3.7e-3,
    C_f = 8e-6)

dc_bus_par = DCBusPars(
    u_dc = 650,
    C_dc = 100e-3)

grid_filter = ACFilter(grid_par, filter_par)

# AC-voltage magnitude (to simulate voltage dips or short-circuits)
e_g_abs_var =  lambda t: grid_par.u_gN
# AC grid model with constant voltage magnitude and frequency
grid_model = model.StiffSource(w_gN=grid_par.w_gN, e_g_abs = e_g_abs_var)
# Inverter model with constant DC voltage
converter = Inverter(dc_bus_par)

# Create system model
mdl = model.StiffSourceAndGridFilterModel(
    converter, grid_filter, grid_model)


# %%
# Configure the control system.

# # Control parameters
cfg = control.GFLControlCfg(
    grid_par=grid_par,
    dc_bus_par=dc_bus_par,
    filter_par=filter_par,
    on_u_cap = True,
    i_max=1.5*base.i
    )
ctrl = control.GFLControl(cfg)


# %%
# Set the time-dependent reference and disturbance signals.

# Set the active and reactive power references
ctrl.ref.p_g = lambda t: (t > .02)*(5e3)
ctrl.ref.q_g = lambda t: (t > .04)*(4e3)

# %%
# Create the simulation object and simulate it.

#mdl.pwm = CarrierComparison()  # Enable the PWM model
sim = Simulation(mdl, ctrl)
sim.simulate(t_stop = .1)

# Print the execution time
#print('\nExecution time: {:.2f} s'.format((time.time() - start_time)))


# %%
# Plot results in SI or per unit values.

# By omitting the argument `base` you can plot
# the results in SI units.
plot_grid(sim, base=base, plot_pcc_voltage=True)
