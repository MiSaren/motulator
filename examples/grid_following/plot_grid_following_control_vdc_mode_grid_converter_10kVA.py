"""
10-kVA grid following converter, dc-bus voltage control
=======================================================
    
This example simulates a grid following controlled converter connected to a
strong grid and regulating the dc-bus voltage at the same time. The control
system includes a DC-bus voltage controller, a phase-locked loop (PLL) to
synchronize with the grid, a current reference generatior and a PI-based
current controller.

"""

# %%
# Imports.

import time
import numpy as np

from motulator.common.model import Simulation, Inverter, CarrierComparison
from motulator.common.utils import BaseValues, NominalValues, FilterPars, DCBusPars

from motulator.grid import model
import motulator.grid.control.grid_following as control
from motulator.grid.utils import GridPars, plot_grid
from motulator.grid.control import DCBusVoltageController

# To check the computation time of the program
start_time = time.time()


# %%
# Compute base values based on the nominal values (just for figures).

nom = NominalValues(
    U=400,
    I=14.5,
    f=50,
    P=10e3)
base = BaseValues.from_nominal(nom)


# %%
# Configure the system model.

# Grid parameters
grid_par = GridPars(
    u_gN = base.u,
    w_gN = base.w)

# Filter parameters
filter_par = FilterPars(L_fc = 10e-3)

# DC-bus parameters
DC_bus_par = DCBusPars(
    u_dc = 600,
    C_dc = 1e-3)

grid_filter = model.LFilter(grid_par, filter_par)

# AC-voltage magnitude (to simulate voltage dips or short-circuits)
e_g_abs_var =  lambda t: base.u
# AC grid model with constant voltage magnitude and frequency
grid_model = model.StiffSource(w_gN=grid_par.w_gN,e_g_abs = e_g_abs_var)

# Inverter model, u_dc in DC bus parameter is the initial DC voltage
converter = Inverter(DC_bus_par)

# Create system model
mdl = model.StiffSourceAndGridFilterModel(
    converter, grid_filter, grid_model)


# %%
# Configure the control system.

# Control parameters
cfg = control.GFLControlCfg(
            grid_par,
            DC_bus_par,
            filter_par,
            on_u_dc= True,
            i_max = 1.5*base.i,
            p_max = base.p,
            )
ctrl = control.GFLControl(cfg)

# %%
# Set the time-dependent reference and disturbance signals.

# Set the active and reactive power references
if cfg.on_u_dc:
    ctrl.dc_bus_volt_ctrl = DCBusVoltageController(
        cfg.zeta_dc, cfg.w_0_dc, cfg.p_max)
    mdl.converter.i_ext = lambda t: (t > .06)*(10)
    ctrl.ref.u_dc= lambda t: 600 + (t > .02)*(50)
else:
    ctrl.ref.p_g = lambda t: (t > .02)*(5e3)
ctrl.ref.q_g = lambda t: (t > .04)*(4e3)


# %%
# Create the simulation object and simulate it.

sim = Simulation(mdl, ctrl)
sim.simulate(t_stop = .1)

# Print the execution time
#print('\nExecution time: {:.2f} s'.format((time.time() - start_time)))


# %%
# Plot results in SI or per unit values.

# By omitting the argument `base` you can plot
# the results in SI units.
plot_grid(sim=sim, base=base, plot_pcc_voltage=True)
