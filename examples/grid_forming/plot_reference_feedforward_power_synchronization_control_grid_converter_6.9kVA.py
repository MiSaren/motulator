"""
6.9-kVA grid-forming converter, reference-feedforward PSC (RFPSC)
=================================================================
    
This example simulates a grid-forming controlled converter connected to a
weak grid. The control system includes a power synchronization loop (PSL) to
synchronize with the grid and an inner P-type current controller used to damp
the current oscillations, enhanced with a reference-feedforward term.

"""

# %%
import time
import numpy as np

from motulator.common.model import Simulation, Inverter
from motulator.common.utils import BaseValues, NominalValues

from motulator.grid import model
import motulator.grid.control.grid_forming as control
from motulator.grid.utils import (plot_grid, GridConverterPars)


# %%
# Compute base values based on the nominal values (just for figures).

nom = NominalValues(U=400, I=10, f=50, P=6.9e3)
base = BaseValues.from_nominal(nom)


# %%
# Configure the system model.

mdl_par = GridConverterPars(
    u_gN=400*np.sqrt(2/3),
    w_gN=2*np.pi*50,
    L_f=8e-3)

grid_filter = model.LFilter(u_gN=mdl_par.u_gN, L_f=mdl_par.L_f, L_g=65.8e-3)
grid_model = model.StiffSource(w_gN=mdl_par.w_gN, e_g_abs = mdl_par.u_gN)
converter = Inverter(u_dc=650)

mdl = model.StiffSourceAndGridFilterModel(converter, grid_filter, grid_model)


# %%
# Configure the control system.

ctrl_par = mdl_par

# Control configuration parameters
cfg = control.PSCControlCfg(
        ctrl_par,
        T_s = 1/(8e3),
        on_rf=True,
        i_max = 1.5*base.i,
        R_a = .2*base.Z,
        w_0_cc = 2*np.pi*5)

# Create the control system
ctrl = control.PSCControl(cfg)


# %%
# Set the references for converter output voltage magnitude and active power.

# Converter output voltage magnitude reference (constant)
ctrl.ref.U = lambda t: mdl_par.u_gN

# Active power reference
ctrl.ref.p_g = lambda t: ((t > .2)*(2.3e3) + (t > .5)*(2.3e3) +
    (t > .8)*(2.3e3) - (t > 1.2)*(6.9e3))


# %%
# Create the simulation object and simulate it.

start_time = time.time()
sim = Simulation(mdl, ctrl)
sim.simulate(t_stop = 1.5)
stop_time = time.time()
print(f"Simulation time: {stop_time-start_time:.2f} s")


# %%
# Plot results in per-unit values. By omitting the argument `base` you can plot
# the results in SI units.

plot_grid(sim, base=base, plot_pcc_voltage=True)
