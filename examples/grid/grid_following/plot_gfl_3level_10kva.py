"""
10-kVA converter
================

This example simulates a grid-following-controlled converter connected to an L
filter and a strong grid. The control system includes a phase-locked loop (PLL)
to synchronize with the grid, a current reference generator, and a PI-based
current controller.

"""

# %%
import numpy as np

from motulator.grid import model, control
from motulator.grid.utils import (
    BaseValues, ACFilterPars, NominalValues, plot)
# from motulator.grid.utils import plot_voltage_vector

# %%
# Compute base values based on the nominal values.

nom = NominalValues(U=400, I=14.5, f=50, P=10e3)
base = BaseValues.from_nominal(nom)

# %%
# Configure the system model.

# Filter and grid
par = ACFilterPars(L_fc=.2*base.L)
ac_filter = model.ACFilter(par)
ac_source = model.ThreePhaseVoltageSource(w_g=base.w, abs_e_g=base.u)
# Inverter with constant DC voltage
converter = model.ThreeLevelConverter(u_dc=650, C_dc1=.5e-3, C_dc2=.5e-3)

# Create system model
mdl = model.GridConverterSystem(converter, ac_filter, ac_source)
mdl.pwm = model.CarrierComparison(
    return_complex=False, level=3)  # Uncomment to enable the PWM model

# %%
# Configure the control system.

cfg = control.GridFollowingControlCfg(
    L=.2*base.L, nom_u=base.u, nom_w=base.w, max_i=1.5*base.i)
ctrl = control.GridFollowingControl(cfg)

# Add the DC-bus voltage controller to the control system
ctrl.dc_bus_voltage_ctrl = control.DCBusVoltageController(
    C_dc=1e-3, alpha_dc=2*np.pi*30, max_p=base.p)

# %%
# Set the time-dependent reference and disturbance signals.

# Set the references for DC-bus voltage and reactive power
ctrl.ref.u_dc = lambda t: 650
ctrl.ref.q_g = lambda t: (t > .04)*4e3

# Set the external current fed to the DC bus
mdl.converter.i_dc = lambda t: (t > .06)*10

# Uncomment lines below to simulate an unbalanced fault (add negative sequence)
# mdl.ac_source.par.abs_e_g = .75*base.u
# mdl.ac_source.par.abs_e_g_neg = .25*base.u
# mdl.ac_source.par.phi_neg = -np.pi/3

# %%
# Create the simulation object and simulate it.

import time

start_time = time.time()

sim = model.Simulation(mdl, ctrl)
sim.simulate(t_stop=.1)

print(time.time() - start_time)

# %%
# Plot the results.

# By default results are plotted in per-unit values. By omitting the argument
# `base` you can plot the results in SI units.

# Uncomment line below to plot locus of the grid voltage space vector
# plot_voltage_vector(sim, base)
plot(sim, base, plot_pcc_voltage=False)
