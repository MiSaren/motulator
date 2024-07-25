"""
2.2-kW PMSM, diode bridge
=========================

This example simulates sensorless current-vector control of a 2.2-kW PMSM 
drive, equipped with a diode bridge rectifier. 

"""
# %%
import numpy as np

from motulator.common.model import (
    Simulation, CarrierComparison, Inverter, DiodeBridge)
from motulator.common.utils import BaseValues, NominalValues, DCBusPars

from motulator.drive import model
from motulator.grid.model import StiffSource
import motulator.drive.control.sm as control
from motulator.drive.utils import (
    plot, plot_extra, SynchronousMachinePars)
from motulator.grid.utils import GridPars

# %%
# Compute base values based on the nominal values (just for figures).

nom = NominalValues(U=370, I=4.3, f=75, P=2.2e3, tau=14)
base = BaseValues.from_nominal(nom, n_p=3)

# %%
# Configure the system model.

# Grid parameters
grid_par = GridPars(
    u_gN=base.u,
    w_gN=base.w)

# DC bus parameters
dc_bus_par = DCBusPars(
    u_dc = 400*np.sqrt(2),
    C_dc = 235e-6,
    L_dc = 2e-3)

mdl_par = SynchronousMachinePars(
    n_p=3, R_s=3.6, L_d=.036, L_q=.051, psi_f=.545)
machine = model.SynchronousMachine(mdl_par)
mechanics = model.StiffMechanicalSystem(J=.015)

# %%
# Frequency converter with a diode bridge
ac_source = StiffSource(
    w_gN=grid_par.w_gN,
    e_g_abs=grid_par.u_gN)

diode_bridge = DiodeBridge(dc_bus_par)
converter = Inverter(dc_bus_par)
mdl = model.DriveWithDiodebridge(
    voltage_source=ac_source,
    diodebridge=diode_bridge,
    converter=converter,
    machine=machine,
    mechanics=mechanics
)
mdl.pwm = CarrierComparison()  # Enable the PWM model

# %%
# Configure the control system.

par = mdl_par  # Assume accurate machine model parameter estimates
ref = control.CurrentReferenceCfg(par, nom_w_m=base.w, max_i_s=1.5*base.i)
ctrl = control.CurrentVectorControl(
    par, ref, J=.015, T_s=250e-6, sensorless=True)

# %%
# Set the speed reference and the external load torque.

# Speed reference (electrical rad/s)
ctrl.ref.w_m = lambda t: (t > .2)*base.w

# External load torque
mdl.mechanics.tau_L = lambda t: (t > .6)*nom.tau

# %%
# Create the simulation object and simulate it.

# Simulate the system
sim = Simulation(mdl, ctrl)
sim.simulate(t_stop=1)

# Plot results in per-unit values
plot(sim, base)
plot_extra(sim, base, t_span=(.8, .825))
