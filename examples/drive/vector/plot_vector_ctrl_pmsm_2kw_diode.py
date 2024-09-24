"""
2.2-kW PMSM, diode bridge
=========================

This example simulates sensorless current-vector control of a 2.2-kW PMSM
drive, equipped with a diode bridge rectifier.

"""
# %%
import time

from motulator.drive import model
import motulator.drive.control.sm as control
from motulator.drive.utils import (
    BaseValues, NominalValues, plot, plot_extra, SynchronousMachinePars)

# %%
# Compute base values based on the nominal values (just for figures).

nom = NominalValues(U=370, I=4.3, f=75, P=2.2e3, tau=14)
base = BaseValues.from_nominal(nom, n_p=3)

# %%
# Configure the system model.

# Machine model and mechanical subsystem
mdl_par = SynchronousMachinePars(
    n_p=3, R_s=3.6, L_d=.036, L_q=.051, psi_f=.545)
machine = model.SynchronousMachine(mdl_par)
mechanics = model.StiffMechanicalSystem(J=.015)

# Frequency converter with a diode bridge
# converter = model.FrequencyConverter(C_dc=235e-6, L_dc=2e-3, U_g=400, f_g=50)
# TODO: Testing the FrequencyConverterWithACInductor model
converter = model.FrequencyConverterWithACInductor(
    C_dc=235e-6, L_g=1e-3, U_g=400, f_g=50, G_dc=1e-3, i_thr=.05)
mdl = model.Drive(converter, machine, mechanics)

mdl.pwm = model.CarrierComparison()  # Enable the PWM model

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

start_time = time.time()

# Simulate the system
sim = model.Simulation(mdl, ctrl)
sim.simulate(t_stop=1)

stop_time = time.time()
print(f"Simulation time: {stop_time-start_time:.2f} s")

# Plot results in per-unit values
plot(sim, base)
plot_extra(sim, base, t_span=(.8, .825))
# plot_extra(sim, base, t_span=(.1, .125))
