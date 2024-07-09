"""
6.9-kVA grid forming converter, power synchronization control (PSC)
===================================================================
    
This example simulates a grid forming controlled converter connected to a
weak grid. The control system includes a power synchronization loop (PSL) to
synchronize with the grid, an inner P_type current controller used to damp the
current oscillations and an optional DC-bus voltage controller.

"""

# %%
# Imports.

import time
import numpy as np

from motulator.grid import model
import motulator.grid.control.grid_forming as control
from motulator.grid.utils import (
    BaseValues, NominalValues, plot_grid, GridModelPars)

# To check the computation time of the program
start_time = time.time()


# %%
# Compute base values based on the nominal values (just for figures).

nom = NominalValues(U=400, I=10, f=50, P=6.9e3)
base = BaseValues.from_nominal(nom)

# %%
# Create the system model.

mdl_par = GridModelPars(
    U_gN=400*np.sqrt(2/3), w_g=2*np.pi*50, L_f=8e-3, C_dc=1e-3)

grid_filter = model.LFilter(
    U_gN=mdl_par.U_gN, R_f=0, L_f=mdl_par.L_f, L_g=65.8e-3, R_g=0)
grid_model = model.StiffSource(w_N=mdl_par.w_g)
converter = model.InverterWithVariableDC()

#dc_model = model.dc_bus.DCBus(C_dc = 1e-3, u_dc0=600, G_dc=0)

# Uncomment the following two lines to use a static grid model, with a fixed DC voltage
#converter = model.Inverter(u_dc=650)
dc_model = model.DCBusVoltageSource(u_dc=650)

mdl = model.dc_bus.DCBusAndLFilterModel(
    converter, grid_filter, grid_model, dc_model)
#mdl.pwm = model.CarrierComparison()  # Enable the PWM model


# %%
# Configure the control system.

# Set variable below to True to use DC bus voltage control mode
on_u_dc = False

# Control parameters
cfg = control.PSCControlCfg(
        mdl_par,
        T_s = 1/(8e3),
        on_rf=False,
        on_u_dc=on_u_dc,
        i_max = 1.5*base.i,
        R_a = .2*base.Z,
        w_0_cc = 2*np.pi*5)
ctrl = control.PSCControl(cfg)

if on_u_dc:
    ctrl.dc_bus_volt_ctrl = control.DCBusVoltageController(
        zeta=1, alpha_dc=2*np.pi*30)


# %%
# Set the time-dependent reference and disturbance signals.

# Converter output voltage magnitude reference
ctrl.ref.U = lambda t: mdl_par.U_gN

# Set the active power reference
if on_u_dc:
    mdl.dc_model.i_ext = lambda t: (t > .6)*(10)
else:
    ctrl.ref.p_g = lambda t: ((t > .2)*(2.3e3) + (t > .5)*(2.3e3) +
        (t > .8)*(2.3e3) - (t > 1.2)*(6.9e3))

# AC-voltage magnitude (to simulate voltage dips or short-circuits)
e_g_abs_var =  lambda t: mdl_par.U_gN
mdl.grid_model.e_g_abs = e_g_abs_var # grid voltage magnitude

# DC voltage reference
if on_u_dc:
    ctrl.ref.u_dc = lambda t: 600 + (t > .2)*(50)

# Create the simulation object and simulate it
sim = model.Simulation(mdl, ctrl)
sim.simulate(t_stop = 1.5)

# Print the execution time
print('\nExecution time: {:.2f} s'.format((time.time() - start_time)))


# %%
# Plot results in SI or per unit values.

plot_grid(sim=sim, base=base, plot_pcc_voltage=True)
