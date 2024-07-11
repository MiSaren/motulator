"""
10-kVA grid following converter, power control
==============================================
    
This example simulates a grid following controlled converter connected to a
strong grid. The control system includes a phase-locked loop (PLL) to
synchronize with the grid, a current reference generatior and a PI-based
current controller.

"""

# %%
# Imports.

import time
import numpy as np

from motulator.grid import model
import motulator.grid.control.grid_following as control
#import motulator.grid.control.grid_following as control
from motulator.common.model import Simulation, Inverter
from motulator.grid.utils import (
    GridModelPars, plot_grid)
from motulator.common.utils import (
    BaseValues, NominalValues)

# To check the computation time of the program
start_time = time.time()


# %%
# Compute base values based on the nominal values (just for figures).

nom = NominalValues(U=400, I=14.5, f=50, P=10e3)
base = BaseValues.from_nominal(nom)

# %%
# Create the system model.

par = GridModelPars(
    U_gN=400*np.sqrt(2/3),
    w_g=2*np.pi*50,
    L_f=10e-3,
    C_dc=1e-3)

# grid impedance and filter model
grid_filter = model.LFilter(U_gN=par.U_gN ,R_f=0 ,L_f=par.L_f, L_g=0, R_g=0)
# AC grid model (either constant frequency or dynamic electromechanical model)
grid_model = model.StiffSource(w_N=par.w_g, e_g_abs=par.U_gN)

# Uncomment the following two lines to use a dynamic grid model, with a variable DC voltage
converter = Inverter(u_dc = 650)

mdl = model.StiffSourceAndGridFilterModel(
    converter, grid_filter, grid_model)

on_u_dc = False

# if dc_model is None:
#     mdl = model.StiffSourceAndLFilterModel(
#         converter, grid_filter, grid_model)
#     on_v_dc=False

# if dc_model == model.DCBusVoltageSource:
#     mdl = model.dc_bus.DCBusAndLFilterModel(
#         converter, grid_filter, grid_model, dc_model)
#     on_v_dc=False
# else:
#     mdl = model.dc_bus.DCBusAndLFilterModel(
#         converter, grid_filter, grid_model, dc_model)
#     on_v_dc=True

# %%
# Configure the control system.

# Control parameters
# pars = control.GridFollowingCtrlPars(
#             L_f=10e-3,
#             C_dc = 1e-3,
#             f_sw = 8e3,
#             T_s = 1/(16e3),
#             on_v_dc=on_v_dc,
#             i_max = 1.5*base.i,
#             p_max = base.p,
#             )
# par = GridModelPars(
#     U_gN=400*np.sqrt(2/3),
#     w_g=2*np.pi*50,
#     L_f=10e-3,
#     C_dc=1e-3)
#ctrl = control.GridFollowingCtrl(pars)
cfg = control.GFLControlCfg(
    par,
    on_u_dc=on_u_dc,
    i_max=1.5*base.i,
    p_max=base.p,
)
ctrl = control.GFLControl(cfg)

if on_u_dc:
    ctrl.dc_bus_volt_ctrl = control.DCBusVoltageController(
        cfg.zeta_dc, cfg.w_0_dc, cfg.p_max)
# %%
# Set the time-dependent reference and disturbance signals.

# Set the active and reactive power references
if on_u_dc:
    mdl.dc_model.i_ext = lambda t: (t > .06)*(10)
else:
    ctrl.ref.p_g = lambda t: (t > 0.02)*(5e3)
ctrl.ref.q_g = lambda t: (t > .04)*(4e3)

# AC-voltage magnitude (to simulate voltage dips or short-circuits)
e_g_abs_var =  lambda t: np.sqrt(2/3)*400
mdl.grid_model.e_g_abs = e_g_abs_var # grid voltage magnitude

# DC voltage reference
if on_u_dc:
    ctrl.ref.u_dc = lambda t: 600 + (t > .02)*(50)
   
# %%
# Create the simulation object and simulate it.

#mdl.pwm = model.CarrierComparison()  # Enable the PWM model
sim = Simulation(mdl, ctrl)
sim.simulate(t_stop = .1)

# Print the execution time
print('\nExecution time: {:.2f} s'.format((time.time() - start_time)))

plot_grid(sim=sim, base=base, plot_pcc_voltage=True)
