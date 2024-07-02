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

from motulator.common.utils import BaseValues, NominalValues

from motulator.grid import model
import motulator.grid.control.grid_following as control
from motulator.grid.utils import plot_grid

# To check the computation time of the program
start_time = time.time()


# %%
# Compute base values based on the nominal values (just for figures).

nom = NominalValues(U=400, I=14.5, f=50, P=10e3)
base = BaseValues.from_nominal(nom)


# %%
# Create the system model.

# grid impedance and filter model
grid_filter = model.LFilter(
    U_gN=400*np.sqrt(2/3), R_f=0 ,L_f=10e-3, L_g=0, R_g=0)
#grid_filter = model.LCLFilter(U_gN=400*np.sqrt(2/3), L_fc=3.7e-3, C_f=8e-6, L_fg = 3.7e-3, L_g=0, R_g=0)

# AC-voltage magnitude (to simulate voltage dips or short-circuits)
e_g_abs_var =  lambda t: np.sqrt(2/3)*400

# AC grid model
grid_model = model.StiffSource(w_N=2*np.pi*50, e_g_abs = e_g_abs_var)

# Inverter model
converter = model.Inverter(u_dc=600, C_dc = 1e-3)

mdl = model.StiffSourceAndLFilterModel(
    converter, grid_filter, grid_model)

on_v_dc=True


# %%
# Configure the control system.

# Control parameters
pars = control.GridFollowingCtrlPars(
            L_f=10e-3,
            C_dc = 1e-3,
            f_sw = 8e3,
            T_s = 1/(16e3),
            on_v_dc=on_v_dc,
            i_max = 1.5*base.i,
            p_max = base.p,
            )
ctrl = control.GridFollowingCtrl(pars)


# %%
# Set the time-dependent reference and disturbance signals.

# Set the active and reactive power references
if on_v_dc:
    mdl.converter.i_ext = lambda t: (t > .06)*(10)
    ctrl.u_dc_ref = lambda t: 600 + (t > .02)*(50)
else:
    ctrl.p_g_ref = lambda t: (t > .02)*(5e3)
ctrl.q_g_ref = lambda t: (t > .04)*(4e3)


# %%
# Create the simulation object and simulate it.

sim = model.Simulation(mdl, ctrl)
sim.simulate(t_stop = .1)

# Print the execution time
print('\nExecution time: {:.2f} s'.format((time.time() - start_time)))


# %%
# Plot results in SI or per unit values.

plot_grid(sim=sim, base=base, plot_pcc_voltage=True)
