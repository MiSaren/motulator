"""Simulation environment."""

import numpy as np
from scipy.integrate import solve_ivp
from scipy.io import savemat


# %%
class Simulation:
    """
    Simulation environment.

    Each simulation object has a system model object and a controller object.

    Parameters
    ----------
    mdl : Model 
        Continuous-time system model.
    ctrl : ControlSystem
        Discrete-time controller.

    """

    def __init__(self, mdl=None, ctrl=None):
        self.mdl = mdl
        self.ctrl = ctrl

    def simulate(self, t_stop=1, max_step=np.inf):
        """
        Solve the continuous-time model and call the discrete-time controller.

        Parameters
        ----------
        t_stop : float, optional
            Simulation stop time. The default is 1.
        max_step : float, optional
            Max step size of the solver. The default is inf.

        Notes
        -----
        Other options of `solve_ivp` could be easily used if needed, but, for
        simplicity, only `max_step` is included as an option of this method.

        """
        try:
            self._simulation_loop(t_stop, max_step)
        except FloatingPointError:
            print(f"Invalid value encountered at {self.mdl.t0:.2f} seconds.")
        # Post-process the solution data
        self.mdl.post_process()
        self.ctrl.post_process()

    @np.errstate(invalid="raise")
    def _simulation_loop(self, t_stop, max_step):
        """Run the main simulation loop."""
        while self.mdl.t0 <= t_stop:

            # Run the digital controller
            T_s, ref_d_c_abc = self.ctrl(self.mdl)

            # Computational delay model
            d_c_abc = self.mdl.delay(ref_d_c_abc)

            # Carrier comparison
            t_steps, q_cs = self.mdl.pwm(T_s, d_c_abc)

            # Loop over the sampling period T_s
            for i, t_step in enumerate(t_steps):

                if t_step > 0:
                    # Update the converter switching state
                    self.mdl.converter.inp.q_cs = q_cs[i]

                    # Get initial values
                    state0 = self.mdl.get_initial_values()

                    # Integrate over t_span
                    t_span = (self.mdl.t0, self.mdl.t0 + t_step)
                    sol = solve_ivp(
                        self.mdl.rhs, t_span, state0, max_step=max_step)

                    # Set the new initial time
                    self.mdl.t0 = t_span[-1]

                    # Save the solution
                    sol.q_cs = len(sol.t)*[q_cs[i]]
                    self.mdl.save(sol)

    def save_mat(self, name="sim"):
        """
        Save the simulation data into MATLAB .mat files.

        Parameters
        ----------
        name : str, optional
            Name for the simulation instance. The default is `sim`.

        """
        savemat(name + "_mdl_data" + ".mat", self.mdl.data)
        savemat(name + "_ctrl_data" + ".mat", self.ctrl.data)
