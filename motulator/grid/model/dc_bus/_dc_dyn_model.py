"""
DC-bus and filter model interconnectors.

    This module interconnects the subsystems of a converter with a grid and
    provides an interface to the solver. More complicated systems could be
    modeled using a similar template. Peak-valued complex space vectors are
    used. The space vector models are implemented in stationary coordinates.

"""
from types import SimpleNamespace
import numpy as np

from motulator.common.model import Model
from motulator.common.utils import complex2abc

# %%
# TODO: combine models to a single class (e.g. GridConverterSystem) which can
# be used with constant or dynamic dc voltage, different filters, ac models
class DCBusAndLFilterModel(Model):
    """
    Continuous-time model for a stiff grid model with an RL impedance model.

    Parameters
    ----------
    grid_filter : LFilter
        RL line dynamic model.
    grid_model : StiffSource
        Constant voltage source model.
    dc_model : DCBus
        DC-bus voltage dynamics.
    converter : Inverter | PWMInverter
        Inverter model.

    """

    def __init__(self, converter=None, grid_filter=None,
                 grid_model=None, dc_model=None):
        super().__init__()
        self.converter = converter
        self.grid_filter = grid_filter
        self.grid_model = grid_model
        self.dc_model = dc_model
        self.subsystems = [self.converter, self.grid_filter,
                           self.grid_model, self.dc_model]

    def interconnect(self, _):
        """Interconnect the subsystems."""
        # TODO: implement a way to connect signals without having to
        # list them again in post_process() below. Maybe as a SimpleNamespace-
        # object where the key corresponds to a signal name and the value is a
        # list with two subsystems, corresponding to the input and output
        # e.g. interconnects=SimpleNamespace(i_cs=[self.converter,self.grid_filter], u_cs=[self.grid_filter,self.converter])

        # other possibility: in post_process() call interconnect() again but
        # replace inp and out with data
        self.converter.inp.i_cs = self.grid_filter.out.i_cs
        self.grid_filter.inp.u_cs = self.converter.out.u_cs
        self.grid_filter.inp.e_gs = self.grid_model.out.e_gs
        self.dc_model.inp.i_dc = self.converter.out.i_dc
        self.converter.inp.u_dc = self.dc_model.out.u_dc

    def post_process(self):
        """Post-process the solution."""
        # Post-processing based on the states
        super().post_process_states()
        # Add the input data to the subsystems for post-processing
        # TODO: move calculation of u_cs and i_dc to a method inside the Inverter class
        self.converter.data.u_dc = self.dc_model.data.u_dc
        self.converter.data.u_cs = self.converter.ac_voltage(self.converter.data.q_cs, self.converter.data.u_dc)
        self.converter.data.i_cs = self.grid_filter.data.i_cs
        self.converter.data.i_dc = self.converter.dc_current(self.converter.data.q_cs, self.converter.data.i_cs)
        self.grid_filter.data.u_cs = self.converter.data.u_cs
        self.grid_filter.data.e_gs = self.grid_model.data.e_gs
        self.dc_model.data.i_dc = self.converter.data.i_dc
        # Post-processing based on the inputs and the states
        super().post_process_with_inputs()


# %%
class DCBusAndLCLFilterModel:
    """
    Continuous-time model for a stiff grid model with an LCL impedance model.

    Parameters
    ----------
    grid_filter : LCLFilter
        LCL filter dynamic model.
    grid_model : StiffSource
        Constant voltage source model.
    dc_model : DCBus
        DC-bus voltage dynamics.
    converter : Inverter | PWMInverter
        Inverter model.

    """
    
    def __init__(
            self, grid_filter=None, grid_model=None,
            dc_model=None, converter=None):
        self.grid_filter = grid_filter
        self.grid_model = grid_model
        self.dc_model = dc_model
        self.converter = converter

        # Initial time
        self.t0 = 0

        # Store the solution in these lists
        self.data = SimpleNamespace()
        self.data.t, self.data.q = [], []
        self.data.i_gs = []
        self.data.i_cs = []
        self.data.u_fs = []
        self.data.u_dc = []
        self.data.i_dc = []

    def get_initial_values(self):
        """
        Get the initial values.

        Returns
        -------
        x0 : complex list, length 4
            Initial values of the state variables.

        """
        x0 = [
            self.grid_filter.i_cs0,
            self.grid_filter.u_fs0,
            self.grid_filter.i_gs0,
            self.dc_model.u_dc0]

        return x0

    def set_initial_values(self, t0, x0):
        """
        Set the initial values.

        Parameters
        ----------
        x0 : complex ndarray
            Initial values of the state variables.

        """
        self.t0 = t0
        self.grid_filter.i_cs0 = x0[0]
        self.grid_filter.u_fs0 = x0[1]
        self.grid_filter.i_gs0 = x0[2]
        self.dc_model.u_dc0 = x0[3].real
        self.converter.u_dc0 = x0[3].real
        # calculation of grid-side voltage
        e_gs0 = self.grid_model.voltages(t0)
        # update pcc voltage
        self.grid_filter.u_gs0 = self.grid_filter.pcc_voltages(
                                                    x0[2], x0[1], e_gs0)

    def rhs(self, t, x):
        """
        Compute the complete state derivative list for the solver.

        Parameters
        ----------
        t : float
            Time.
        x : complex ndarray
            State vector.

        Returns
        -------
        complex list
            State derivatives.

        """
        # Unpack the states
        i_cs, u_fs, i_gs, u_dc = x
        # Interconnections: outputs for computing the state derivatives
        u_cs = self.converter.ac_voltage(self.converter.q, self.converter.u_dc0)
        e_gs = self.grid_model.voltages(t)
        q = self.converter.q
        i_c_abc = complex2abc(i_cs)
        i_dc = self.dc_model.dc_current(i_c_abc, q) # DC-current
        # State derivatives
        lcl_f = self.grid_filter.f(i_cs, u_fs, i_gs, u_cs, e_gs)
        dc_f = self.dc_model.f(t, u_dc, i_dc)
        # List of state derivatives
        all_f = [lcl_f[0], lcl_f[1], lcl_f[2], dc_f]
        return all_f

    def save(self, sol):
        """
        Save the solution.

        Parameters
        ----------
        sol : SimpleNamespace object
            Solution from the solver.

        """
        self.data.t.extend(sol.t)
        self.data.i_cs.extend(sol.y[0])
        self.data.u_fs.extend(sol.y[1])
        self.data.i_gs.extend(sol.y[2])
        self.data.u_dc.extend(sol.y[3].real)
        self.data.q.extend(sol.q)
        q_abc=complex2abc(np.asarray(sol.q))
        i_c_abc=complex2abc(sol.y[0])
        self.data.i_dc.extend(
            q_abc[0]*i_c_abc[0] + q_abc[1]*i_c_abc[1] + q_abc[2]*i_c_abc[2])
                                
    def post_process(self):
        """
        Transform the lists to the ndarray format and post-process them.

        """
        # From lists to the ndarray
        self.data.t = np.asarray(self.data.t)
        self.data.i_cs = np.asarray(self.data.i_cs)
        self.data.u_fs = np.asarray(self.data.u_fs)
        self.data.i_gs = np.asarray(self.data.i_gs)
        self.data.u_dc = np.asarray(self.data.u_dc)
        self.data.q = np.asarray(self.data.q)
        # Some useful variables
        self.data.i_dc = np.asarray(self.data.i_dc)
        self.data.e_gs = self.grid_model.voltages(self.data.t)
        self.data.theta = np.mod(self.data.t*self.grid_model.w_N, 2*np.pi)
        self.data.u_cs = self.converter.ac_voltage(self.data.q, self.data.u_dc)
        self.data.u_gs = self.grid_filter.pcc_voltages(
            self.data.i_gs,
            self.data.u_fs,
            self.data.e_gs)
