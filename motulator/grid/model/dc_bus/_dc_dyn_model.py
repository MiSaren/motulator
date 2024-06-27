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
# TODO: remove, these models are now obsolete as DC bus is implemented as
# part of Inverter class and not a separate subsystem
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
