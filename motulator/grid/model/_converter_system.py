"""
Continuous-time grid converter system model.

This interconnects the subsystems of a converter with a grid and provides an
interface to the solver. More complicated systems could be modeled using a
similar template. Peak-valued complex space vectors are used.

"""
from motulator.common.model import Model


# %%
class GridConverterSystem(Model):
    """
    Continuous-time model for a grid converter system.

    Parameters
    ----------
    converter : VoltageSourceConverter
        Converter model.
    ac_filter : LFilter | LCLFilter
        Dynamic model for converter output filter and grid impedance.
    ac_source : ThreePhaseVoltageSource
        Three-phase grid voltage source model.

    """

    def __init__(self, converter=None, ac_filter=None, ac_source=None):
        super().__init__()
        self.converter = converter
        self.ac_filter = ac_filter
        self.ac_source = ac_source
        self.subsystems = [self.converter, self.ac_filter, self.ac_source]

    def interconnect(self, _):
        """Interconnect the subsystems."""
        self.converter.inp.i_cs = self.ac_filter.out.i_cs
        self.ac_filter.inp.u_cs = self.converter.out.u_cs
        self.ac_filter.inp.e_gs = self.ac_source.out.e_gs

    def post_process(self):
        """Post-process the solution."""
        # Post-processing based on the states
        super().post_process_states()
        # Add the input data to the subsystems for post-processing
        self.converter.data.i_cs = self.ac_filter.data.i_cs
        self.ac_filter.data.u_cs = self.converter.data.u_cs
        self.ac_filter.data.e_gs = self.ac_source.data.e_gs
        # Post-processing based on the inputs and the states
        super().post_process_with_inputs()


# %%
class GridConverterWithDCSource(Model):
    """
    Continuous-time model for a grid converter with DC-side current source.

    Parameters
    ----------
    converter : VoltageSourceConverter
        Converter model.
    ac_filter : LFilter | LCLFilter
        Dynamic model for converter output filter and grid impedance.
    ac_source : ThreePhaseVoltageSource
        Three-phase grid voltage source model.
    dc_source : DCPowerSource
        DC-side source.

    """

    def __init__(
            self,
            converter=None,
            ac_filter=None,
            ac_source=None,
            dc_source=None):
        super().__init__()
        self.converter = converter
        self.ac_filter = ac_filter
        self.ac_source = ac_source
        self.dc_source = dc_source
        self.subsystems = [
            self.converter, self.ac_filter, self.ac_source, self.dc_source
        ]

    def interconnect(self, _):
        """Interconnect the subsystems."""
        self.dc_source.inp.u_dc = self.converter.out.u_dc
        self.converter.inp.i_dc = self.dc_source.out.i_dc
        self.converter.inp.i_cs = self.ac_filter.out.i_cs
        self.ac_filter.inp.u_cs = self.converter.out.u_cs
        self.ac_filter.inp.e_gs = self.ac_source.out.e_gs

    def post_process(self):
        """Post-process the solution."""
        # Post-processing based on the states
        super().post_process_states()
        # Add input data for direct feedthrough
        self.dc_source.data.u_dc = self.converter.data.u_dc
        # Post-processing direct feedthrough values
        super().post_process_direct_feedthrough()
        # Add the input data to the subsystems for post-processing
        self.converter.data.i_dc = self.dc_source.data.i_dc
        self.converter.data.i_cs = self.ac_filter.data.i_cs
        self.ac_filter.data.u_cs = self.converter.data.u_cs
        self.ac_filter.data.e_gs = self.ac_source.data.e_gs
        # Post-processing based on the inputs and the states
        super().post_process_with_inputs()
