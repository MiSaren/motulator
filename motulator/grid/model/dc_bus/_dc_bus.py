"""
Dynamic model of a DC bus.

A DC bus between an external current source or sink and a converter is modeled 
considering an equivalent circuit comprising a capacitor and parallel resistor.
"""
from types import SimpleNamespace

from motulator.common.model import Subsystem

# %%
# TODO: remove as obsolete (moved to Inverter class)
class DCBusVoltageSource(Subsystem):
    """"
    DC bus model with a constant voltage source.

    This model is used when the DC bus voltage is constant. The voltage is given 
    as a parameter and the DC current is an input to the model.
    """
    def __init__(self, u_dc):
        super().__init__()
        self.u_dc = u_dc
        self.out = SimpleNamespace(u_dc = [])
        if callable(u_dc):
            self.state = SimpleNamespace(u_dc = u_dc(0))
        else:
            self.state = SimpleNamespace(u_dc = u_dc)
        self.sol_states = SimpleNamespace(u_dc = [])
        self.inp = SimpleNamespace(i_dc = 0)

    def set_outputs(self,t):
        """Set input variables."""
        if callable(self.u_dc):
            self.state.u_dc = self.u_dc(t).real
        self.out.u_dc = self.state.u_dc.real

    def rhs(self):
        """
        Set the state derivatives to zero.
        """
        # State derivative
        du_dc = 0
        return [du_dc]
    
    def meas_dc_voltage(self):
        """
        Measure the DC bus voltage at the end of the sampling period.
    
        Returns
        -------
        u_dc: float
            DC bus voltage (V)
    
        """
        return self.state.u_dc
    
    def post_process_states(self):
        """Post-process data."""
        self.data.u_dc = self.data.u_dc.real
 
class DCBus(Subsystem):
    """
    DC bus model

    This model is used to compute the capacitive DC bus dynamics. Dynamics are 
    modeled with an equivalent circuit comprising a capacitor and its parallel 
    resistor that is parametrized using a conductance. The capacitor voltage is 
    used as a state variable.

    Parameters
    ----------
    C_dc : float
        DC-bus capacitance (F)
    G_dc : float
        Parallel conductance of the capacitor (S)
    i_ext : function
        External DC current, seen as disturbance, `i_ext(t)`.

    """
    def __init__(self, C_dc=1e-3, G_dc=0, i_ext=lambda t: 0, u_dc0 = 650):
        super().__init__()
        self.par = SimpleNamespace(C_dc=C_dc, G_dc=G_dc, i_ext=i_ext)
        self.udc0 = u_dc0
        self.i_ext = i_ext
        # Initial values
        self.state = SimpleNamespace(u_dc = u_dc0)
        self.inp = SimpleNamespace(i_dc = 0, i_ext = i_ext(0))
        self.sol_states = SimpleNamespace(u_dc = [])

    def set_outputs(self, _):
        """Set output variables."""
        state, out = self.state, self.out
        out.u_dc = state.u_dc.real

    def set_inputs(self, t):
        """Set input variables."""
        self.inp.i_ext = self.i_ext(t)

    def rhs(self):
        """
        Compute the state derivatives.

        Parameters
        ----------
        t : float
            Time (s)
        u_dc: float
            DC bus voltage (V)
        i_dc : float
            Converter DC current (A)
        Returns
        -------
        double list, length 1
                Time derivative of the complex state vector, [du_dc]

        """
        # State derivative
        du_dc = (self.inp.i_ext - self.inp.i_dc - self.par.G_dc*self.state.u_dc)/self.par.C_dc
        return [du_dc]

    def meas_dc_voltage(self):
        """
        Measure the DC bus voltage at the end of the sampling period.
    
        Returns
        -------
        u_dc: float
            DC bus voltage (V)
    
        """
        return self.state.u_dc

    def post_process_states(self):
        """Post-process data."""
        self.data.u_dc = self.data.u_dc.real
