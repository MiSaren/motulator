DC-Bus Voltage Control
======================
The DC-bus voltage control uses a two-degrees-of-freedom (2DOF) proportional-integral (PI) controller. 
The PI controller is designed to control the energy of the DC-bus capacitance 
and not the DC-bus voltage in order to have a linear closed-loop system. [#Hur2001]_

The DC-bus voltage control is implemented in the class :class:`motulator.grid.control.DCBusVoltageController`.

.. rubric:: References
    
.. [#Hur2001] Hur, Jung, Nam, "A Fast Dynamic DC-Link Power-Balancing
       Scheme for a PWM Converter–Inverter System," IEEE Trans. Ind. Electron.,
       2001, https://doi.org/10.1109/41.937412