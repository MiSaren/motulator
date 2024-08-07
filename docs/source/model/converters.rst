Converters
==========

Inverter
--------

The figure below shows a three-phase, two-level voltage source inverter and its equivalent model, where ideal switches are assumed.
In the equivalent model, each changeover switch is connected to either negative or positive potential of the DC-bus.
The switching phenomena are assumed to be infinitely fast.
The inverter model is provided in the class :class:`motulator.common.model.Inverter`. 


.. figure:: figs/inverter.svg
   :width: 100%
   :align: center
   :alt: Three-phase two-level inverter
   :target: .

   Three-phase two-level inverter: (left) main circuit; (right) equivalent model. The DC-bus voltage and currents are :math:`u_\mathrm{dc}` and :math:`i_\mathrm{dc}`, respectively.

By default, the DC-bus voltage is stiff. Alternatively, the capacitive dynamics of the DC-bus can be simulated.
The equivalent circuit with the dynamic model of the DC-bus is shown in the figure below.
An external current source is feeding the DC-bus that is modeled with
a parallel-connected DC-bus capacitor and resitor. The model is implemented as

.. math::
   \frac{\mathrm{d}\boldsymbol{u}_\mathrm{dc}}{\mathrm{d} t} 
   = \frac{1}{C_\mathrm{dc}}(i_\mathrm{ext} 
   - i_\mathrm{dc} - G_\mathrm{dc}u_\mathrm{dc})
   :label: DC_bus_model

where :math:`u_\mathrm{dc}` is the DC-bus voltage, :math:`i_\mathrm{ext}` is the 
external DC current, :math:`i_\mathrm{dc}` is the converter DC current, 
:math:`C_\mathrm{dc}` is the DC-bus capacitance, and :math:`G_\mathrm{dc}` is 
the conductance of the parallel resistor. The converter DC current is calculated from the converter phase currents and switching states as 

.. math::
   i_\mathrm{dc} = q_\mathrm{a} i_\mathrm{a} + q_\mathrm{b} i_\mathrm{b}
   + q_\mathrm{c} i_\mathrm{c}
   :label: DC_current

.. figure:: figs/inverter_dc.svg
   :width: 100%
   :align: center
   :alt: Equivalent model of inverter with DC-bus dynamic model
   :target: .
   
   Equivalent model of inverter with DC-bus dynamic model.

Six-Pulse Diode Bridge
----------------------

The figure below shows a six-pulse diode bridge rectifier, where the inductor :math:`L` is placed in the DC link.
This model is implemented in the class :class:`motulator.common.model.DiodeBridge`.

.. figure:: figs/diode_bridge.svg
   :width: 100%
   :align: center
   :alt: Diode bridge
   :target: .

   Six-pulse diode bridge rectifier.

The model is implemented as 

.. math:: 
   \frac{\mathrm{d}i_{L}}{\mathrm{d}t} = \frac{1}{L_{\mathrm{dc}}}(u_\mathrm{di} - u_\mathrm{dc})
   :label: diode_bridge

where :math:`i_\mathrm{L}` is the DC-bus current, :math:`u_\mathrm{di}` is the diode bridges output DC voltage, :math:`u_\mathrm{dc}` is the dc bus voltage, and :math:`L_{\mathrm{dc}}` is the inductance of the DC link.
   
The use case example of six-pulse diode bridge can be found in :doc:`/auto_examples/vhz/plot_vhz_ctrl_im_2kw` and 
:doc:`/auto_examples/vector/plot_vector_ctrl_pmsm_2kw_diode`

Carrier Comparison
------------------

The figure below shows an inverter equipped with a generic three-phase load.
In pulse-width modulation (PWM), carrier comparison is commonly used to generate
instantaneous switching state signals :math:`q_\mathrm{a}`, :math:`q_\mathrm{b}`,
and :math:`q_\mathrm{c}` from duty ratios :math:`d_\mathrm{a}`, :math:`d_\mathrm{b}`,
and :math:`d_\mathrm{c}`. The duty ratios are continuous signals in the range [0, 1} while the switching states are either 0 or 1.

.. figure:: figs/pwm_inverter.svg
   :width: 100%
   :align: center
   :alt: Inverter and carrier comparison
   :target: .

   Instantaneous switching states are defined by the carrier comparison. In this example, the switching states are :math:`q_\mathrm{a}=1`, :math:`q_\mathrm{b}=0`, and :math:`q_\mathrm{c}=0`.

The figure below shows the principle of carrier comparison. The logic shown in the figure
is implemented in the class :class:`motulator.common.model.CarrierComparison`,
where the switching instants are explicitly computed in the beginning of each sampling period
(instead of searching for zero crossings), allowing faster simulations.

.. figure:: figs/carrier_comparison.svg
   :width: 100%
   :align: center
   :alt: Carrier comparison
   :target: .

   Carrier comparison. The duty ratios are :math:`d_\mathrm{a}`, :math:`d_\mathrm{b}`, and :math:`d_\mathrm{c}` are constant over the sampling period :math:`T_\mathrm{s}` (or, optionally, over the the switching period :math:`T_\mathrm{sw}=2T_\mathrm{s}`). The carrier signal is the same for all three phases and varies between 0 and 1.

The zero-sequence voltage does not affect the phase currents if the neutral of the load is not connected.
Therefore, the reference potential of the phase voltages can be freely chosen when computing the space vector of the converter output voltage.
The converter voltage vector in stationary coordinates is

.. math::
	\boldsymbol{u}_\mathrm{c}^\mathrm{s} &= \frac{2}{3}\left(u_\mathrm{an} + u_\mathrm{bn}\mathrm{e}^{\mathrm{j}2\pi/3} + u_\mathrm{cn}\mathrm{e}^{\mathrm{j} 4\pi/3}\right) \\
	&= \frac{2}{3}\left(u_\mathrm{aN} + u_\mathrm{bN}\mathrm{e}^{\mathrm{j} 2\pi/3} + u_\mathrm{cN}\mathrm{e}^{\mathrm{j} 4\pi/3}\right) \\
   &= \underbrace{\frac{2}{3}\left(q_\mathrm{a} + q_\mathrm{b}\mathrm{e}^{\mathrm{j} 2\pi/3} + q_\mathrm{c}\mathrm{e}^{\mathrm{j} 4\pi/3}\right)}_{\boldsymbol{q}_\mathrm{c}^\mathrm{s}}u_\mathrm{dc}
   :label: carrier_comparison
    
where :math:`\boldsymbol{q}_\mathrm{c}^\mathrm{s}` is the switching-state space vector.

.. note::
   The carrier comparison is compatible with all standard pulse-width modulation (PWM) methods, such as space-vector PWM (see :class:`motulator.common.control.PWM`) and discontinuous PWM methods [#Hol1994]_, [#Hav1999]_.

   The sampling period :math:`T_\mathrm{s}` is returned by the control method, and it does not need to be constant. 

   If the zero sequence is of interest, it could be easily added to the inverter model.

Switching-Cycle Averaging
-------------------------

If the switching ripple is not of interest in simulations, the carrier comparison can be replaced with zero-order hold (ZOH) of the duty ratios.
In this case, the output voltage vector is

.. math::
	\boldsymbol{u}_\mathrm{c}^\mathrm{s} = \underbrace{\frac{2}{3}\left(d_\mathrm{a} + d_\mathrm{b}\mathrm{e}^{\mathrm{j} 2\pi/3} + d_\mathrm{c}\mathrm{e}^{\mathrm{j} 4\pi/3}\right)}_{\boldsymbol{d}_\mathrm{c}^\mathrm{s}}u_\mathrm{dc}
   :label: switching_cycle_averaging

where :math:`\boldsymbol{d}_\mathrm{c}^\mathrm{s}` is the duty ratio space vector. This ZOH is the default option in most of :doc:`/auto_examples/index`.

.. rubric:: References

.. [#Hol1994] Holtz, "Pulsewidth modulation for electronic power conversion," Proc. IEEE, 1994, https://doi.org/10.1109/5.301684

.. [#Hav1999] Hava, Kerkman, Lipo, "Simple analytical and graphical methods for carrier-based PWM-VSI drives," IEEE Trans. Ind. Appl., 1999, https://doi.org/10.1109/63.737592