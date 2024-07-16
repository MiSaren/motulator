Synchronization Methods
=======================

Phase-Locked Loop
-----------------

TODO

Power Synchronization
---------------------

The :doc:`/auto_examples/grid_forming/index` examples use active power synchronization as a means of
synchronizing with the grid. The dynamics of a synchronous machine are emulated,
as the converter output active power is tied to the angle of the converter output voltage.
This allows for synchronization of a converter with the grid without the use of a PLL.
More details on the control methods used can be found in [#Har2019]_ and [#Har2020]_.

The power synchronization is implemeted as

.. math::
    \frac{\mathrm{d}\vartheta_\mathrm{c}}{\mathrm{d}t} = \omega_\mathrm{g} + k_\mathrm{p} (p_\mathrm{g,ref} - p_\mathrm{g})
    :label: psl

where :math:`\vartheta_\mathrm{c}` is the converter output voltage angle, :math:`\omega_\mathrm{g}` the nominal grid angular frequency and
:math:`k_\mathrm{p}` the active power control gain. Furthermore, :math:`p_\mathrm{g,ref}` and :math:`p_\mathrm{g}` are the reference and
realized value for the converter active power output, respectively. The active power output is calculated from the measured converter current
and the realized converter output voltage obtained from the PWM.

.. rubric:: References

.. [#Har2019] Harnefors, Hinkkanen, Riaz, Rahman, Zhang, "Robust Analytic Design of Power-Synchronization Control," IEEE Trans. Ind. Electron., Aug. 2019, https://doi.org/10.1109/TIE.2018.2874584

.. [#Har2020] Harnefors, Rahman, Hinkkanen, Routimo, "Reference-Feedforward Power-Synchronization Control," IEEE Trans. Power Electron., Sep. 2020, https://doi.org/10.1109/TPEL.2020.2970991
