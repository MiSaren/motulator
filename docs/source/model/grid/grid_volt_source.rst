Grid Voltage Source
===================

This document describes continuous-time models for a three-phase AC voltage source.
This section will be extended in the future.

Stiff Source
------------

A model for a stiff grid voltage is implemented in the class :class:`motulator.grid.model.StiffSource`.
The model is an ideal three-phase voltage source, where the phase voltages are

.. math::
    e_\mathrm{ga} &= \hat{e}_\mathrm{ga}\cos(\vartheta_\mathrm{p} - \phi) \\
    e_\mathrm{gb} &= \hat{e}_\mathrm{gb}\cos(\vartheta_\mathrm{p} - 2\pi/3 - \phi) \\
    e_\mathrm{gc} &= \hat{e}_\mathrm{gc}\cos(\vartheta_\mathrm{p} - 4\pi/3 - \phi)
    :label: grid_voltages

The peak values of the phase voltages are :math:`\hat{e}_\mathrm{ga}`, :math:`\hat{e}_\mathrm{gb}`
and :math:`\hat{e}_\mathrm{gc}`, which can be given separately as time-dependent functions to
simulate nonsymmetric grid faults. Furthermore, :math:`\vartheta_\mathrm{p}` is the
grid phase voltage angle and :math:`\phi` an additional phase shift. If the phase voltages are symmetric,
:math:`\vartheta_\mathrm{g}=\vartheta_\mathrm{p}` holds, where :math:`\vartheta_\mathrm{g}` is the
angle of the grid voltage space vector :math:`\boldsymbol{e}_\mathrm{g}^\mathrm{s}`.
If there is nonsymmetry in the phase voltages, a zero-sequence component

.. math::
    e_\mathrm{g0} = \frac{1}{3}\left(e_\mathrm{ga} + e_\mathrm{gb} + e_\mathrm{gc}\right) 
    :label: grid_zerosequence

is also present. This component is calculated separately, as the information of the zero-sequence
is lost in the space vector transformation.

Flex Source
-----------

A grid model taking into account the electromechanical dynamics of synchronous
generators is implemented in the class :class:`motulator.grid.model.FlexSource`.
See the example :doc:`/auto_examples/grid_forming/plot_RFPSC_grid_converter_12.5kVA_AC_grid_model`.