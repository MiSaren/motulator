Grid Voltage Source
===================

This document describes continuous-time models for a three-phase AC voltage source.
This section will be extended in the future.

Stiff Source
------------

A model for a stiff grid voltage is implemented in the class :class:`motulator.grid.model.StiffSource`.
The model is an ideal three-phase voltage source, where the phase voltages are

.. math::
    e_\mathrm{ga} &= \hat{e}_\mathrm{ga}\cos(\vartheta_\mathrm{g}) \\
    e_\mathrm{gb} &= \hat{e}_\mathrm{gb}\cos(\vartheta_\mathrm{g} - 2\pi/3) \\
    e_\mathrm{gc} &= \hat{e}_\mathrm{gc}\cos(\vartheta_\mathrm{g} - 4\pi/3)
    :label: grid_voltages

The grid phase angle is simply :math:`\vartheta_\mathrm{g} = \omega_g t`, where
the angular frequency of the grid can either be constant or a time-dependent function.

.. TODO: nonsymmetric faults

Flex Source
-----------

A grid model taking into account the electromechanical dynamics of synchronous
generators is implemented in the class :class:`motulator.grid.model.FlexSource`.
See the example :doc:`/auto_examples/grid_forming/plot_RFPSC_grid_converter_12.5kVA_AC_grid_model`.