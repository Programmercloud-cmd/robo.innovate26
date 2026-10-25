# Eccentric native-weld bookkeeping diagnostic

This is one two-actor diagnostic using the installed BrickSim binary. It is not
an alternative to the required 300-brick workcell benchmark, which still fails
its idle gate. There is no robot, manipulation, training or hardware control.

A kinematic native 2×4 lower brick supports one dynamic native 2×4 upper brick,
rotated 90°, with one stud of overlap. Existing connection geometry determines
the relative pose and interface frames. One genuine native link is initialized
before measurement; no attachment or pose is edited during measurement.

The run is capped at eight steps, first native/finite/pose failure, and 180 seconds.
Default timestep, source solver counts 32/4, epsilon 1e-4, native clutch strengths,
materials and break rules are retained. Only native debug output is enabled.
All execution and array evaluation require the default Pueue queue and frozen
snapshots. No retry, second geometry, parameter sweep or native build is planned.

Only the dynamic upper brick's **external impulse after native weld subtraction**
is compared with the gravity-only reference: linear impulse m·g·dt and angular
impulse about its actual COM of zero, conditional on the observed damping,
reference frame and timing. A legitimate eccentric weld reaction moment is not
an error. Raw observations, assumptions and descriptive SI/normalized residuals
must stay distinct; no percentage threshold establishes calibrated physical
correctness. The kinematic lower reaction is outside this criterion.

Passing would not validate redundant lattice constraints, stability of the full
source, sensing, extraction or insertion. A native break alone is not proof of
bookkeeping error or incorrect one-stud capacity.

Execution and independent audit are pending. This diagnostic does not reset the
existing Stage 2 improvement budget; two unsuccessful improvements are preserved.
