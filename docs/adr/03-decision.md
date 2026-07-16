# ADR 03: coordinates, units, and scale
**Status:** accepted; validated at M1.

## Decision
Right-handed coordinates, +Y up, radians, metres, kilograms, seconds. Recommended f32 features are 0.01–100 m and coordinates within ±10 km; users should rebase beyond this. f64 math supports wider coordinate ranges. Degenerate classification combines `ABS_EPSILON=1e-6` and `REL_EPSILON=1e-5`; contact slop is 0.005 m.
## Alternatives
+Z up and centimetre units were rejected because the selected convention matches both native 2D and common gameplay expectations.
## Consequences
Imported assets may need conversion. Scale-dependent tolerances are explicit.
## Validation
Tests cover 1 mm through 1 km primitive scales, large collinear inputs, transform round trips and finite rejection.
