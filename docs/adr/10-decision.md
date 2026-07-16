# ADR 10: particle and fluid method
**Status:** accepted; revisit during M7.

## Context
Physical particles and incompressible fluids require a simulation method compatible with the engine's deterministic design.

## Decision
- **PBF (Position-Based Fluids)** (planned): Position-based constraint for incompressibility with:
  - Deterministic density constraint solve using SPH kernels.
  - Density error correction per substep.
  - Artificial viscosity for damping.
  - Vorticity confinement for small-scale detail.
- **Neighbor search**: Spatial hashing with deterministic sorting for O(n) expected neighbor lookup.
- **Emitter system**: Seeded deterministic emitters producing particles with configurable rate, velocity, lifetime.
- **Buoyancy**: Volume-displacement buoyancy force on rigid bodies immersed in fluid.

## Alternatives
- SPH with explicit integration: requires small time steps for stability.
- FLIP/PIC (grid-based): hybrid approach; more complex to implement deterministically.
- Smoothed-particle explicit: not as stable for incompressible flow.

## Consequences
- PBF allows larger time steps than explicit SPH.
- Deterministic neighbor search ensures replay reliability.
- Two-way coupling adds complexity to the solver dispatch.

## Validation
- Same-seed emitter produces identical particle streams.
- Dam-break simulation settles with density error within 5%.
- Neighbor search results match brute-force O(n²) enumeration.
- Floating box reaches hydrostatic equilibrium within 1% of analytic buoyancy.
