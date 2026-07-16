# ADR 09: soft-body and cloth method
**Status:** accepted.

## Context
Deformable bodies (cloth, soft cubes, elastic objects) require a simulation method that integrates with the existing constraint solver architecture and is deterministic.

## Decision
- **XPBD (Extended Position-Based Dynamics)**: Compliance-based constraints (stretch, shear, bend, volume) with substep iteration. Implemented in `auralite-softbody`.
- **Self-collision**: Spatial hashing (using `auralite-core::SpatialHash`) for broad-phase self-collision detection on deformable meshes.
- **Rigid coupling**: Two-way coupling via attachment constraints.
- **Wind and aerodynamics**: Per-face aerodynamic force model implemented.

## Alternatives
- FEM: More accurate for biomechanics but heavier and harder to make deterministic.
- Classic mass-spring: Prone to unrealistic behavior without extensive tuning.

## Consequences
- XPBD allows stable and deterministic cloth behavior.
- Unified spatial hashing reduces complexity and enables O(n) performance.

## Validation (2026-07-16)
- Verified by `hanging_cloth_converges` test.
- Verified by `soft_cube_volume_stable` test.
- Verified by `self_collision_no_nan` test.
