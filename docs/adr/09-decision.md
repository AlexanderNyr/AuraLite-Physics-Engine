# ADR 09: soft-body and cloth method
**Status:** accepted; revisit during M6.

## Context
Deformable bodies (cloth, soft cubes, elastic objects) require a simulation method that integrates with the existing constraint solver architecture and is deterministic.

## Decision
- **XPBD (Extended Position-Based Dynamics)** (planned): Compliance-based constraints (stretch, shear, bend, volume) with substep iteration. Chosen over FEM and mass-spring systems because:
  - Natural integration with the existing sequential-impulse constraint pipeline.
  - Deterministic and controllable with iteration counts.
  - Well-understood cloth and soft-body behavior.
- **Self-collision**: Spatial hashing for broad-phase self-collision detection on deformable meshes.
- **Rigid coupling**: Two-way coupling via attachment constraints (pin, weld, distance) between deformable vertices and rigid bodies.
- **Wind and aerodynamics**: Per-face aerodynamic force model.

## Alternatives
- FEM: More accurate for biomechanics but heavier and harder to make deterministic.
- Classic mass-spring: Prone to unrealistic behavior without extensive tuning.
- FTL (Finite Volume): Proven but complex for real-time.

## Consequences
- XPBD requires careful compliance tuning to avoid excessive stretch/bounce.
- Self-collision spatial hash adds memory cost but ensures O(n) expected performance.
- Cloth folding and crumpling naturally handled.

## Validation
- Hanging cloth converges with stretch error below 1% under gravity.
- Soft cube volume error within 5% over 60 s.
- Folded cloth self-collision stable (no NaN or explosion).
- Deterministic replays identical across 10,000 steps.
