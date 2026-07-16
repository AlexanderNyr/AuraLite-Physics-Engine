# ADR 08: solver and stabilization strategy
**Status:** accepted.

## Context
Rigid-body contacts and constraints must be resolved with a stable, deterministic solver. Stacking, friction, and restitution require iterative methods.

## Decision
- **Sequential Impulse Solver** (planned): Gauss-Seidel style constraint solver with warm starting from manifold cached impulses. Configurable iteration count with penetration stabilization and substep support.
- **Penetration stabilization**: Baumgarte-style correction with slop threshold.
- **Constraint types** (planned): Contact (normal + friction), weld/point-to-point limits, distance/spring, motor, and joint limits.
- **Sleeping**: Bodies below a kinetic energy threshold enter sleep after a stabilization period; island-based wake propagation.

## Alternatives
- Direct complementarity solvers (Lemke, PATH): more robust but significantly more complex and harder to make deterministic.
- XPBD for rigid contacts: possible but adds compliance tuning complexity compared to traditional impulse-based methods.
- Global LCP solves: not real-time suitable for many contacts.

## Consequences
- Sequential impulses are the industry standard for real-time physics and well-understood for determinism.
- Penetration stabilization may introduce visible correction; slop threshold keeps small penetrations silent.
- Iteration count must be configurable to trade speed for accuracy.

## Validation
- Stacking tower/pyramid remains stable for 60 simulated seconds at 60 Hz (penetration ≤ CONTACT_SLOP).
- Restitution and friction match analytic predictions within documented tolerance.
- Deterministic replay: 10,000-step ×3 produces identical state hash (Tier A).
