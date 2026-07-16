# Progress
## Current milestone: M6 complete on 2026-07-16; M7 pending
M0–M5 remain green. M6 work completed this session:

**Soft Bodies & Cloth (auralite-softbody crate):**

- XPBD particle system with pre_step/solve_constraints/post_step pipeline
- Constraint types: Stretch, Bend, Volume (tetrahedral gradient), Attachment
- `build_cloth_grid`: rows×cols grid with stretch/shear (diagonal) / bend (skip-1) constraints
- `build_cloth_strip`: narrow hanging strip builder
- `build_soft_cube`: 8-corner cube with 12 edge stretch + 6 tetra volume constraints
- Self-collision via `SpatialHash` (cell-indexed broad phase) with pairwise correction
- Rigid body coupling: `apply_rigid_coupling_2d` and `apply_rigid_coupling_3d`
- Wind/aerodynamics and damping integrated in pre_step
- Kinetic energy diagnostic

**Tests:** 7 softbody tests: hanging cloth convergence, soft cube volume stability, self-collision (no NaN), cloth strip hangs, kinetic energy finite, attachment pulls toward target, spatial hash queries.

**Gates:** fmt, strict clippy, **99 unit tests** (30 collision, 22 dynamics, 21 geometry, 11 math, 7 softbody, 4 core, 3 serialize, 1 ffi) + f64 math (11 tests), release build — all green.

## Resume pointer
M6 complete. Continue M7: particles, fluids, buoyancy, fields — seeded deterministic emitters, SoA storage, PBF fluid with density/incompressibility solve, buoyancy from displaced volume, field force zones.
