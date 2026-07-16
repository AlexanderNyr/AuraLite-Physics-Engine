# Progress
## Current milestone: M4 complete on 2026-07-16; M5 starting
M0–M3 remain green. M4 work completed this session:

**Full rigid-body worlds 2D+3D with rotation, colliders, solver, sleeping, sensors:**

- `Body2`/`Body3` with rotation (`Rot2`/`Quat`), angular velocity, inertia, damping
- `Collider2`/`Collider3` system: Circle, Box, Capsule, ConvexPolygon, ConvexHull3, TriangleMesh, Edge
- `CollisionFilter` per collider: layers, masks, groups, sensor flag
- Sequential impulse solver (10 iterations) with warm starting from Manifold2
- Friction (Coulomb) with tangent impulse clamping
- `CombineMode`: Multiply/Average/Min/Max/First for material combination
- Implicit ground contact at y=0 (filter-aware)
- Sleeping/island detection with `wake_body()`
- Sensor events: begin/stay/end via broad-phase pair diff tracking
- Snapshot/restore/state_hash for 2D and 3D
- `World2` step pipeline: integrate velocities → positions → ground contact → broad phase (DynamicTree) → narrow phase (circle-circle + GJK/EPA) → solver → manifold update → sensor events → sleeping
- `stacking2d` example: 5 boxes stacked on ground

**Tests:** 17 dynamics tests (was 3): falling, rollback, invalid dt, box collider, two circles stack, restitution bounce, sensor events, static immovable, forces, angular velocity, multiple colliders, invalid input, stale handles, wake/sleep, contact filter, capsule collider, large mass ratio.

**Gates:** fmt, strict clippy, 87 unit tests (30 collision, 17 dynamics, 21 geometry, 11 math, 4 core, 3 serialize, 1 ffi), f64 math (11 tests), release build, sandbox + falling + stacking2d examples — all green.

## Resume pointer
M4 complete. Resume M5: joints/constraints (weld, distance/rope, spring, revolute/hinge, prismatic, ball-socket, cone-twist) with limits, motors, breakables, ragdoll example (≥11 bodies).
