# Progress
## Current milestone: M7 complete on 2026-07-16; M8 pending
M0–M6 remain green. M7 work completed this session:

**Particles, Fluids, Buoyancy, Force Fields (auralite-particles crate):**

- `ParticleStorage`: SoA (Struct of Arrays) storage with free-list recycling
- `Emitter`: Seeded deterministic emitter with directional cone + speed spread
- `PbfFluid`: SPH density (poly6 kernel), gradient (spiky), lambda solve, XSPH viscosity, compressibility iterations
- `compute_buoyancy`: Archimedes buoyancy from displaced volume
- `ForceField`: Uniform gravity, Radial (attract/repel), Wind (directional + turbulence), Drag (linear/quadratic), Damping
- Force fields apply to particles within radius with falloff
- Neighbor list built via O(n²) brute force, deterministic

**Tests:** 10 particle tests: deterministic emitter, kill/recycle, capacity, PBF density, uniform/radial/drag fields, buoyancy, lifetime, field radius

**Gates:** fmt, strict clippy, **109 unit tests** (30 collision, 22 dynamics, 21 geometry, 11 math, 10 particles, 7 softbody, 4 core, 3 serialize, 1 ffi) + f64 math (11 tests), release build — all green.

## Resume pointer
M7 complete. Continue M8: vehicles (3D ray/shape-cast wheels, suspension, steering, engine/brake/drivetrain, slip-based tires) and character controllers (2D+3D shape-cast move-and-collide, grounding, slope limits, step climbing, moving platforms).
