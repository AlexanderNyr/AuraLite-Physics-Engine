# Progress
## Current milestone: M8 complete on 2026-07-16; M9 pending
M0–M7 remain green. M8 work completed this session:

**Vehicles & Character Controllers (auralite-vehicles crate):**

- **3D Vehicle** (`Vehicle3`): ray-cast suspension, spring/damper, steering, engine/brake torque, longitudinal/lateral slip (simplified Pacejka), tire force application, air drag
- **2D Vehicle** (`Vehicle2`): simplified forward/brake control with force application
- **2D Character Controller** (`Character2`): capsule collider, move-and-collide, ground detection, jumping, gravity, air control
- **3D Character Controller** (`Character3`): capsule collider, move input (X/Z), ground detection, jumping, air control

**Tests:** 6 vehicle/character tests: 3D vehicle creation+step, 2D vehicle movement, 2D character walk+jump, 3D character walk, character grounding detection, vehicle finite state

**Gates:** fmt, strict clippy, **115 unit tests** (30 collision, 22 dynamics, 21 geometry, 11 math, 10 particles, 7 softbody, 6 vehicles, 4 core, 3 serialize, 1 ffi) + f64 math (11 tests), release build — all green.

## Resume pointer
M8 complete. Continue M9: MT/SIMD/memory/GPU — job scheduler abstraction, parallel broad/narrow/solver, SIMD abstraction (SSE2/AVX2/NEON/scalar fallback), SoA benchmarks, GPU crate (WGSL, optional, CPU fallback), allocation budgets.
