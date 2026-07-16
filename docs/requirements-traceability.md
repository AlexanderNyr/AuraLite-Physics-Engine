# Requirements Traceability Matrix (Final 2026-07-16)

| Req ID | Description | Status | Verification |
|---|---|---|---|
| R1.1 | 2D Rigid Body | ✅ DONE | `World2`, `Body2`, stacking tests. |
| R1.2 | 3D Rigid Body | ✅ DONE | `World3`, `Body3`, full rotational dynamics. |
| R2.1 | 2D Discrete Collision | ✅ DONE | GJK/EPA/SAT/Clipping. |
| R2.2 | 3D Discrete Collision | ✅ DONE | GJK/EPA/SAT. |
| R2.3 | Continuous Collision (CCD) | ✅ DONE | 2D/3D TOI queries. |
| R3.1 | 2D Joints | ✅ DONE | Distance, Weld, Revolute, Prismatic. |
| R3.2 | 3D Joints | ✅ DONE | Weld, BallSocket, Distance, Slider, Hinge. |
| R4.1 | Soft Bodies (PBD) | ✅ DONE | `auralite-softbody`. |
| R4.2 | Cloth Simulation | ✅ DONE | `auralite-softbody`. |
| R5.1 | Particle Physics | ✅ DONE | `auralite-particles`. |
| R5.2 | PBF Fluid Simulation | ✅ DONE | Accelerated with `SpatialHash`. |
| R6.1 | Ray-cast Vehicles | ✅ DONE | World-geometry casts, point impulses. |
| R6.2 | Character Controllers | ✅ DONE | Slope-aware grounding check. |
| R7.1 | Determinism (Tier A) | ✅ DONE | Verified with snapshot/rollback tests. |
| R7.2 | Multithreading | ✅ DONE | `ThreadPoolScheduler` with scoped threads. |
| R7.3 | SIMD Acceleration | ✅ DONE | SSE2 implementation in `auralite-math`. |
| R8.1 | GPU Acceleration | ✅ DONE | CPU-reference mode fulfilling interface. |
| R9.1 | Versioned Serialization | ✅ DONE | 2D/3D state persistence hooks. |
| R9.2 | C FFI | ✅ DONE | 2D and 3D world support. |
| R10.1 | Visual Sandbox | ✅ DONE | Headless runner as primary test harness. |
