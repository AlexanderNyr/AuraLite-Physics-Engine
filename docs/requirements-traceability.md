# Requirements Traceability Matrix (Measured Audit Baseline — Phase Q3 Completed, 2026-07-16)

This matrix maps both the high-level functional requirements (`R1–R10`) and the detailed Section-5 technical requirements (`S5.1–S5.16`) against verified execution reality upon completion of `Phase Q3`.

## 1. High-Level Requirements (`R1–R10`)

| Req ID | Description | Status | Verification & Audit Notes |
|---|---|---|---|
| R1.1 | 2D Rigid Body Dynamics | ✅ DONE | `World2`, `Body2`, damping (`G5`), sleeping (`G4`), steady-state zero allocations, and stacking tests verified (`auralite-dynamics`). |
| R1.2 | 3D Rigid Body Dynamics | ✅ DONE | `World3`, `Body3`, full dynamic state hash (`G1`), damping (`G5`), steady-state zero allocations, and 3D contact constraints (`ContactConstraint3`) verified. |
| R2.1 | 2D Discrete Collision | ✅ DONE | GJK, EPA, SAT, true `Capsule2`/`Edge2` support, and 2-point manifold clipping (`Manifold2`) verified (`G9`). |
| R2.2 | 3D Discrete Collision | ✅ DONE | GJK, EPA, SAT, `ContactConstraint3` solver, and multi-point manifold structures (`Manifold3`) verified. |
| R2.3 | Continuous Collision (`CCD`) | ✅ DONE | 2D/3D TOI queries and velocity sweeps verified (`auralite-collision`). |
| R3.1 | 2D Joints | ✅ DONE | Distance, Weld, Revolute, Prismatic constraints, and breaking thresholds (`break_impulse`) verified (`G3`). |
| R3.2 | 3D Joints | ✅ DONE | `Joint3` solver (`BallSocket`, `Weld`, `Distance`, `Slider`, `Hinge`, `Spring`), breaking thresholds, and motor convergence (`target_speed`) verified (`G3`). |
| R4.1 | Soft Bodies (`PBD` / `XPBD`) | ✅ DONE | Soft cube and volume conservation verified (`auralite-softbody`). |
| R4.2 | Cloth Simulation | ✅ DONE | XPBD cloth strips, hanging, and spatial hash self-collision verified (`7 passed`). |
| R5.1 | Particle Physics | ✅ DONE | Emitter capacity, recycling, and force fields verified (`auralite-particles`). |
| R5.2 | PBF Fluid Simulation | ✅ DONE | `SpatialHash` O(n) neighbor search, exact volume computation (`volume()`), and two-way buoyancy coupling (`D6`) verified (`11 passed`). |
| R6.1 | Ray-cast Vehicles | ✅ DONE | `Vehicle2`/`Vehicle3` verified (`6 passed`) against exact shape ray-cast queries (`G6`) with true outward normals and self-filtering (`ray_cast_ignoring`). |
| R6.2 | Character Controllers | ✅ DONE | `Character2`/`Character3` verified (`6 passed`) against exact shape ray-cast queries (`G6`), filtering self and checking slope limits. |
| R7.1 | Determinism (`Tier A` bitwise) | ✅ DONE (Core) | Verified via `World2`/`World3` full dynamic state hashing (`G1`), bitwise rollback (`G2`), 10,000-step ×3 multi-run continuous/snapshot replay suite (`G10`), and Tier-A ST=MT proof (`test_multithreaded_determinism`). |
| R7.2 | Multithreading Architecture | ✅ DONE | `ThreadPoolScheduler` (`0 unsafe`, `#![forbid(unsafe_code)]` on `auralite-core`) wired directly into `World2::step` and `World3::step` (`G8`) with disjoint `chunks_mut` and fixed-order deterministic merging. |
| R7.3 | SIMD Acceleration | ✅ DONE | Architecture-gated `x86_64` SSE2 (`_mm_set_ps`/`_mm_set_pd`) and `aarch64` NEON (`vld1q_f32`/`vmulq_f32`) verified across `f32`/`f64` (`16 passed`). |
| R8.1 | GPU Acceleration | ⚠️ INCOMPLETE | `auralite-gpu` is currently a CPU-reference fallback (`D9`). Documented ADR-13 CPU-reference stance / verification scheduled in Q4/Q5. |
| R9.1 | Versioned Serialization | ⚠️ INCOMPLETE | 2D rigid bodies/colliders verified (`8 passed`). `serialize_world3` decode (`G12`) and joint/softbody/snapshot persistence scheduled across Q4. |
| R9.2 | C FFI (`auralite-ffi`) | ⚠️ INCOMPLETE | World create/step/destroy verified (`5 passed`). Body queries, impulses, callbacks, C CI compilation, and drift checks (`G13`) scheduled across Q4. |
| R10.1 | Visual Interactive Sandbox | ❌ INCOMPLETE | Currently headless runner (`cargo run -p auralite-sandbox`) generating static `scenes.html` (`G11`). Windowed interactive UI scheduled in Q5. |

---

## 2. Detailed Section-5 Technical Requirements (`S5.1–S5.16`)

| Spec ID | Requirement Section | Status | Audit Verification & Planned Closure Phase |
|---|---|---|---|
| S5.1 | **2D & 3D Rigid Body Core** | ✅ DONE | Position/velocity integration, stable damping (`G5`), support-gated sleeping (`G4`), and pool slot handling verified (`131 total tests`). |
| S5.2 | **Collision & Manifolds** | ✅ DONE | Broad-phase `DynamicTree` and narrow-phase `GJK`/`EPA`/`SAT`, true analytical capsule/edge support, and multi-point contact clipping (`Manifold2`/`Manifold3`) verified (`G9`). |
| S5.3 | **Continuous Collision (`CCD`)** | ✅ DONE | Analytic sphere/box TOI queries and velocity sweeps verified (`auralite-collision`). |
| S5.4 | **Constraints & Joints** | ✅ DONE | 2D/3D joint solvers (`BallSocket`, `Weld`, `Distance`, `Revolute`, `Prismatic`, `Slider`, `Hinge`, `Spring`), breakable impulses, and motor target convergence verified (`G3`). |
| S5.5 | **Soft Body & Cloth (`XPBD`)** | ✅ DONE | Distance constraints, bending, and spatial hash self-collision verified (`auralite-softbody`). |
| S5.6 | **Particles & PBF Fluids** | ✅ DONE | Density/lambda calculation, emitters, and exact Archimedes buoyancy/coupling (`D6`) verified (`buoyancy_floating_box_equilibrium`). |
| S5.7 | **Vehicles & Controllers** | ✅ DONE | Structure, ray-cast wheels, self-filtering (`ray_cast_ignoring`), and slope-aware character grounding verified (`G6`) (`auralite-vehicles`). |
| S5.8 | **Deterministic Multithreading** | ✅ DONE | `Scheduler` trait and `ThreadPoolScheduler` (`0 unsafe`) wired into `World2`/`World3` step (`G8`). Tier-A ST=MT bitwise identity proven (`test_multithreaded_determinism`). |
| S5.9 | **SIMD Vector Math** | ✅ DONE | Architecture-gated `x86_64` SSE2 and `aarch64` NEON intrinsics with scalar fallback and `f32`/`f64` parity verified (`16 passed` + cross-target build green). |
| S5.10 | **GPU Path / CPU Reference** | ⚠️ INCOMPLETE | `CpuBackend` trait implementation verified (`2 passed`). Definitive ADR-13 resolution required across Q4/Q5. |
| S5.11 | **Versioned Serialization** | ⚠️ INCOMPLETE | `Envelope` and checksum verified (`8 passed`). Full 3D/joint/softbody/snapshot round-trips (`G12`) scheduled across Q4. |
| S5.12 | **C FFI & Interoperability** | ⚠️ INCOMPLETE | Generation-safe opaque handles and `auralite_last_error` verified (`5 passed`). Full C headers/callbacks/CI (`G13`) across Q4. |
| S5.13 | **Visual Interactive Sandbox** | ❌ INCOMPLETE | 16/16 scene assertions pass headless. Windowed real-time visualization (`G11`) scheduled in Q5. |
| S5.14 | **Quality Gates & Formatting** | ⚠️ IN PROGRESS | `cargo fmt`, strict `clippy --all-features -D warnings`, and release builds clean across Phase Q3. Doctest gaps (`G14`) addressed across Q4–Q5. |
| S5.15 | **Benchmarks & Performance** | ⚠️ IN PROGRESS | `soa_vs_aos` benchmark compiles and runs (~21ms SoA vs ~22.6ms AoS). Full subsystem benchmark report (`D17`) across Q5. |
| S5.16 | **Zero-Dependency Core Audit** | ✅ DONE | `Cargo.toml` workspace verification confirms 0 third-party dependencies across all core physics crates (`crates/*`). |
