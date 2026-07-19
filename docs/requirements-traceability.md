# Requirements Traceability Matrix (R4 / CI-Green Baseline — 2026-07-19)

Maps high-level requirements (R1–R10) and Section-5 technical requirements (S5.1–S5.16) to **executed, CI-observed** evidence. Baseline: green CI run `29682753719` (all 5 jobs success on ubuntu/windows/macOS + aarch64 parity + audit) + local battery `scripts/ci-local.sh` exit 0; **151 tests** (142 unit/integration + 9 doctests), 0 failed; fuzz 1350 iterations 0 panics.

## 1. High-Level Requirements (`R1–R10`)

| Req ID | Description | Status | Verification & Evidence |
|---|---|---|---|
| R1.1 | 2D Rigid Body Dynamics | ✅ DONE | `World2`/`Body2` damping+sleeping, zero-realloc budget `steady_state_step_allocation_budget_2d` (`dynamics/src/lib.rs:2942`), stacking scenes; CI all OSes. |
| R1.2 | 3D Rigid Body Dynamics | ✅ DONE | `World3`/`Body3`, full state hash (`G1`), `ContactConstraint3`, 10k suite `long_run_determinism_suite_10k_steps_3d` (`:2840`). |
| R2.1 | 2D Discrete Collision | ✅ DONE | GJK/EPA/SAT, true `Capsule2`/`Edge2`, 2-point `Manifold2` clipping (`G9`); 30 collision tests incl. robustness suite. |
| R2.2 | 3D Discrete Collision | ✅ DONE | GJK/EPA/SAT, `ContactConstraint3`, `Manifold3` (+known-limitation: single-point persistence, low-sev). |
| R2.3 | Continuous Collision (`CCD`) | ✅ DONE | `sphere_toi_analytic`, `velocity_sweep_never_tunnels_plane`; CCD scene 3/16. |
| R3.1 | 2D Joints | ✅ DONE | Distance/Weld/Revolute/Prismatic, break thresholds (`G3`), ragdoll scene 2/16. |
| R3.2 | 3D Joints | ✅ DONE | `BallSocket`, `Weld`, `Distance`, `Slider`, `Hinge`, `Spring`, **ConeTwist** (R2/H5: `joint3_cone_twist_limits_never_exceeded`, `joint3_cone_twist_stability_long_run`), break/motor tests. |
| R4.1 | Soft Bodies (`PBD`/`XPBD`) | ✅ DONE | Soft cube + volume conservation, binary round-trip serialization (`G12`), scenes 6–7/16. |
| R4.2 | Cloth Simulation | ✅ DONE | XPBD strips/hanging, spatial-hash self-collision (`auralite-softbody` tests). |
| R5.1 | Particle Physics | ✅ DONE | Emitters/recycling/force fields, `ParticleStorage` round-trip serialization (`G12`), scene 8/16. |
| R5.2 | PBF Fluid Simulation | ✅ DONE | `SpatialHash` neighbors, exact `volume()`, two-way buoyancy (`D6`, `buoyancy_floating_box_equilibrium`), scenes 9–10/16. |
| R6.1 | Ray-cast Vehicles | ✅ DONE | `Vehicle3` + `ray_cast_ignoring` true normals (`G6`), scene 12/16. |
| R6.2 | Character Controllers | ✅ DONE | `Character2/3` slope-aware grounding (`G6`), scenes 13–14/16. |
| R7.1 | Determinism (`Tier A` bitwise) | ✅ DONE (scope stated) | ST=MT `test_multithreaded_determinism` (`tests/integration_tests.rs:93`; job-count chunking = core-count independent), 10k×3 (`:2746/:2840`), rollback bitwise (`:2525`), snapshot round-trip (`serialize:1639/1679`), lockstep `lockstep_replay_hash_equals` (`lockstep.rs:63`). Tier-B cross-platform: not bitwise, **measured + documented** (`known-limitations.md`). |
| R7.2 | Multithreading Architecture | ✅ DONE | `ThreadPoolScheduler` 0 unsafe (`forbid(unsafe_code)` core), wired into `World2/3::step` (`G8`); ST=MT proof; external C scheduler (`H7`, `ffi_scheduler_callback_invoked` `ffi:869`). |
| R7.3 | SIMD Acceleration | ✅ DONE | SSE2 (x86-64) + NEON (aarch64) arch-gated with scalar fallback; differential `simd_fallback_*` tests; exercised for real on macOS ARM64 CI. |
| R8.1 | GPU Acceleration | ✅ DONE (per ADR-13 resolved outcome) | ADR-13 resolved to **CPU-reference backend** (`auralite-gpu`, 2 tests); hardware wgpu documented as roadmap, deps policy recorded (ADR-16). |
| R9.1 | Versioned Serialization | ✅ DONE | AURA v2 envelopes, 64 MiB quota, checksums, bitwise round-trips; hostile-input fuzz driver (500 mutation iters → Err, never panic). |
| R9.2 | C FFI (`auralite-ffi`) | ✅ DONE | 2D/3D world/body/impulse/batch APIs, log/debug-draw/scheduler callbacks (`H7`), generation-safe handles, header drift test, compiled C example green locally + CI (ubuntu/macOS). |
| R10.1 | Visual Interactive Sandbox | ✅ DONE | Real eframe app, every control engine-driven (see DoD row 5 in `final-report.md`): scene browser 16, time controls, debug toggles, inspection, editable runtime, profiling, real determinism controls incl. **implemented record/replay** (snapshot + hash trace + verified replay, bounded); headless 16/16 + watermarked recorded-replay viewer retained as scene tests. |

---

## 2. Detailed Section-5 Technical Requirements (`S5.1–S5.16`)

| Spec ID | Requirement Section | Status | Verification & Evidence |
|---|---|---|---|
| S5.1 | **2D & 3D Rigid Body Core** | ✅ DONE | Integration, damping, sleep, pool reuse (151-test sweep, CI all OSes). |
| S5.2 | **Collision & Manifolds** | ✅ DONE | `DynamicTree` broad-phase + GJK/EPA/SAT + clipping (`G9`). |
| S5.3 | **Continuous Collision (`CCD`)** | ✅ DONE | Analytic TOI + sweeps, never-tunnel tests. |
| S5.4 | **Constraints & Joints** | ✅ DONE | 2D/3D solvers + ConeTwist, break/motor convergence (`G3`). |
| S5.5 | **Soft Body & Cloth (`XPBD`)** | ✅ DONE | Distance/bend constraints, self-collision, serialization round-trip. |
| S5.6 | **Particles & PBF Fluids** | ✅ DONE | Density/lambda, buoyancy coupling (`D6`), serialization round-trip. |
| S5.7 | **Vehicles & Controllers** | ✅ DONE | Ray-cast wheels, self-filter, slope limits (`G6`). |
| S5.8 | **Deterministic Multithreading** | ✅ DONE | Job-count-chunked deterministic scheduler; Tier-A ST=MT bitwise; chunking core-count-independent (2-core local + 3/4-core CI agree). |
| S5.9 | **SIMD Vector Math** | ✅ DONE | SSE2+NEON+scalar fallback, f32/f64 parity (16 tests), aarch64 cross-parity CI job. |
| S5.10 | **GPU Path / CPU Reference** | ✅ DONE (ADR-13 outcome) | `CpuBackend` verified (2 tests); ADR-13 documents the resolved stance. |
| S5.11 | **Versioned Serialization** | ✅ DONE | Envelope + checksum + full bitwise rollback replays (`G12`). |
| S5.12 | **C FFI & Interoperability** | ✅ DONE | Handles, error API, callbacks incl. scheduler (`H7`), canonical header test, C example. |
| S5.13 | **Visual Interactive Sandbox** | ✅ DONE | Real eframe interactive app (all tools engine-driven; record/replay implemented 2026-07-19) + headless 16/16 + watermarked recorded-replay viewer. |
| S5.14 | **Quality Gates & Formatting** | ✅ DONE | fmt PASS; strict clippy `--all-targets --all-features -D warnings` PASS **with zero blanket suppressions** (4 removed 2026-07-19; 70 hidden lints fixed); doctests 9 covering dynamics/math/serialize/particles/vehicles; all gates also CI-green (run 29682753719). |
| S5.15 | **Benchmarks & Performance** | ✅ DONE | `soa_vs_aos` 5-run medians (SoA 21.05 ms, 1.02×; density 49 ns, 1.20×) + env capture + smoke/rigorous labeling (`docs/benchmark-report.md`); `cargo bench --workspace --no-run` CI gate. |
| S5.16 | **Zero-Dependency Core Audit** | ✅ DONE | `cargo tree` zero third-party for all core crates; sandbox-only eframe tree fully audited: `cargo deny check` (pinned 0.20.2) exit 0 locally + CI audit job green; license justifications + regenerated THIRD_PARTY_NOTICES (322-pkg lock). |

---

**Traceability rule reaffirmed 2026-07-19:** statuses may only cite executed evidence (command+output) or observed CI runs (ID/URL). CI history — including red runs 29583407674 and 29682146269 — is disclosed in `docs/platform-support.md` and `CHANGELOG.md`; nothing is verified-by-claim.
