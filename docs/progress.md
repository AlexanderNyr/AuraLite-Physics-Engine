# Progress & Phase Completion Log

## Current Phase: Phase Q4 Completed — Phase Q5 Next (`Verify → Repair → Complete`)

### Phase Checklist & Status

- [x] **Phase Q0 — Truth Refresh (Completed 2026-07-16)**
  - Executed Section-1 gates and recorded exact measured outputs (`116 executed tests`).
  - Fixed initial formatting drift, clippy lints (`possible_missing_else`, `collapsible_if`, `unused_imports`), and SSE2 `f64` compilation errors (`simd.rs`).
  - Moved virtual workspace integration test to `crates/auralite-dynamics/tests/integration_tests.rs`.
  - Restored verbatim 16-item Definition of Done in `docs/final-report.md`.

- [x] **Phase Q1 — Determinism & Correctness Core (Completed 2026-07-16)**
  - **G1**: Fixed broken 3D state hash (`World3::state_hash` — extended to full dynamic state mirror of `World2`: positions, rotations, velocities, angular velocities, `sleeping`, and `kind`).
  - **G2**: Restored real bitwise snapshot/rollback equality test (`rollback_replays_bitwise` and `rollback_replays_bitwise_2d` using exact `World2::clone()` / `World3::clone()`).
  - **G4**: Fixed airborne sleeping bug (`World2::step` / `World3::step` — gated `b.sleeping = true` on contact/ground support `has_contact_support`, preventing airborne bodies from freezing at jump apex).
  - **G5**: Implemented linear and angular damping across `World2::step` / `World3::step` (`b.velocity *= (1.0 - damping * dt).max(0.0)`).
  - **G9**: Fixed contact-model inaccuracies (`generic_convex_contact_2d` / `_3d`, 2D multi-point manifolds via `clip_contacts2`, true analytical `Capsule2`/`Edge2` support without bounding sphere or closest-point hacks, warm starting applied directly prior to solver loop, and Coulomb friction `tangent_impulse` solver). Un-ignored and verified `test_long_running_stacking` (10 stacked dynamic boxes settle in stable equilibrium). Added `ContactConstraint3` and `solve_contacts_3d_once` for full 2D/3D contact solver parity.
  - **G3 & Section 3**: Added 4 unit tests verifying joint breaking threshold (`break_impulse` triggering `broken = true` across `Joint2` and `Joint3`) and motor target convergence (`Hinge` and `Slider` driving angular/linear velocity to `target_speed`).
  - **G10**: Built and verified 10,000-step ×3 multi-run continuous vs independent vs snapshot-rollback replay verification suite (`long_run_determinism_suite_10k_steps_2d` and `_3d`).

- [x] **Phase Q2 — World Queries & Gameplay Truth (Completed 2026-07-16)**
  - **G6**: Replaced bounding-sphere/hardcoded `Vec3::Y` approximations in `World3::ray_cast` and `World2::ray_cast` with true analytical shape-level ray intersection queries (`ray_intersection`) across `Sphere3`, `Box3`, `Capsule3`, `ConvexHull3`, `TriangleMesh`, and `Circle2`, `Box2`, `Capsule2`, `ConvexPolygon`, `Edge2`, `Heightfield2`. Added `ray_cast_ignoring` to filter out self-collisions.
  - **Vehicles & Characters**: Re-wired `Vehicle2`, `Vehicle3`, `Character2`, and `Character3` to call `ray_cast_ignoring(..., self.body)` against exact world geometry with true outward normals (`n.dot(Vec3::Y).acos() <= slope_limit`). Updated tests (`6 passed`) with real static ground geometry (`Box2` / `Box3`).
  - **Coupling & Buoyancy (`D6`)**: Added exact `volume()` calculation across all 2D and 3D collider shapes (`Sphere3`, `Box3`, `Capsule3`, `ConvexHull3`, etc.) and wired true displaced volume into `apply_buoyancy_to_world`. Built and verified `buoyancy_floating_box_equilibrium` test proving exact Archimedes fluid-rigid neutral buoyancy equilibrium.

- [x] **Phase Q3 — Real Multithreading & SIMD (Completed 2026-07-16)**
  - **G7**: Rewrote `ThreadPoolScheduler` without aliasing mutable references using exact disjoint `chunks_mut` (`and div_ceil`). Restored `#![forbid(unsafe_code)]` on `auralite-core`, guaranteeing zero unsafe code across the entire core scheduling/pool crate.
  - **G8**: Wired `Scheduler` (`ThreadPoolScheduler` under multithread feature, `SingleThreadScheduler` under single-thread) directly into `World2::step` and `World3::step` via `step_with_scheduler`. Verified Tier-A ST=MT bitwise identity across complex multi-chunk execution vs single-threaded execution (`test_multithreaded_determinism`).
  - **SIMD Parity**: Verified ARM64 (`aarch64`) cross-compilation (`cargo check --target aarch64-unknown-linux-gnu`) and implemented native NEON (`vld1q_f32`, `vmulq_f32`) vector intrinsics inside `auralite-math/src/simd.rs` alongside `x86_64` SSE2.
  - **Allocation-Budget Tests**: Added scratch buffers (`scratch_pairs`, `scratch_handles`, `scratch_constraints`, `scratch_raw_contacts`) across `World2` and `World3` and verified zero vector re-allocations / zero capacity growth across steady-state stepping (`steady_state_step_allocation_budget_2d`).

- [x] **Phase Q4 — Interop & Persistence (Completed 2026-07-16)**
  - **G12**: Implemented full versioned `AURA` binary envelope serialization and deserialization for `World2` (`deserialize_world2`) and `World3` (`deserialize_world3`, resolving `serialize_joints` stub and missing decode). Implemented `serialize_soft_body` / `deserialize_soft_body` and `serialize_particle_storage` / `deserialize_particle_storage`. Built and verified exact bitwise simulation rollback replay tests (`world2_snapshot_round_trip_replays_bitwise` and `world3_snapshot_round_trip_replays_bitwise`) proving exact state hash identity after snapshot restoration.
  - **G13**: Expanded C FFI (`auralite-ffi`) to include `auralite_world2_add_body`, `auralite_world3_add_body`, `auralite_world2_body_query`, `auralite_world3_body_query`, `auralite_world2_body_apply_impulse`, `auralite_world3_body_apply_impulse`, and `auralite_world3_batch_query_positions`. Added callbacks (`auralite_set_log_callback`, `auralite_set_debug_draw_line_callback`). Synchronized `auralite.h` with `CANONICAL_HEADER`. Built and verified compiled C verification binary (`crates/auralite-ffi/c_example/main.c`) linking against `libauralite_ffi.a`.
  - **All Gates Verified Green**: `133 workspace tests passed`, `cargo check`, `cargo check --target aarch64-unknown-linux-gnu`, `cargo fmt --all --check`, `cargo clippy --all-features -- -D warnings`, `cargo test -p auralite-math --features f64` (`16 passed`), `cargo build --release`, `cargo run -p auralite-sandbox --release` (`16/16 scenes verified`), `cargo build -p auralite-dynamics --features single-thread`, `gcc ... main.c ... passed`.

- [ ] **Phase Q5 — Sandbox & Release Hardening (Next / In Progress)**
  - **G11**: Implement visual interactive sandbox (`windowed winit/wgpu-class app or software renderer with immediate-mode UI, time-scale, debug-draw toggles, inspection panels, profiling overlay`).
  - **G14**: Remove all blanket lint suppressions (`missing_docs`, etc.) across crates; write primary API documentation and real doctests (`>0 doctests executed`).
  - **Audits & Matrix**: Document all dependencies (`dependencies.md`, `ADR-16`, `cargo-deny`), complete `docs/benchmark-report.md`, refresh `docs/platform-support.md`, and generate final report evaluating 100% completion against the restored 16-item DoD.

---

### Resume Pointer (Exact Next File / Task / Command)

1. **Target File**: `crates/auralite-sandbox/src/main.rs` and `.github/workflows/ci.yml`.
2. **Next Tasks (Phase Q5)**:
   - Fix **G11** (`Visual interactive sandbox`): currently `auralite-sandbox` runs headless or generates static `scenes.html`. Implement an interactive windowed/graphical sandbox tool or interactive software/wgpu rendering harness with scene browser (`all 16 demo scenes`), time-control toggles (`pause, single-step, 0.1x/1.0x speed`), debug draw overlays (`AABBs, contacts, normals, centers of mass, joints, sleeping state, cloth, fluid`), and inspection panel. Justify any new dependencies (`winit`, `wgpu` or software framebuffers) via ADR-16 + `docs/dependencies.md`.
   - Fix **G14** (`Blanket lint/doc suppressions`): remove `#![allow(missing_docs, ...)]` from `crates/*/src/lib.rs` and write clear public API documentation and real doctests across primary entry points (`World2`, `World3`, `Body2`, `Body3`, `Ray2`, `Ray3`).
   - Expand `.github/workflows/ci.yml` (`CI Expansion`): wire matrix builds (`x86_64`, `aarch64`, `single-thread`, `f64` math), doctests, C FFI compilation and header verification (`gcc crates/auralite-ffi/c_example/main.c target/release/libauralite_ffi.a`), benchmark smoke check, and `cargo-deny` audit checks.
   - Complete `docs/benchmark-report.md`, refresh `docs/platform-support.md`, and present the honest final report against the **verbatim 16-item Definition of Done** (`Section 8`).
3. **Verification Command**:
   ```sh
   cargo test --workspace --all-features
   cargo test --doc --workspace
   cargo run -p auralite-sandbox --release
   ```
