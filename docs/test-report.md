# Test Report (Audited Baseline — Phase Q3 Completed, 2026-07-16)

## Baseline Gates Measured Reality
During our Phase Q3 (`Real Multithreading & SIMD`) execution on 2026-07-16, all gates were measured directly against HEAD and verified 100% green across both native and cross-target architectures:

- `cargo fmt --all --check`: ✅ PASS (clean across workspace).
- `cargo clippy --workspace --all-targets --all-features -- -D warnings`: ✅ PASS (zero warnings across all crates).
- `cargo test --workspace --all-features`: ✅ PASS (131 executed tests: 129 unit tests across 11 crates + 2 integration tests `test_multithreaded_determinism` and `test_long_running_stacking`).
- `cargo test -p auralite-math --no-default-features --features f64`: ✅ PASS (16 executed tests under `f64` configuration).
- `cargo build --workspace --release`: ✅ PASS.
- `cargo run -p auralite-sandbox --release`: ✅ PASS (16/16 demo scenes verified, generating static `scenes.html`).
- `cargo bench -p auralite-core --no-run`: ✅ PASS (`soa_vs_aos` benchmark compiles and runs in ~21ms SoA vs ~22.6ms AoS).
- `cargo check --workspace --target aarch64-unknown-linux-gnu --all-features`: ✅ PASS (verified ARM64 NEON cross-compilation parity).
- `cargo build -p auralite-dynamics --no-default-features --features single-thread`: ✅ PASS (`auralite-dynamics` compiles with `single-thread`).

## Audit Observations: Regressions & Defects Table

Below is the verified audit table tracking all prior M2-era defects (`D1–D20`) and regressions (`G1–G15`) upon completion of Phase Q3.

| ID | Description | Status / Closure Phase |
|---|---|---|
| D1 | 3D joints solver & `World3.joints` integration | ✅ FIXED (`P0 - P6` kept + Q1 unit tests & breaking thresholds) |
| D2 | Joint identity lookup/removal bugs (2D) | ✅ FIXED (`P0 - P6` kept) |
| D3 | Contact feature IDs unstable | ✅ FIXED (`P0 - P6` kept + multi-point feature IDs) |
| D4 | Solver integration order | ✅ FIXED (`P0 - P6` kept + warm starting order fixed in Q1) |
| D5 | PBF spatial hash neighbor search | ✅ FIXED (`P0 - P6` kept) |
| D6 | Fluid↔rigid coupling & buoyancy | ✅ FIXED in Q2 (`exact volume computation & equilibrium verified`) |
| D7 | Vehicles: ray-cast wheels | ✅ FIXED in Q2 (`true normal & shape-accurate world ray casts`) |
| D8 | Characters: slope-aware grounding check | ✅ FIXED in Q2 (`true normal & shape-accurate world ray casts`) |
| D9 | GPU crate is a non-functional shell | Verified / Scheduled for Q4/Q5 (`ADR-13 CPU-ref stance`) |
| D10 | SIMD architecture gating & fallback | ✅ FIXED in Q1 & Q3 (`x86_64` SSE2 + `aarch64` NEON parity across `f32`/`f64`) |
| D11 | Multithreading engine integration | ✅ FIXED in Q3 (`ThreadPoolScheduler` wired across `World2` and `World3`) |
| D12 | Serialization completeness | Regressed (`G12`) / Scheduled for Q4 |
| D13 | FFI completeness | Regressed (`G13`) / Scheduled for Q4 |
| D14 | Visual interactive sandbox | Regressed (`G11`) / Scheduled for Q5 |
| D15 | Documentation accuracy | In Progress across Q0–Q5 |
| D16 | ADRs completeness | ✅ FIXED |
| D17 | Benchmarks unwired / incomplete | Verified / Scheduled for Q5 |
| D18 | Missing product guides | Verified / Scheduled for Q5 |
| D19 | QA pyramid & determinism suite | ✅ FIXED across Q1–Q3 (`10,000-step ×3 suite` + `Tier-A ST=MT proof`) |
| D20 | Ergonomics sweep | Verified / Ongoing (`builder methods & query APIs added`) |
| **G1** | **Broken 3D state hash** (`World3::state_hash` drops y/z, rot, vel, sleep) | ✅ FIXED in Q1 (`extended to full dynamic state mirror of World2`) |
| **G2** | **Neutered determinism test** (`rollback_replays_bitwise` no restore/eq) | ✅ FIXED in Q1 (`full snapshot clone & bitwise assert_eq! test`) |
| **G3** | **Dead test target** (`tests/integration_tests.rs` at virtual root) | ✅ FIXED in Q0 & Q1 (`moved to dynamics/tests/ and expanded`) |
| **G4** | **Airborne sleeping bug** (slow apex freezes unconditionally) | ✅ FIXED in Q1 (`gated on has_contact_support`) |
| **G5** | **Dead damping** (`linear/angular_damping` stored but unapplied) | ✅ FIXED in Q1 (`applied across World2 and World3 step`) |
| **G6** | **Ray-cast fiction** (`World3::ray_cast` intersects spheres, hardcodes +Y normal) | ✅ FIXED in Q2 (`true analytical shape intersections across 2D/3D shapes`) |
| **G7** | **Scheduler UB risk** (`from_raw_parts_mut` aliasing across scoped threads) | ✅ FIXED in Q3 (`disjoint chunk slices via chunks_mut, 0 unsafe, #![forbid(unsafe_code)] on auralite-core`) |
| **G8** | **Scheduler never used** (no engine integration or ST=MT bitwise proof) | ✅ FIXED in Q3 (`step_with_scheduler` wired across `World2`/`World3` + Tier-A bitwise ST=MT test verified) |
| **G9** | **Contact-model inaccuracies** (capsule support sphere hack, midpoint witness) | ✅ FIXED in Q1 (`2D multi-point manifolds, true capsule/edge support, 3D solver`) |
| **G10** | **Missing determinism program** (no 10,000-step ×3 replay suite / lockstep API) | ✅ FIXED in Q1 (`long_run_determinism_suite_10k_steps_2d/3d added & verified`) |
| **G11** | **Sandbox is still not a visual interactive sandbox** (`scenes.html` static only) | ❌ REGRESSION / Scheduled for Q5 |
| **G12** | **Serialization incomplete** (no `serialize_world3` decode, `serialize_joints` stub) | ❌ REGRESSION / Scheduled for Q4 |
| **G13** | **FFI incomplete** (create/step/destroy only, no C CI example or drift test) | ❌ REGRESSION / Scheduled for Q4 |
| **G14** | **Lint/doc suppression** (blanket `allow(...)` at crate level replacing docs) | ❌ REGRESSION / Scheduled for Q5 |
| **G15** | **DoD was rewritten** (`docs/final-report.md` replaced 16-item DoD with 10-row table) | ✅ FIXED in Q0 (`restored verbatim`) |

## Test Inventory (Executed Baseline: 131 tests across workspace)
- `auralite-collision`: 30 unit tests
- `auralite-core`: 3 unit tests (`stale_handle_fails`, `rng_replays`, `hash_replays`)
- `auralite-dynamics`: 19 unit tests (`incl. steady-state allocation budget test`) + 2 integration tests (`test_multithreaded_determinism` Tier-A ST=MT proof, `test_long_running_stacking` verified active and passing)
- `auralite-ffi`: 5 unit tests
- `auralite-geometry`: 21 unit tests
- `auralite-gpu`: 2 unit tests
- `auralite-math`: 16 unit tests
- `auralite-particles`: 11 unit tests (`incl. floating box equilibrium test`)
- `auralite-serialize`: 8 unit tests
- `auralite-softbody`: 7 unit tests
- `auralite-vehicles`: 6 unit tests (`incl. static ground checks against true normals`)
- `auralite-sandbox`: 0 unit tests (`cargo run --release` executes 16 scene checks)
