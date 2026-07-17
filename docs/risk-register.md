# Risk Register — R2 Truth Pass (2026-07-17)

## Closed M-Era Risks (from M1/M2) — Evidence Links

| Risk | Likelihood | Impact | Status / Evidence |
|---|---|---|---|
| Scope exceeds one execution session | Certain | Critical | **CLOSED** — Continuity via `docs/progress.md` resume pointer, phased commits R0 (f88d1ac), R1 (8f68d41) etc. Honest incremental delivery, never claimed completion while H-items open (final-report now interim). |
| Robust convex collision degeneracy | High | High | **CLOSED** — Bounded GJK (32 iter max), EPA degenerate fallback, SAT, `clip_contacts2` multi-point clipping, `epa_agrees_with_sat_for_boxes`, `robustness_deep_penetration`, `robustness_degenerate_near_zero`, `robustness_km/mm_scale`, `robustness_plate_stacking_sat` tests (auralite-collision 30 tests). Evidence: `crates/auralite-collision/src/narrow.rs` |
| Floating-point cross-platform drift | High | High | **CLOSED** — Tiered determinism: Tier A bitwise proven via `long_run_determinism_suite_10k_steps_2d/_3d` (10k steps x3 replay), `rollback_replays_bitwise`, `test_multithreaded_determinism` ST=MT, full state_hash hashing pos/rot/vel/sleep/kind. No Tier C claim. Evidence: `crates/auralite-dynamics/src/lib.rs:2280`, `tests/integration_tests.rs` |
| FFI memory safety | Medium | Critical | **PARTIALLY CLOSED** — Isolated unsafe in `auralite-ffi` (pointer writes, null checks, generation-safe tokens), C header drift test `header_self_verify`, compiled C example `crates/auralite-ffi/c_example/main.c` passes. Remaining: allocator story documented in ADR-15 (global allocator embedder-wide), scheduler callback planned H7 (not yet implemented), missing_safety_doc still present (H3). |
| Mobile/GPU SDK unavailable | Certain here | Medium | **CLOSED as Guidance-only** — Documented in `docs/platform-support.md` as Guidance-only: Android NDK absent, iOS requires macOS/Xcode. Scripts `build-android.sh`/ `build-ios.sh` exist but never executed. No overclaim. |
| Performance architecture premature | Medium | High | **CLOSED** — Reference paths maintained, SoA vs AoS benchmark measured `cargo bench -p auralite-core` (SoA 20.99ms vs AoS 21.45ms, 1.02x/1.20x), allocation budget test `steady_state_step_allocation_budget_2d` verifies zero realloc. Full benchmark rigor (H9) pending methodology upgrade. |

## Current Risks (R0-R2 Open)

| Risk | Likelihood | Impact | Owner | Mitigation / Status |
|---|---|---|---|---|
| Single-platform verification (Linux x86-64 only) | Certain (current env) | High | QA/CI | Only Linux x86-64 verified locally (138 lib tests + 6 doctests). ARM64 cross-check via `cargo check --target aarch64-unknown-linux-gnu`. Windows/macOS CI-configured but latest observed run 29574448824 conclusion failure (fmt+clippy). No successful CI run observed. Mitigation: honest matrix in `docs/platform-support.md`, note blocker (needs Windows/macOS runners). |
| Sandbox dependency introduction (eframe) supply-chain | Medium | High | Systems / Sandbox | Introduced in R1: `eframe 0.32.1` with default-features off (glow+default_fonts+x11+wayland), `deny.toml` allows MIT/Apache-2.0/BSD/ISC. CI job `audit` runs `cargo deny check`. Core remains zero-dep. Headless CI skips interactive feature. Mitigation done: ADR-17 + dependencies.md + deny.toml + CI audit. |
| 3D manifold multi-point persistence depth | Medium | Medium | Collision / Solver | 2D has multi-point clipping (`clip_contacts2`), 3D uses single-point per contact (`generate_clip_points_3d` returns single point). Depth persistence not fully proven. Mitigation: note in known-limitations, add test hole in R3. |
| Extreme mass ratios (km/mm scale, high inertia) | Medium | Medium | Numerics | Tests `robustness_km/mm_scale` finite but solver may jitter at extreme ratios. Tolerances justified, not raised-to-pass. Documented in `known-limitations.md`. |
| Missing docs / blanket lint suppressions (H3) | Certain | Medium | SDK / Docs | `cargo clippy --workspace --all-targets --all-features -- -D warnings` FAIL (324 missing_docs in dynamics + blanket allow in ffi, gpu, particles, serialize, softbody, vehicles). Doctests: only 6 (dynamics+math) vs requirement serialize/particles/vehicles at least one each. Mitigation: R2 started fixing joints.rs docs (ConeTwist added with docs), gpu allow removed, but many remain. DoD row 7 not green. |
| Cone-twist joint limits enforcement (H5) | High (was missing) | Medium | Constraints | **FIXED in R2**: Added `JointType3::ConeTwist { axis_local, swing_limit, twist_limit }` with swing/twist decomposition, enforcement via corrective angular impulses, tests `joint3_cone_twist_limits_never_exceeded` and `stability_long_run` pass. Evidence: `crates/auralite-dynamics/src/joints.rs:460+` |
| Sensor stay event (H6) | High (was missing) | Low | Dynamics | **FIXED in R2**: Added `is_stay` field to `SensorEvent`, emits stay for ongoing pairs in deterministic sorted order, `is_begin`/`is_end`/`is_stay` methods. Existing begin/end preserved. Triggers scene still checks begin, now also could check stay. |
| FFI callback incomplete (H7) | Medium | Medium | FFI / SDK | Present: log, debug-draw-line. Missing: allocator (embedder-wide global allocator, documented in ADR-15 as guidance), scheduler (external callback). Planned: `auralite_set_scheduler_callback` + `ExternalCScheduler` implementing `Scheduler` trait. Not yet implemented in R2, gap remains. |
| Fuzzing / sanitizer absent (H8) | Certain | High | QA | No `fuzz/` targets, no Miri/sanitizer runs recorded. DoD row 8 asserts "Miri/Sanitizer/race safe" without evidence — overclaim. Need stable self-owned fuzz harness (seeded deterministic mutators over serialization, shape, narrow-phase, world-step ops) + CI fuzz-smoke + Miri/TSan where allowed or exact unavailability reason. |
| Benchmark rigor (H9) | High | Medium | QA / Perf | Current benchmark-report single-shot sandbox wall-times (18ms stacking, etc.) labeled as smoke but presented as benchmarks. Need repeated independent runs median+range, env capture (CPU/OS/toolchain/flags), keep smoke labeled. Performance adjectives in README/docs not mapped to measurement. |
| Lockstep API (H10) | Medium | Medium | Dynamics / SDK | Existing seed+snapshot+deterministic-step constitutes lockstep but no small input-recording/replay helper (record (step,input) streams, re-apply deterministically, hash-compare) with example/test. Planned for R2/R3. |
| Doc-set incomplete (H11) | Certain | High | Tech Writer | Missing: Rust API guide, C FFI guide, 2D/3D tutorials, dynamics guide, constraints guide, soft-body/cloth guide, particles/fluids guide, vehicles guide, determinism/replay/rollback guide, performance/tuning guide, sandbox guide (needs write for H1 real sandbox), SECURITY.md, CONTRIBUTING.md, third-party notices (trivial zero-dep but file must exist), expand six thin guides. |
| Final report alignment (H12) | High | Critical | Release | DoD rows 3,5,7,8 previously carried false evidence. R0/R1 fixed 3 (platform) and 5 (sandbox). Rows 7 and 8 still overclaimed. Need per-row links (file:line/test-name/command-output). Status must be honest interim until all 16 evidenced. |

## Ownership

- Architect: overall DoD truth, ADR-16/17
- Simulation/Numerics/Collision/Solver: joints cone-twist, sensor stay, manifold depth
- SDK/FFI: safety docs, scheduler/allocator callbacks, header drift
- QA/Fuzz/Benchmark: fuzz harness, Miri/sanitizer, benchmark rigor
- CI/Release: platform matrix, deny audit, changelog, final-report evidence table
- Tech Writer: risk-register, guides, SECURITY, CONTRIBUTING, third-party notices
