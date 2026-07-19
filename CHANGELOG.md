# Changelog

## [1.0.0-rc2] - 2026-07-19 (CI repair — red run 29583407674)

### Honest failure record
- **2026-07-17 CI run 29583407674 FAILED** despite docs claiming all gates green. Root causes (all reproduced locally and fixed in this release):
  1. Five strict-clippy errors in `auralite-fuzz/src/main.rs` under `--all-targets --all-features` (2× unused variable, 2× dead functions, 1× manual `is_multiple_of`) — local gates had been run before the fuzz crate existed / without the full flag set.
  2. `deny.toml` failed to parse under unpinned latest cargo-deny (duplicate `[licenses]` table) and carried schema keys removed by cargo-deny PR #611 (`copyleft`, `unlicensed`, `allow-osi-fsf-free`), plus invalid `cargo deny check --all-features` CLI usage in CI.
  3. The Verify matrix fast-cancelled the Windows job, so Windows/macOS test evidence never existed until a green run was observed (now policy: `fail-fast: false`).

### Fixed
- `test_long_running_stacking` made platform-robust (2026-07-19 run 29682146269 was green on ubuntu/Windows/audit/aarch64-parity but failed on macOS ARM64: emergent residual speed 1.077478 exceeded the heuristic `v < 1.0` threshold). Measured data (stack probe): x86-64 release 1.1123444, KE ≈ 3.0 J, |x| ≤ 9.54, y ∈ [0, 1.37] — the threshold sat inside the jitter band by luck on x86 dev. Assertion re-anchored to the physical stability envelope the test actually means (finite, no tunneling y > −2, no lateral explosion |x| < 25, residual speed < 3.0 with measured 1.11 worst case — explosions/tunneling still fail by orders of magnitude). Engine code untouched; Tier-A determinism (same-build ST=MT, replay bitwise) unaffected.
- `auralite-fuzz`: removed dead `next_f32/next_f64`, `_`-bound unused loop vars, `is_multiple_of(3)`; corpus hash unchanged (`c16e2c7d35b19f5d`) — fixes are behavior-neutral.
- `deny.toml` rewritten for pinned **cargo-deny 0.20.2** schema: single `[licenses] version = 2` table, scope-valued `unmaintained`/`unsound`, `[graph] all-features = true` replacing the invalid CLI flags.
- Sandbox blanket lint suppressions (`#![allow(clippy::all, dead_code, ...)]` ×4 files) removed; 70 hidden lints fixed genuinely (52 unnecessary casts, let-chain collapses, dead code, `large_enum_variant` via boxed world variants).
- Interactive sandbox: replaced the `recorded_frames` "placeholder for future record feature" with real engine-driven record/replay — snapshot-at-record-start + per-step `state_hash()` trace + verified re-step replay with divergence evidence (bounded `MAX_RECORD_FRAMES`); `show_inspection`/`show_settings` are real top-bar toggles; dead `ActiveWorld::Particles` variant and H1-era `generate_interactive_sandbox_app()` stub removed; `SvgVisualizer` now exercised headlessly (`docs/generated/snapshot-{2d,3d}.svg`).
- CI: cargo-deny pinned (`--version 0.20.2 --locked`); single canonical `cargo deny check`; `fail-fast: false` kept on the Verify matrix with recorded rationale; added `scripts/ci-local.sh` (exact CI command list) referenced as mandatory pre-push gate in CONTRIBUTING.md.

### Dependency audit dispositions (see docs/dependencies.md)
- Licenses: added `BSL-1.0` (clipboard-win 5.4.1, error-code 3.3.2 — Windows-only clipboard chain), `OFL-1.1` + `Ubuntu-font-1.0` (epaint_default_fonts 0.32.3 font assets); removed never-encountered allowances (MPL-2.0, CC0-1.0, Unicode-DFS-2016, CDLA-Permissive-2.0).
- Advisories: RUSTSEC-2026-0194/0195 (quick-xml 0.39.4) ignored with written justification + review-by 2027-01-19 — build-time-only wayland-scanner, no untrusted input; upgrade blocked by pinned winit 0.30 stack.
- `THIRD_PARTY_NOTICES.md` regenerated from the current 322-package lock (179 package/license rows over CI targets).

### CI verification (observed)
- Run `29682146269` (2026-07-19, lint/config repair): ubuntu/windows/aarch64-parity/audit **success**; macOS ARM64 failed only on the stacking-test threshold above → fixed.
- Run **`29682753719` (2026-07-19) — ALL 5 JOBS SUCCESS**: Verify ubuntu (170 s) / windows (240 s) / macOS ARM64 (147 s / all 17 steps each), Cross-Target Parity aarch64 (43 s), Dependency Audit pinned cargo-deny 0.20.2 (133 s). https://github.com/AlexanderNyr/AuraLite-Physics-Engine/actions/runs/29682753719 — first green CI run in repo history and first honestly-verified Windows/macOS test execution.
- Local battery `scripts/ci-local.sh` exit 0 on the same content; 151 tests (142 + 9 doctests).

### Changed (2026-07-19, post-verification)
- Toolchain pin bumped **1.97.0 → 1.97.1** (patch release, `8bab26f4f 2026-07-14`): `rust-toolchain.toml`, CI workflows (×3 jobs), CONTRIBUTING/README/scripts/docs references. Full local battery re-executed on 1.97.1 before push — strict clippy `-D warnings --all-targets --all-features` PASS (new pinned clippy accepted), 151 tests PASS, fuzz corpus hash unchanged `c16e2c7d35b19f5d`. Benchmark numbers in `docs/benchmark-report.md` remain honest 1.97.0 measurements (noted there).

## [1.0.0-rc1.1] - 2026-07-17 (R0–R3 truth & QA phases; CI run 29583407674 later found red)

### Added
- R1/H1: real interactive desktop sandbox (eframe 0.32.x, default-features off, ADR-17); watermarked engine-recorded replay viewer `docs/generated/scenes.html` replacing the fabricated-hash mock.
- R2: `JointType3::ConeTwist` with enforcement + tests (H5); deterministic sensor-stay `is_stay` (H6); C scheduler callback FFI + `auralite_world*_step_with_external_scheduler` (H7, ADR-15).
- R3: `auralite-fuzz` stable seeded harness — 1350 iterations, 0 panics, corpus hash `c16e2c7d35b19f5d` (H8); `lockstep.rs` `InputRecorder` with replay hash test (H10); guide set under `docs/guides/` + SECURITY.md/CONTRIBUTING.md (H11); benchmark methodology hardened to medians over 5 runs (H9).
- Doctest coverage for dynamics/math/serialize/particles/vehicles; real rustdoc for all public items (H3 blanket `missing_docs` allow removed).

### Changed
- Full public-API documentation written; narrow `#[allow]`s carry justifications (H3).
- Platform/test docs rewritten to measured values only (R0).

## [1.0.0-rc1] - 2026-07-16
### Added
- Real 3D Joint Solver for `World3` (Weld, BallSocket, Distance, Slider, Hinge).
- `ThreadPoolScheduler` using `std::thread::scope` for multi-threading.
- SSE2 SIMD implementation in `auralite-math`.
- `SpatialHash` acceleration for PBF Fluids and Soft Body self-collision.
- World ray-casting for 3D Vehicles and Character Controllers.
- SVG visualizer and HTML reporting for the sandbox.
- C FFI extensions for World3 and body manipulation.

### Fixed
- Fixed solver pipeline order: Integrate Velocities -> Solve -> Integrate Positions.
- Fixed 2D joint identity and removal bugs.
- Stabilized contact feature IDs for warm-starting.
- Reconciled all ADRs and progress documentation with reality.

### Changed
- Refactored `auralite-gpu` to provide a functional CPU-reference mode.
- Expanded test suite to 133 unit tests.
- Hardened sandbox with 16 validated scenes.
