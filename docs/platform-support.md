# Platform Support & Verification Matrix (R4 — CI Green Observed, 2026-07-19)

Every cell is **CI-verified** (observed green run, cited by ID/URL), **Verified-locally** (command+output on this host), or **Guidance-only** (exact blocker). No cell claims execution that was not observed.

## CI Observation History (all API-observed, honest — including the red ones)

- **2026-07-19 — RUN `29682753719` — ✅ SUCCESS (all 5 jobs)** — head `a2edbb1` — https://github.com/AlexanderNyr/AuraLite-Physics-Engine/actions/runs/29682753719
  - Verify (ubuntu-latest): success, 170 s, 17 steps
  - Verify (windows-latest): success, 240 s, 17 steps
  - Verify (macos-latest, ARM64): success, 147 s, 17 steps
  - Cross-Target Parity Check (aarch64): success, 43 s
  - Dependency Audit (cargo-deny 0.20.2 pinned): success, 133 s
- 2026-07-19 — run `29682146269` — failure — head `0388337`: ubuntu/windows/audit/cross success; **macOS failed** in `cargo test --workspace --all-features` — `test_long_running_stacking` panicked `vel len: 1.0774778` (emergent residual speed exceeded heuristic threshold). Diagnosis + measured fix: see CHANGELOG (rc2) and `known-limitations.md` "Tier-B trajectory divergence".
- 2026-07-17 — run `29583407674` — failure — head `cc738e2`: fuzz clippy errors (`--all-targets` caught what local gates missed) + `deny.toml` unparseable under unpinned cargo-deny; Windows job auto-cancelled by fast-fail. Root causes + fixes in CHANGELOG (rc2).
- 2026-07-17 — run `29574448824` — failure — pre-R3 baseline (fmt + missing_docs), superseded.

## Matrix

| Platform & Architecture | Compilation | Test Execution | Classification & Evidence |
|---|---|---|---|
| **Linux x86-64 GNU** (`x86_64-unknown-linux-gnu`) | ✅ CI-verified + Verified-locally | ✅ CI-verified + Verified-locally | CI: run 29682753719 Verify (ubuntu-latest) success (fmt, clippy `-D warnings --all-targets --all-features`, full test suite, doctests, f64, single-thread, release, headless sandbox 16 scenes, fuzz smoke, bench compile, C FFI gcc example, interactive build). Local (this host, 2026-07-19, `scripts/ci-local.sh` exit 0): identical battery; 142 unit/integration + 9 doctests = **151**; C example prints "completed successfully". |
| **Windows x86-64 MSVC** (`x86_64-pc-windows-msvc`) | ✅ CI-verified | ✅ CI-verified | CI: run 29682753719 Verify (windows-latest) **success, 240 s, all 17 steps** — first observed green Windows run in repo history (previous runs failed or were cancelled: 29583407674 cancelled by fast-fail, even older runs red). C-FFI gcc step is Linux/macOS-only by design; all other gates executed. |
| **macOS ARM64** (`aarch64-apple-darwin`) | ✅ CI-verified | ✅ CI-verified | CI: run 29682753719 Verify (macos-latest) **success, 147 s, all 17 steps**, incl. C FFI example and full test suite on Apple Silicon (NEON path exercised). Preceded by honest failure 29682146269 (stacking test threshold) — fixed and re-verified. |
| **Linux ARM64 GNU** (`aarch64-unknown-linux-gnu`) | ✅ CI-verified (cross-check) | ⚠️ NOT executed (compile only) | CI: run 29682753719 Cross-Target Parity `cargo check --workspace --target aarch64-unknown-linux-gnu --all-features` success (NEON arch-gates compile). No qemu/hardware available for test execution — honest compile-only cell, same locally. |
| **Android ARM64** (`aarch64-linux-android`) | ⚠️ Guidance-only | ⚠️ Guidance-only | `scripts/build-android.sh` requires `ANDROID_NDK_HOME` (absent here); no CI job; script existence ≠ compilation. Blocker: Android NDK/SDK not installed; no device observed. |
| **iOS ARM64** (`aarch64-apple-ios`) | ⚠️ Guidance-only | ⚠️ Guidance-only | `scripts/build-ios.sh` requires macOS + Xcode (host is Linux); no CI job. Blocker: requires Apple toolchain; never executed. |

## Summary

- Linux x86-64, Windows x86-64, macOS ARM64: **CI-verified** (compilation + tests + doctests + all quality gates) by observed green run `29682753719`; Linux additionally Verified-locally with the identical command list (`scripts/ci-local.sh`).
- Linux ARM64: compilation CI-verified (`cargo check`); test execution not performed anywhere — honest guidance cell.
- Android/iOS: Guidance-only — configured scripts, never executed; no inflation.

Core (math/core/geometry/collision/dynamics/softbody/particles/vehicles/serialize/ffi) stays zero-dependency, so portability beyond the verified matrix is *plausible but unproven* — any such claim must come with an observed run.
