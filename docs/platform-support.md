# Platform Support & Verification Matrix (R0 Truth Pass — 2026-07-17)

This matrix is rewritten to satisfy H2: every cell must be **Verified-locally** (command+output), **CI-configured** (workflow file citation + observed run if available), or **Guidance-only** (exact blocker). No unexecuted platform is claimed as verified.

## CI Observation (2026-07-17 UTC, Europe/Moscow local date 2026-07-17)

- Workflow file: `.github/workflows/ci.yml` (name `CI & Quality Gates`)
- Latest observed runs via GitHub API `https://api.github.com/repos/AlexanderNyr/AuraLite-Physics-Engine/actions/runs?per_page=5`:
  - Run ID `29574448824` (head SHA `9f8fbccd91fea8be88ecaed0071ac899815ff30d`, title `Q4 - Q5`, event `push`, branch `main`, created `2026-07-17T10:43:58Z`) — status `completed`, conclusion `failure`, URL https://github.com/AlexanderNyr/AuraLite-Physics-Engine/actions/runs/29574448824
  - Run ID `29571374468` (head SHA `068425685da0a94d558e26cb805d71af49011278`, title `Q0 - Q3`) — conclusion `failure`
- Interpretation: CI is **configured** for ubuntu/windows/macos matrix plus aarch64 cross-check, but latest observed conclusions are **failure** (formatting + clippy missing_docs). CI does **not** prove Windows/macOS test execution green; it proves configuration exists.
- If logs were fetchable: `logs_url` https://api.github.com/repos/AlexanderNyr/AuraLite-Physics-Engine/actions/runs/29574448824/logs (requires auth). Jobs API: https://api.github.com/repos/AlexanderNyr/AuraLite-Physics-Engine/actions/runs/29574448824/jobs . We record failure as observed evidence.

## Matrix (Honest)

| Platform & Architecture | Compilation Status | Test Execution | Classification & Evidence |
|---|---|---|---|
| **Linux x86-64 GNU** (`x86_64-unknown-linux-gnu`) | ✅ Verified-locally | ✅ Verified-locally | Compilation: `cargo build --workspace --release` EXIT 0 on 2026-07-17 (stable 1.97.0). Tests: `cargo test --workspace --all-features` -> 136 unit + 2 integration = 138 + 6 doctests = 144 passing locally. `cargo test --doc --workspace` 6 passed. `cargo test -p auralite-math --no-default-features --features f64` 16 passed. `cargo bench -p auralite-core` compiles and runs (SoA vs AoS). C FFI: `gcc crates/auralite-ffi/c_example/main.c target/release/libauralite_ffi.a -lpthread -ldl -lm -o /tmp/c_verify && /tmp/c_verify` EXIT 0. Sandbox: `cargo run -p auralite-sandbox --release` 16/16 scenes pass, generating `docs/generated/scenes.html`. SSE2 + multithread scheduler 100% green locally. |
| **Linux ARM64 GNU** (`aarch64-unknown-linux-gnu`) | ✅ Verified-locally (cross-check) | ⚠️ NOT verified locally (compile only) | Compilation: `rustup target add aarch64-unknown-linux-gnu` + `cargo check --workspace --target aarch64-unknown-linux-gnu --all-features` EXIT 0 on 2026-07-17, proving NEON intrinsics arch-gated. Test execution: NOT executed locally (no qemu/aarch64 host). CI: job `cross_check` in `ci.yml` does same `cargo check`, not test execution. Classification: **Compilation verified-locally, test execution Guidance-only**. |
| **Windows x86-64 MSVC** (`x86_64-pc-windows-msvc`) | ⚠️ CI-configured (not verified locally) | ⚠️ CI-configured, latest failure observed | CI config: `strategy.matrix.os: [ubuntu-latest, windows-latest, macos-latest]` in `.github/workflows/ci.yml`. Previous claim "✅ Verified via CI (test execution)" had zero cited run; actual observed latest run 29574448824 conclusion **failure**. Local verification impossible (env is Linux x86-64, not Windows). Classification: **CI-configured, no observed successful run, not verified locally**. Blocker: requires Windows runner with MSVC toolchain; CI failure indicates fmt + clippy missing_docs. |
| **macOS x86-64 / ARM64** (`x86_64-apple-darwin` / `aarch64-apple-darwin`) | ⚠️ CI-configured (not verified locally) | ⚠️ CI-configured, latest failure observed | Same CI matrix as Windows. Latest observed run failure contradicts prior "✅ Verified via CI". NEON/SSE2 native support assumed but not executed. Classification: **CI-configured, no observed successful run**. Blocker: requires macOS runner; local env is Linux. |
| **Android ARM64** (`aarch64-linux-android`) | ⚠️ Guidance-only (script exists, not executed) | ⚠️ Guidance-only | Script `scripts/build-android.sh` exists: requires `ANDROID_NDK_HOME`, does `rustup target add aarch64-linux-android` + `cargo build --release -p auralite-ffi --target aarch64-linux-android`. Local execution not attempted — NDK absent (env var unset). No CI job for Android. Prior claim "✅ YES (scripts/build-*.sh)" treated script existence as compilation success — overclaim per H2. Classification: **Guidance-only**, blocker: NDK not installed in this environment. |
| **iOS ARM64** (`aarch64-apple-ios`) | ⚠️ Guidance-only (script exists, not executed) | ⚠️ Guidance-only | Script `scripts/build-ios.sh` exists: requires macOS (`uname = Darwin`) + Xcode, adds targets `aarch64-apple-ios` and `aarch64-apple-ios-sim`, builds `auralite-ffi`. Local env is Linux, not Darwin, so script would exit 1 with "requires macOS/Xcode". No CI job for iOS. Classification: **Guidance-only**, blocker: requires macOS/Xcode. |

## Summary

- Only Linux x86-64 is **Verified-locally** for both compilation and test execution.
- Linux ARM64 is **Verified-locally for cross-compilation** (`cargo check`) but **not test execution**.
- Windows/macOS are **CI-configured** (workflow exists, matrix includes them) but latest observed CI conclusion is **failure** (https://github.com/AlexanderNyr/AuraLite-Physics-Engine/actions/runs/29574448824). No local verification possible; cannot claim "verified via CI" without log evidence.
- Android/iOS are **Guidance-only**: build scripts exist but were never executed (NDK/Xcode absent); script existence ≠ successful compile.

Zero-dependency core (math/core/geometry/collision/dynamics) remains portable POSIX/Windows/Mobile per cargo tree, but portability is theoretical until executed.

## Commands Executed Locally (R0)

```
cargo fmt --all --check  # initially FAIL (visualizer.rs diff), after fmt PASS
cargo fmt --all
cargo clippy --workspace --all-targets --all-features -- -D warnings  # FAIL: 324 missing_docs in auralite-dynamics (H3)
cargo test --workspace --all-features  # 138 (136 unit +2 integration) + 6 doctests = 144 total, 0 failed
cargo test --doc --workspace  # 6 doctests PASS
cargo test -p auralite-math --no-default-features --features f64  # 16 PASS
cargo build -p auralite-dynamics --no-default-features --features single-thread  # PASS
cargo build --workspace --release  # PASS
cargo run -p auralite-sandbox --release  # 16/16 scenes PASS, generates docs/generated/scenes.html + stale root scenes.html (drift noted H1)
cargo bench -p auralite-core  # compiles, runs SoA vs AoS benchmark (21ms vs 20ms)
gcc crates/auralite-ffi/c_example/main.c target/release/libauralite_ffi.a -lpthread -ldl -lm -o /tmp/c_verify && /tmp/c_verify  # PASS
rustup target add aarch64-unknown-linux-gnu
cargo check --workspace --target aarch64-unknown-linux-gnu --all-features  # PASS
curl -s https://api.github.com/repos/AlexanderNyr/AuraLite-Physics-Engine/actions/runs?per_page=5  # observed failure
```

All outputs captured in test-report.
