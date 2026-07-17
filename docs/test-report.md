# Test Report — R0 Truth Refresh (2026-07-17 Measured)

Date: 2026-07-17 (local Europe/Moscow, UTC same day)
Toolchain: Rust stable 1.97.0 (rust-toolchain.toml pinned), cargo 1.97.0, stable-x86_64-unknown-linux-gnu
Host: Linux x86_64 GNU (local verification only; cross-check aarch64 via cargo check)

## Phase R0 — Section-1 Gates (Trust Only What You Run)

### 1.1 `cargo fmt --all --check`
- Initial HEAD (9f8fbcc Q4-Q5): **FAIL**
  ```
  Diff in /home/user/AuraLite-Physics-Engine/crates/auralite-sandbox/src/visualizer.rs:227
    Vec3 { x: -hx, y: -hy, z: -hz } -> formatted multi-line
  ```
  The diff was due to unformatted long struct literals in visualizer.rs (H1 file).
- After `cargo fmt --all`: **PASS** (clean across workspace, exit 0). Commit fixes formatting.

### 1.2 `cargo clippy --workspace --all-targets --all-features -- -D warnings`
- Result: **FAIL (101)** with 324 `missing documentation` errors in `auralite-dynamics`
  ```
  error: missing documentation for a struct field
   --> crates/auralite-dynamics/src/joints.rs:420:14
    Spring { stiffness: Real, damping: Real },
  ...
  error: missing documentation for a variant Weld, Hinge, Slider...
  error: missing documentation for associated function Joint3::new, set_limits, set_motor, set_break_impulse
  ...
  error: could not compile `auralite-dynamics` (lib) due to 324 previous errors
  ```
- Root cause: workspace.lints.rust.missing_docs = warn promoted to deny via `-D warnings`; primary crates still lack docs (H3). Blanket `allow(missing_docs)` remains in FFI, GPU, particles, serialize, softbody, vehicles but dynamics does NOT have blanket allow, so it fails.
- Evidence: cargo clippy output captured. This gate is **not green** at R0; requires H3 fix in R2.
- Classification: H3 open.

### 1.3 `cargo test --workspace --all-features`
- Result: **PASS** after fmt fix, despite clippy failure (tests don't require docs).
- Measured counts (2026-07-17):
  ```
  auralite-collision: 30 unit
  auralite-core: 3 unit
  auralite-dynamics: 19 unit
  integration_tests: 2 (test_multithreaded_determinism, test_long_running_stacking)
  auralite-ffi: 7 unit
  auralite-geometry: 21 unit
  auralite-gpu: 2 unit
  auralite-math: 16 unit
  auralite-particles: 11 unit
  auralite-sandbox: 0 unit
  auralite-serialize: 14 unit
  auralite-softbody: 7 unit
  auralite-vehicles: 6 unit
  ----
  Unit: 136, Integration: 2, Total lib: 138
  Doc tests in same invocation: 4 (dynamics: BodyBuilder2, BodyBuilder3, World2, World3) + 2 (math: Ray2, Ray3) = 6
  Grand total run by cargo test --workspace --all-features: 144 (138 lib+integration +6 doc)
  All passed, 0 failed, 0 ignored.
  ```
- Previous claim 133 = 131 unit +2 integration is **outdated**; real count higher (138 lib). The report is corrected here.

### 1.4 `cargo test --doc --workspace`
- Result: **PASS** 6 doctests (same 4+2)
- Output: `test crates/auralite-dynamics/src/lib.rs - BodyBuilder2 (line 362) ... ok` etc.

### 1.5 `cargo test -p auralite-math --no-default-features --features f64`
- Result: **PASS** 16 tests (simd fallback + math)
- Matches claimed 16.

### 1.6 `cargo build -p auralite-dynamics --no-default-features --features single-thread`
- Result: **PASS** (dev profile, 0.72s, 324 missing_docs warnings but not denied without -D warnings)
- Note: single-thread config compiles.

### 1.7 `cargo build --workspace --release` + `cargo run -p auralite-sandbox --release`
- `cargo build --workspace --release`: **PASS** (release, lto=thin, 8.22s)
- `cargo run -p auralite-sandbox --release`: **PASS** 16/16 scenes
  ```
  [1/16] Stacking (5 boxes, 60s) ... ✅ hash 4a1332d789cab55f (19.7ms)
  [2/16] Joints (ragdoll 11 bodies) ... ✅ 11 joints, hash 8e4613ecde7d93ec (91.5ms)
  [3/16] CCD (fast sphere) ... ✅ y=0.500 (7.3µs)
  [4/16] Triggers/fields ... ✅ 1 events (70.5µs)
  [5/16] Deterministic replay ... ✅ hash 65ffc1b2d7e8fce0 (42.1µs)
  [6/16] Soft body (cloth hanging) ... ✅ 64 particles, KE=4.428 (10.7ms)
  [7/16] Self-collision (folded cloth) ... ✅ 36 particles, no NaN (3.2ms)
  [8/16] Particles (emitter) ... ✅ 50 emitted, 50 alive (10.5µs)
  [9/16] Fluid (PBF density) ... ✅ 25 fluid particles (43.3µs)
  [10/16] Buoyancy ... ✅ F_buoy = 9810.000 (1.8µs)
  [11/16] Force fields (wind + drag) ... ✅ wind + drag applied (1.2µs)
  [12/16] Vehicle (3D) ... ✅ pos=(0.00,0.00) (103.3µs)
  [13/16] Character controller (2D) ... ✅ x=6.344, grounded=false (125.2µs)
  [14/16] Character controller (3D) ... ✅ pos=(0.09,0.04) (109.4µs)
  [15/16] Serialization round-trip ... ✅ 155.0 bytes (5.1µs)
  [16/16] Stress (100 bodies) ... ✅ 100 bodies, hash cecf27c3499cc080 (1.7s)
  Generating visual report (scenes.html)...
  ```
- Artifact drift: generates BOTH `docs/generated/scenes.html` and `scenes.html` (root). Root `scenes.html` exists in repo since earlier commit, stale artifact. H1 requires single output path fix.
- **H1 truth gap confirmed**: generated `scenes.html` contains mocked JS:
  - `simStep()` only `stepCount +=1; simTime += ...` — no physics
  - "live state hash" computed as `BigInt(sdata.hash) + BigInt(stepCount*1337)` annotated "Deterministic pseudo hash variation for visualization"
  - Snapshot/Rollback only restores counters
  - Scenes render baked static data (`scenesData`)
  - Previous final-report claimed this as "Functioning visual interactive sandbox" — fabricated metric, violates no-mock rule. Needs R1 rebuild.

### 1.8 `cargo bench -p auralite-core` + falling/stacking examples
- `cargo bench -p auralite-core`: **PASS** compiles, runs:
  ```
  === AoS vs SoA Particle Layout Benchmark ===
  Particles: 10000
  Integration (1000 iterations):
    AoS: 21.45ms (2.1 ns/particle/iter)
    SoA: 20.99ms (2.1 ns/particle/iter)
    Ratio: 1.02x
  Density O(n²) (100 particles, 1 run):
    AoS: 60ns
    SoA: 50ns
    Ratio: 1.20x
  ```
  Previous claim 2.1 vs 2.3 ns, 1.07x/1.31x slightly outdated but same order; measured 1.02x/1.20x on this host.
- Examples `falling`, `stacking2d` in `auralite-dynamics/examples/`: exist, compile via `cargo run -p auralite-dynamics --example falling --features multithread` (not executed in R0 to save time, but buildable).

### 1.9 C FFI: `gcc crates/auralite-ffi/c_example/main.c target/release/libauralite_ffi.a -lpthread -ldl -lm -o /tmp/c_verify && /tmp/c_verify`
- Result: **PASS**
  ```
  AuraLite C FFI verification example starting...
  AuraLite C FFI verification example completed successfully!
  ```

### 1.10 Cross: `cargo check --workspace --target aarch64-unknown-linux-gnu --all-features`
- `rustup target add aarch64-unknown-linux-gnu`: installed `rust-std` for target
- `cargo check ... --target aarch64-unknown-linux-gnu --all-features`: **PASS** (1.08s, warnings about missing_docs but check succeeds)
- Note: cross-check is NOT test execution; only validates NEON intrinsics arch gating. Classification per H2.

### 1.11 GitHub CI observability
- API: `curl -s https://api.github.com/repos/AlexanderNyr/AuraLite-Physics-Engine/actions/runs?per_page=5`
- Latest run ID `29574448824` (SHA 9f8fbcc, title Q4-Q5): status completed, conclusion **failure**, html_url https://github.com/AlexanderNyr/AuraLite-Physics-Engine/actions/runs/29574448824
- Second latest `29571374468` (SHA 0684256, Q0-Q3): failure
- Workflow file `.github/workflows/ci.yml` defines verify matrix ubuntu/windows/macos + cross_check aarch64. CI config exists but no successful run observed; our local clippy failure matches CI failure expectation (fmt + missing_docs).
- We **cannot** claim Windows/macOS verified via CI; we claim CI-configured with observed failure.

## 2. Flagship Tests Spot-Verification

Existence and pass locally:

- `long_run_determinism_suite_10k_steps_2d` in `crates/auralite-dynamics/src/lib.rs:2280`: ✅ exists + passes (part of 19)
- `long_run_determinism_suite_10k_steps_3d` line 2400: ✅ exists + passes
- `test_multithreaded_determinism` in `crates/auralite-dynamics/tests/integration_tests.rs`: ✅ exists + passes (Tier-A ST=MT bitwise proof)
- `steady_state_step_allocation_budget_2d` line 2638: ✅ exists + passes; `_3d` variant **does NOT exist** (only 2D). Brief mentions 2d/_3d but code only has 2d — gap noted, will add 3d in R3 if feasible.
- `rollback_replays_bitwise` (3D) line 2221 + `rollback_replays_bitwise_2d`: ✅ both exist + pass
- `world2_snapshot_round_trip_replays_bitwise` + `world3_snapshot_round_trip_replays_bitwise` in `auralite-serialize/src/lib.rs`: ✅ exist + pass
- `buoyancy_floating_box_equilibrium` in `auralite-particles`: ✅ exists + pass
- Joint break/motor: `joint2_break_impulse_breaks_under_excess_force`, `joint3_break_impulse_breaks_under_excess_force`, `joint3_hinge_motor_converges_to_target_speed`, `joint3_slider_motor_converges_to_target_speed` in `joints.rs`: ✅ exist + pass

## 3. Regressions & Defects Closure (R0 Reality)

Same as prior D1–D20/G1–G15 closure per execution, except:

- G11 (sandbox) is **NOT FIXED** — mock confirmed; H1 requires R1.
- G14 (lint/doc) **NOT FIXED** — clippy failure proves blanket allow remains and docs missing; H3 requires R2.
- G10 lockstep API: 10k suites exist but small input-recording helper missing (H10).
- Other gaps H2-H12 documented separately.

## 4. Test Inventory Corrected (2026-07-17)

- collision: 30
- core: 3
- dynamics: 19 + 2 integration =21
- ffi: 7 + C binary
- geometry: 21
- gpu: 2
- math: 16 + 2 doctests =18 (16 unit +2 doc)
- particles: 11
- sandbox: 0 (headless runner 16 checks)
- serialize: 14
- softbody: 7
- vehicles: 6
- Total unit+integration: 138 (136+2)
- Total doctests: 6 (4 dynamics +2 math) when run via `--doc`
- Total run by `cargo test --workspace --all-features`: 144 (138+6)

## 5. Environment Capture

- CPU: unknown CI host, local measurement 1.02x SoA speedup (host dependent)
- OS: Linux x86_64 GNU
- Toolchain: Rust stable 1.97.0, profile release lto=thin codegen-units=1
- Flags: default features (multithread + f32), also verified single-thread and f64 configs

## 6. Next Phase

R0 complete: fmt fixed, platform matrix rewritten truthfully, test-report updated with measured numbers, CI observation recorded. Gates: fmt PASS, tests PASS, f64 PASS, single-thread PASS, release PASS, sandbox headless PASS, bench PASS, C FFI PASS, aarch64 check PASS, clippy FAIL (H3). Commit and proceed to R1 sandbox truth.

## Appendix: Raw Command Log

See platform-support.md for command list and CI URLs.
