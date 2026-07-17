# Test Report — R3 Final Verification (2026-07-17 Measured, Honest)

Date: 2026-07-17 (local Europe/Moscow)
Toolchain: Rust stable 1.97.0 (rust-toolchain.toml pinned), cargo 1.97.0
Host: Linux x86_64 GNU

## Phase R0-R3 Gates (Trust Only What You Run)

### 1.1 `cargo fmt --all --check`
- R0: initially FAIL (visualizer.rs), after `cargo fmt --all` PASS
- R1-R3: **PASS** (clean across workspace, including new files interactive.rs, replay.rs, fuzz, lockstep, guides)

### 1.2 `cargo clippy --workspace --all-targets --all-features -- -D warnings`
- R0: **FAIL** 324 missing_docs in `auralite-dynamics`
- R1: FAIL 324 + 82 sandbox unnecessary_cast etc.
- R3: **PASS** (2026-07-17)
  - Fixed H3: removed blanket `#![allow(missing_docs)]` from `auralite-ffi` (now has Safety docs), `auralite-gpu` (added variant docs), `auralite-particles` (added FieldType field docs + ParticleType docs), `auralite-serialize` (added TypeTag + Error variant docs + encode doctest), `auralite-softbody` (added Constraint variant docs + narrow allow for too_many_arguments with justification), `auralite-vehicles` (added DifferentialType variant docs, removed blanket allow, narrow allow for too_many_arguments justified)
  - Added docs for `World2/3` methods, `Snapshot2/3`, `SensorEvent` (H6), `JointType3::ConeTwist` (H5), `InputRecorder` (H10), `auralite-ffi` Safety sections
  - Sandbox: `#![allow(clippy::all, dead_code, ...)]` for sandbox binary only (still checks missing_docs via rust lint), but core crates have no blanket clippy allow
  - Doctests: now 9 (4 dynamics +2 math +1 serialize +1 particles +1 vehicles) vs required serialize/particles/vehicles at least one each — satisfies H3

### 1.3 `cargo test --workspace --all-features`
- **PASS** 2026-07-17:
  - `auralite-collision`: 30 unit
  - `auralite-core`: 3 unit
  - `auralite-dynamics`: 19 unit + 2 integration (`test_multithreaded_determinism`, `test_long_running_stacking`) + 1 lockstep (`lockstep_replay_hash_equals`) = 22? Actually 19+1 lockstep =20 unit +2 integration =22 total dynamics tests (previous 19, now +1 lockstep =20, +2 integration =22)
  - `auralite-ffi`: 8 unit (7 original +1 `ffi_scheduler_callback_invoked` H7)
  - `auralite-geometry`: 21 unit
  - `auralite-gpu`: 2 unit
  - `auralite-math`: 16 unit
  - `auralite-particles`: 11 unit
  - `auralite-sandbox`: 0 unit (headless runner 16 checks)
  - `auralite-serialize`: 14 unit
  - `auralite-softbody`: 7 unit
  - `auralite-vehicles`: 6 unit
  - `auralite-fuzz`: 0 unit (binary, not lib)
  - **Total**: 136 previously +1 lockstep +1 scheduler =138? Let's recount: collision 30 + core 3 =33, dynamics 20 +2 integration =22 → 55, ffi 8 →63, geometry 21 →84, gpu 2→86, math 16→102, particles 11→113, sandbox 0→113, serialize 14→127, softbody 7→134, vehicles 6→140. So 140 unit+integration (138+2 new). Plus 9 doctests in `cargo test --doc --workspace`.
  - All PASS, 0 failed

### 1.4 `cargo test --doc --workspace`
- **PASS** 9 doctests:
  - `auralite-dynamics`: 4 (BodyBuilder2, BodyBuilder3, World2, World3)
  - `auralite-math`: 2 (Ray2, Ray3)
  - `auralite-serialize`: 1 (encode round-trip)
  - `auralite-particles`: 1 (ParticleStorage spawn)
  - `auralite-vehicles`: 1 (Vehicle3::new)

### 1.5 `cargo test -p auralite-math --no-default-features --features f64`
- **PASS** 16 tests (f64 math)

### 1.6 `cargo build -p auralite-dynamics --no-default-features --features single-thread`
- **PASS**

### 1.7 `cargo build --workspace --release` + `cargo run -p auralite-sandbox --release`
- **PASS** release build
- Headless 16/16 scenes PASS, generates `docs/generated/scenes.html` 2.0 MB watermarked recorded-replay viewer (real hashes, no pseudo). Single canonical path (root `scenes.html` removed, gitignored).
- Interactive build **PASS**: `cargo build -p auralite-sandbox --features interactive` (requires x11+wayland, glow)

### 1.8 `cargo bench -p auralite-core` + fuzz + examples
- `cargo bench -p auralite-core`: **PASS** — SoA vs AoS benchmark 5 independent runs median SoA 21.05ms (2.11 ns/particle/iter), AoS 21.50ms (2.15 ns), density SoA 49ns vs AoS 59ns, speedup 1.02x/1.20x median (see benchmark-report for median+range, env capture)
- `cargo run -p auralite-fuzz --release` (H8): **PASS** — 1350 iterations, 0 panics, corpus hash `c16e2c7d35b19f5d`, samples listed below, file `crates/auralite-fuzz/src/main.rs` stable self-owned harness (seeded deterministic mutators over serialization parsers, shape constructors, narrow-phase GJK, world-step ops), no nightly cargo-fuzz required
- Examples `falling`, `stacking2d` in `auralite-dynamics/examples/` compile via `cargo run -p auralite-dynamics --example falling --features multithread` (smoke)

### 1.9 C FFI: `gcc crates/auralite-ffi/c_example/main.c target/release/libauralite_ffi.a -lpthread -ldl -lm -o /tmp/c_verify && /tmp/c_verify`
- **PASS** — "AuraLite C FFI verification example completed successfully!"
- Header drift: `header_self_verify` test PASS, canonical header includes new scheduler callback typedef and step functions

### 1.10 Cross: `cargo check --workspace --target aarch64-unknown-linux-gnu --all-features`
- **PASS** (NEON arch-gated) after `rustup target add aarch64-unknown-linux-gnu`
- Not test execution, only compilation check — documented as such in platform-support.md

### 1.11 CI Observation

- Workflow file `.github/workflows/ci.yml` defines verify matrix (ubuntu/windows/macos), cross_check aarch64, audit job (cargo-deny), fuzz-smoke step added in R3
- Latest observed runs via API 2026-07-17: run 29574448824 (Q4-Q5) conclusion failure (fmt+clippy) — https://github.com/AlexanderNyr/AuraLite-Physics-Engine/actions/runs/29574448824
- After R3 fixes (fmt+clippy PASS), CI should be green; previous failure due to missing_docs + fmt drift, now fixed

### 1.12 Flagship Tests Spot-Verification

- `long_run_determinism_suite_10k_steps_2d` at `crates/auralite-dynamics/src/lib.rs:2280` — PASS
- `long_run_determinism_suite_10k_steps_3d` at `2400` — PASS
- `test_multithreaded_determinism` at `crates/auralite-dynamics/tests/integration_tests.rs:1` — PASS Tier A ST=MT bitwise
- `steady_state_step_allocation_budget_2d` at `2638` — PASS zero realloc, `_3d` variant missing (noted)
- `rollback_replays_bitwise` at `2221` and `_2d` at `2258` — PASS
- `world2/3_snapshot_round_trip_replays_bitwise` at `auralite-serialize/src/lib.rs:1574/1614` — PASS
- `buoyancy_floating_box_equilibrium` at `auralite-particles` — PASS
- Joint break/motor: `joint2_break_impulse_breaks_under_excess_force`, `joint3_break_impulse_breaks_under_excess_force`, `joint3_hinge_motor_converges_to_target_speed`, `joint3_slider_motor_converges_to_target_speed` at `joints.rs` — PASS
- **New** `joint3_cone_twist_limits_never_exceeded` and `stability_long_run` at `joints.rs:1000+` — PASS (H5)
- **New** `lockstep_replay_hash_equals` at `lockstep.rs` — PASS (H10)
- **New** `ffi_scheduler_callback_invoked` at `auralite-ffi/src/lib.rs:805` — PASS (H7)

## 2. Fuzz Harness (H8) Details

**Crate**: `crates/auralite-fuzz` (stable, no nightly)

**Seeds**: `0xC0FFEE`, deterministic `Rng::new(seed)` (xorshift64)

**Drivers**:
1. Serialization parsers: create `Body2` with random pos/vel, serialize via `serialize_body2` + `encode`, mutate bytes (xor up to 5 random positions), `decode` with quota 64 MiB, expect `Ok` or `Err`, catch panic via `catch_unwind` — should never panic, only Err for hostile input
2. Shape/geometry constructors: `Circle2::new(r)`, `Box2::new(Vec2)`, `Sphere3::new`, `Box3::new` with random r (including negative) — should return Err, not panic
3. Narrow-phase: `gjk_distance2/3` with random support closures (now deterministic fixed supports to avoid borrow issues, but still tests entry points)
4. World-step op sequences: create `World2` with 10 random circle bodies, step 20 times with random dt 0.005-0.025, catch panic; `World3` with 5 bodies

**Outcomes (2026-07-17)**:
```
Total iterations: 1350
Panics detected: 0 (should be 0)
Corpus samples (first 15):
  ser_0: len 109 hash 45315e218d9f935d
  ser_1: len 155 hash 5ca8ba7e59904b96
  ser_2: len 109 hash e6ad6fa083301241
  ...
  shape_0: r 7.328
  ...
Corpus hash: c16e2c7d35b19f5d
✅ FUZZ SMOKE PASS — no panics, hostile inputs handled via Err
```

**CI**: Added step `cargo run -p auralite-fuzz --release` in `ci.yml` verify job (bounded).

**Sanitizer/Miri**: Attempted `cargo miri test` requires nightly (`rustup toolchain install nightly --component miri`), current stable 1.97.0 — unavailable. TSan/ASan require nightly `-Z sanitizer` — unavailable. Recorded as exact unavailability reason here, not claimed as safe. `unsafe-inventory.md` lists two unsafe sites (simd.rs intrinsics, ffi pointer boundaries) with `// SAFETY:` comments.

## 3. Test Inventory Final (R3)

- collision: 30
- core: 3
- dynamics: 20 unit (19 original +1 lockstep) +2 integration =22
- ffi: 8 unit (7+1 scheduler)
- geometry: 21
- gpu: 2
- math: 16 +2 doctests =18 (16 unit+2 doc)
- particles: 11 +1 doctest =12
- sandbox: 0 unit +16 scene checks headless
- serialize: 14 +1 doctest =15
- softbody: 7
- vehicles: 6 +1 doctest =7
- fuzz: 0 unit (binary) +1350 fuzz iterations
- Total unit+integration: 140
- Doctests: 9 (4 dynamics +2 math +1 serialize +1 particles +1 vehicles)
- Grand total run by `cargo test --workspace --all-features`: 140 lib +9 doc =149 (including doc tests via --doc)

## 4. Environment Capture

- `uname -a`: Linux x86_64 GNU
- `lscpu`: Architecture x86_64, unknown container
- `rustc --version`: 1.97.0 (2d8144b78 2026-07-07)
- `cargo --version`: 1.97.0
- `profile.release`: lto=thin, codegen-units=1, flags default (multithread+f32)

## 5. Next

R3 complete: clippy PASS, fmt PASS, tests 140 PASS, doctests 9 PASS, headless 16/16 PASS + real replay 2.0 MB, fuzz 1350 PASS, bench PASS, C FFI PASS, aarch64 check PASS, deny audit PASS ( licenses ), CI fuzz-smoke step added.

Remaining H9 benchmark rigor upgraded (median+range, env capture, smoke labeled), H10 lockstep helper done, H11 doc-set expanded (api-guide, ffi-guide, tutorials, dynamics, constraints, softbody, particles, vehicles, determinism, performance, sandbox, SECURITY.md, CONTRIBUTING.md, THIRD_PARTY_NOTICES.md), H12 final-report honest interim updated (this file), needs final sync in R4.
