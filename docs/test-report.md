# Test Report (Audited Baseline 2026-07-16)

## Baseline Gates
- `cargo fmt --all --check`: ✅ PASS
- `cargo clippy --workspace --all-targets --all-features -- -D warnings`: ✅ PASS
- `cargo test --workspace --all-features`: ✅ PASS (131 tests)
- `cargo test -p auralite-math --no-default-features --features f64`: ✅ PASS (16 tests)
- `cargo build --workspace --release`: ✅ PASS
- `cargo run -p auralite-sandbox`: ✅ PASS (16/16 scenes)
- `cargo run -p auralite-dynamics --example falling`: ✅ PASS
- `cargo run -p auralite-dynamics --example stacking2d`: ✅ PASS

## Audit Observations
The following defects were verified during the audit on 2026-07-16:

| ID | Description | Status |
|---|---|---|
| D1 | 3D joints are non-functional; World3 missing joints | Verified |
| D2 | Joint identity bugs (2D); incorrect removal logic | Verified |
| D3 | Contact feature IDs are unstable (step-dependent) | Verified |
| D4 | Solver architecture: integration order and ground handling | Verified |
| D5 | PBF neighbor search is brute-force O(n²) | Verified |
| D6 | No fluid↔rigid coupling / buoyancy unused in step | Verified |
| D7 | Vehicles: ray-cast only y=0 plane; CoM impulses only | Verified |
| D8 | Character controllers: kinematic movers only, no slope/step | Verified |
| D9 | GPU crate is a non-functional shell | Verified |
| D10 | SIMD is a scalar facade | Verified |
| D11 | Multithreading is absent | Verified |
| D12 | Serialization: 2D-only, many TypeTags unimplemented | Verified |
| D13 | FFI: World2-only, minimal API, no World3/Body/etc. | Verified |
| D14 | Sandbox: Headless only, no visual/interactive UI | Verified |
| D15 | Docs drifted: README and reports claim M3/M4 status | Verified |
| D16 | ADRs incomplete or template status | ✅ FIXED |
| D17 | Benchmarks unwired / incomplete | Verified |
| D18 | Missing product guides and tutorials | Verified |
| D19 | QA pyramid gaps: no fuzz, Miri, or long-run tests | Verified |
| D20 | Ergonomics sweep needed | Verified |

## Test Inventory (131 total)
- `auralite-collision`: 30 tests
- `auralite-core`: 4 tests
- `auralite-dynamics`: 22 tests
- `auralite-ffi`: 5 tests
- `auralite-geometry`: 21 tests
- `auralite-gpu`: 2 tests
- `auralite-math`: 16 tests
- `auralite-particles`: 10 tests
- `auralite-serialize`: 8 tests
- `auralite-softbody`: 7 tests
- `auralite-vehicles`: 6 tests
