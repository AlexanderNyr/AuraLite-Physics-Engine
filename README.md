# AuraLite Physics Engine

AuraLite is a deterministic, dimension-safe (`2D` + `3D`), Apache-2.0 Rust physics engine under active development. It features generic GJK/EPA/SAT collision detection, sequential-impulse rigid body dynamics, joint constraints (`BallSocket`, `Weld`, `Revolute`, `Distance`, `Spring`, `Hinge`, `Slider`), continuous collision detection (`CCD`), spatial hash `PBF` fluids, `XPBD` soft bodies and cloth, ray-cast vehicles, slope-aware character controllers, SSE2 SIMD acceleration, and multi-threaded scheduling.

## Current Audit & Completion Status (2026-07-16 — Phase Q0 Truth Refresh)

An exhaustive execution audit conducted on 2026-07-16 (`Phase 0 — Q0 Truth Refresh`) established that prior claims of `PRODUCTION COMPLETE (1.0.0-rc1)` were unsupported by execution reality. While substantial real progress exists in the codebase (`Joint3` solver, spatial hash neighbor search, SSE2 intrinsics, FFI and serialization hooks), multiple critical regressions (`G1–G15`) and incomplete subsystems (`D1–D20`) were verified.

We have restored the **original 16-item Definition of Done (DoD)** verbatim in `docs/final-report.md`. The project is operating under a rigorous continuation plan across phases `Q1–Q5` (`Verify → Repair → Complete`) to systematically resolve all defects and verify determinism, performance, and cross-platform correctness by execution.

## Quick Start & Quality Gates

The engine requires Rust stable (`1.97.0`). All core crates operate with zero third-party dependencies.

```sh
# Verify code formatting and strict clippy lints
cargo fmt --all --check
cargo clippy --workspace --all-targets --all-features -- -D warnings

# Execute full workspace test suite (116 total tests across 11 crates)
cargo test --workspace --all-features

# Verify double-precision (f64) SIMD configuration
cargo test -p auralite-math --no-default-features --features f64

# Verify explicit single-thread dynamics configuration
cargo build -p auralite-dynamics --no-default-features --features single-thread

# Run the 16 automated sandbox scene checks
cargo run -p auralite-sandbox --release
```

## Documentation

- **Final Report & Restored DoD**: [docs/final-report.md](docs/final-report.md)
- **Measured Test & Defect Report**: [docs/test-report.md](docs/test-report.md)
- **Requirements Traceability Matrix**: [docs/requirements-traceability.md](docs/requirements-traceability.md)
- **Phase Progress & Next Tasks**: [docs/progress.md](docs/progress.md)
- **Architecture & ADRs**: [docs/architecture.md](docs/architecture.md) · [ADR Index](docs/adr/)
- **Known Limitations & Risk Register**: [docs/known-limitations.md](docs/known-limitations.md) · [docs/risk-register.md](docs/risk-register.md)
