# Test report — 2026-07-16
Environment: Linux x86-64, Rust 1.97.0. Commands `cargo test --workspace` and `cargo clippy --workspace --all-targets -- -D warnings` passed. 45 unit tests passed, 0 failed; all crate doctest harnesses passed (0 doctests). Coverage includes finite rejection, rotation/cross/matrix inverse/transform composition properties, ray/plane/segment boundaries, degenerate triangles, analytic box/polygon mass, convex support differential checks, transformed bounds, scaling rejection, stale handles, RNG replay, coincident/touching analytic contacts, deterministic broad-phase pairs, falling/resting 2D, 3D snapshot rollback hash, invalid-step atomicity, parser truncation/quota hostility, and FFI lifecycle errors.

Not run: ASan/LSan/UBSan/TSan (nightly/tooling unavailable), Miri, cargo-fuzz, cargo-audit/cargo-deny (tools not installed; zero external dependencies reduces but does not eliminate toolchain risk), race tests, cross-platform tests. Full mandatory test pyramid is incomplete.


M1 full gate: workspace all-features tests, isolated f64 math tests (11), strict all-feature clippy, fmt, and release build all passed on 2026-07-16. Seeded properties execute 10,000 cases per precision.


M2 full gate (2026-07-16): 45 workspace tests passed. Geometry contributes 20 tests, including 10,000-direction convex support differential coverage, malformed constructors/assets, analytic mass, bounds, ray/closest/containment/scaling, hull mass and mesh parity regressions. All-feature strict clippy and release build passed.
