# Contributing Guide

## Development Setup

- Pinned toolchain `1.97.0` via `rust-toolchain.toml` (minimal profile, components rustfmt, clippy)
- **Pre-push gate (mandatory): `scripts/ci-local.sh`** — runs the exact CI command list locally (fmt, strict clippy `--all-targets --all-features`, tests, doctests, f64, single-thread, release, sandbox scenes, fuzz-smoke, bench compile, C example, interactive build, aarch64 check, `cargo deny check`). Added 2026-07-19 after a red CI run escaped local gating; do not push with a failing step.
- `cargo fmt --all`, `cargo clippy --workspace --all-targets --all-features -- -D warnings`, `cargo test --workspace --all-features`, `cargo test --doc --workspace`, `cargo run -p auralite-sandbox --release`, `cargo bench -p auralite-core`, `cargo check --target aarch64-unknown-linux-gnu --all-features`
- C FFI: `gcc crates/auralite-ffi/c_example/main.c target/release/libauralite_ffi.a -lpthread -ldl -lm -o /tmp/c_verify && /tmp/c_verify`
- Fuzz: `cargo run -p auralite-fuzz --release`
- Interactive sandbox: `cargo run -p auralite-sandbox --release --features interactive -- --interactive` (requires DISPLAY)
- Dependency audit: pinned `cargo install cargo-deny --version 0.20.2 --locked`, then `cargo deny check` (root covers the full workspace feature graph incl. sandbox `interactive` via `[graph] all-features = true`; `--all-features` is not a valid `check` flag on pinned cargo-deny).

## Code Style

- `#![forbid(unsafe_code)]` on core crates (`auralite-core` enforces 0 unsafe), minimal unsafe in `auralite-math` (SIMD intrinsics with `// SAFETY:`) and `auralite-ffi` (C boundaries, null checks, generation-safe).
- Determinism Tier A required+tested (ST=MT bitwise, 10k×3 replay, snapshot round-trip).
- No mocks/hard-coded demos as implementation (H1 rule).
- No blanket lint suppressions as documentation — write real docs, Safety sections for FFI, narrow `#[allow]` with justification.
- Tolerances justified, never raised-to-pass; bounded iterative algorithms with diagnostics.
- Zero-dependency core (math/core/geometry/collision/dynamics stay dep-free, ADR-16); new dep only with justification + default-features off + license check + audit in same commit (ADR-17).

## Pull Requests

- Preserve history, commit per coherent phase.
- Update continuity docs: `progress.md` resume pointer, `requirements-traceability.md`, `known-limitations.md`, `risk-register.md`, `test-report.md`, `benchmark-report.md`, `unsafe-inventory.md`, `dependencies.md`, `platform-support.md`, ADRs, changelog.
- No evidence-free claims (platform cells, CI results, sanitizer safety, benchmark adjectives).
- CI must be green (`fmt`, `clippy -D warnings`, tests, doctests, f64, single-thread, release, sandbox scenes, bench compile, C example, aarch64 check, deny audit, fuzz-smoke).

## Phases

Current phase plan per continuation brief: R0 Verify, R1 Sandbox Truth (H1), R2 API/Document Integrity (H3-H7/H10-H12), R3 QA (H8/H9), R4 Final Report.

After each phase: full gates green (or exact failure reason recorded), continuity docs updated, resume pointer rewritten.

