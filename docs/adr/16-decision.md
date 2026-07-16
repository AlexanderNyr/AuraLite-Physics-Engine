# ADR 16: dependency policy
**Status:** accepted; validated repository-wide.

## Context
The engine's core must remain dependency-free (zero third-party Rust crates) for auditability, licensing, and platform portability. Non-core crates (sandbox, ffi, optional GPU) may have justified dependencies.

## Decision
- **Core crates** (math, core, geometry, collision, dynamics): Zero third-party dependencies. Only `std` and `core`. This is enforced at crate level by the absence of `[dependencies]` in `Cargo.toml`.
- **Serialization, FFI**: Zero third-party dependencies (own binary format, own C-ABI wrapper).
- **Sandbox** (future): May depend on windowing, graphics, and UI crates. Each dependency must:
  - Have a written justification in `docs/dependencies.md`.
  - Have default features disabled.
  - Be license-compatible with Apache-2.0.
  - Be covered by CI audit (cargo-deny).
- **GPU crate** (future): May depend on wgpu for cross-platform GPU access. Same rules apply.
- **Test/bench support**: Small crates like `rand` for test fixtures are acceptable only behind `cfg(test)` or `[dev-dependencies]`.
- **No large frameworks**: Full game engines, render pipelines, or UI frameworks as dependencies are not acceptable.

## Alternatives
- Maximal dependency reuse: faster but increases supply-chain risk and license burden.
- Full DIY: more work but complete control and auditability.

## Consequences
- Core engine is auditable with zero supply-chain risk for the physics pipeline.
- Sandbox/GPU deps are purely additive and clearly bounded.
- CI must include `cargo-audit` and `cargo-deny` for dependency scanning (when dependencies exist).

## Validation
- `cargo tree` on every core crate confirms zero dependencies.
- `cargo deny` check passes for sandbox/GPU deps (when added).
- License compatibility documented.
