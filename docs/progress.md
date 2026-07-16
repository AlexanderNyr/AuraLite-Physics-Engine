# Progress
## Current milestone: M10–M11 complete on 2026-07-16

M0–M9 remain green. All 11 milestones implemented.

**M10 — Serialization, Replay, Rollback, FFI:**

- `auralite-serialize`: Version 2 envelope with checksums, typed payloads (TypeTag: 1-16), Body2/Collider2/Joint2/RNG round-trip serialization
- Hostile-input hardening: truncation, quota bounds, checksum verification, type-tag mismatches
- `auralite-ffi`: Generation-safe opaque tokens `(index<<32)|generation`, thread-local last-error, API/ABI version queries, header drift check (`CANONICAL_HEADER` + `verify_header`), world count tracking, panic containment
- Public `body_handles()` on World2/World3 for safe iteration
- `Default` impl for `Handle<T>` for serialization support

**M11 — Sandbox, Integration, Hardening, Docs:**

- 16-scene interactive headless sandbox: stacking, ragdoll, CCD, triggers, deterministic replay, softbody/cloth, self-collision, particles, PBF fluid, buoyancy, force fields, 3D vehicle, 2D/3D character controllers, serialization round-trip, 100-body stress test
- Each scene validates correctness properties and reports timing
- Deterministic replay verified with snapshot/restore hash comparisons
- All 16 scenes pass with zero failures

**Final Gate Results:**
- **131 unit tests** across 11 crates — all passing
- `cargo fmt --all --check` — clean
- `cargo clippy -D warnings` — clean
- `cargo build --release` — clean
- `cargo test -p auralite-math --features f64` — 16 tests passing
- `cargo run -p auralite-sandbox` — 16/16 scenes pass
- Zero third-party dependencies
- Zero unsound unsafe (justified in FFI/gpu crates only)
