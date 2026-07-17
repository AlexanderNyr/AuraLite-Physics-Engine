# Dependencies — Audited (R1 Truth Pass, 2026-07-17)

## Core Zero-Dependency Policy (ADR-16)

All core physics crates remain **zero third-party Rust dependencies** (only `core`/`std`):

- `auralite-math`, `auralite-core`, `auralite-geometry`, `auralite-collision`, `auralite-dynamics`, `auralite-softbody`, `auralite-particles`, `auralite-vehicles`, `auralite-serialize`, `auralite-ffi`, `auralite-gpu`

Verification:
```
cargo tree -p auralite-math --no-default-features
cargo tree -p auralite-core
cargo tree -p auralite-dynamics --no-default-features --features single-thread
# All show no third-party crates
```

## Sandbox Dependencies (Non-Core, Justified per ADR-17, H1)

Sandbox is downstream consumer, may have dependencies with justification, default-features off, license audit, cargo-deny.

### Added in R1 (H1 fix — Real Interactive Sandbox)

**Crate:** `eframe = { version = "0.32.1", default-features = false, features = ["glow", "default_fonts", "x11", "wayland"], optional = true }` in `crates/auralite-sandbox/Cargo.toml`
- Feature gating: `interactive = ["dep:eframe"]` — headless CI does not require eframe; `cargo run -p auralite-sandbox --release --features interactive -- --interactive` launches window
- **Justification** (ADR-17): Minimal window+pixels/softbuffer-class stack + immediate-mode UI in one crate. Provides winit (windowing, x11+wayland), glow (OpenGL cross-platform graphics), egui (immediate UI). Reduces glue vs separate winit+softbuffer+egui_winit. Supports custom painter for 2D/3D physics debug draw (AABBs, contacts, velocities, joints, sleep, softbody, particles). Mature, widely used. License clean.
- **Default-features off**: true — we explicitly enable only `glow`, `default_fonts`, `x11`, `wayland` (needed for Linux windowing). No `wgpu` to keep lighter.
- **License**: MIT OR Apache-2.0 (eframe, egui, winit, glow, glutin, etc.). All transitive deps permissive (MIT/Apache-2.0/BSD-3/ISC/Zlib/Unicode). Verified via `cargo deny check` with `deny.toml`.
- **Audit**: `deny.toml` allows MIT, Apache-2.0, BSD-2/3, ISC, Zlib, Unicode-DFS-2016, CC0-1.0, MPL-2.0 (small). Denies copyleft GPL, unlicensed, unknown registry. `cargo deny check licenses` passes for sandbox after R1 (with exceptions for M1 platform when needed).
- **Usage**: Only in `auralite-sandbox` binary, not library; physics never depends on it.

### Transitive Dependency License Sampling (via cargo deny / cargo tree)

Run:
```
cargo tree -p auralite-sandbox --features interactive --depth 2 | head -n 100
cargo deny check licenses --manifest-path crates/auralite-sandbox/Cargo.toml
```

Observed licenses (sample, 2026-07-17):
- `eframe 0.32.3` MIT OR Apache-2.0
- `egui 0.32.3` MIT OR Apache-2.0
- `egui_glow 0.32.3` MIT OR Apache-2.0
- `winit 0.30.12` Apache-2.0
- `glow 0.16.0` MIT OR Apache-2.0
- `glutin 0.32.3` Apache-2.0
- `x11rb-protocol 0.13.2` X11 (MIT-like)
- `wayland-client 0.31.12` MIT
- `smithay-client-toolkit 0.19.2` MIT
- `raw-window-handle 0.6.2` MIT OR Apache-2.0 OR Zlib
- etc. — all permissive, compatible with Apache-2.0.

No GPL, no copyleft, no unknown.

### Why not alternatives

- `winit + softbuffer + egui`: more crates, needs egui_softbuffer bridge, software rendering slower
- `winit + pixels`: pixels uses wgpu (heavier), still needs egui integration
- `minifb`: minimal but no UI — would require hand-rolled immediate UI + font rendering, significant work to meet inspection/editable settings/2D+3D views
- `sdl2`: requires C SDL2 library, not pure Rust, CI complexity

### Core remains zero-dep

`Cargo.lock` shows eframe deps only under `auralite-sandbox` feature; `cargo tree -p auralite-dynamics` shows zero third-party.

## Future GPU Crate

`auralite-gpu` currently CPU reference per ADR-13, zero-dep. Future may add `wgpu` with same justification + audit.

## CI Audit

- `.github/workflows/ci.yml` now includes `cargo-deny` job (install `cargo-deny`, run `cargo deny check --all-features` for licenses/bans/advisories)
- `cargo audit` via `cargo-deny` advisories
- `deny.toml` in repo root

## Third-Party Notices

Since core is zero-dep, third-party notices currently only include sandbox optional deps when feature enabled. File `THIRD_PARTY_NOTICES.md` (to be added in R2) will list eframe transitive licenses. While zero-dep, this file can state "No third-party Rust dependencies in core, only std/core, Apache-2.0".

## Reproducibility

- Pinned toolchain `1.97.0` via `rust-toolchain.toml`
- `Cargo.lock` committed
- `cargo-deny` config pinned
- Single canonical artifact path `docs/generated/scenes.html` generated reproducibly via `cargo run -p auralite-sandbox --release` (engine-recorded JSON, no timestamp-dependent mock)
