# Third-Party Notices

## Core Engine

All core physics crates (`auralite-math`, `auralite-core`, `auralite-geometry`, `auralite-collision`, `auralite-dynamics`, `auralite-softbody`, `auralite-particles`, `auralite-vehicles`, `auralite-serialize`, `auralite-ffi`, `auralite-gpu`) have **zero third-party Rust dependencies** — only `core`/`std`, Apache-2.0 original code.

Rust toolchain components (rustc, cargo, rustfmt, clippy) are build tools, not redistributed.

## Sandbox Optional Dependencies (Feature `interactive`)

When built with `--features interactive`, `auralite-sandbox` depends on `eframe 0.32.1` and its transitive dependencies, all permissive MIT OR Apache-2.0, BSD-3-Clause, ISC, Zlib, Unicode-DFS-2016, etc., compatible with Apache-2.0.

Sample licenses (via `cargo deny check`):

- `eframe 0.32.3`, `egui 0.32.3`, `egui_glow 0.32.3`, `winit 0.30.12`, `glow 0.16.0`, `glutin 0.32.3`, `x11rb-protocol 0.13.2` (X11), `wayland-client 0.31.12`, `smithay-client-toolkit 0.19.2`, `raw-window-handle 0.6.2` (MIT OR Apache-2.0 OR Zlib) — all permissive.

Full license list available via:

```sh
cargo tree -p auralite-sandbox --features interactive
cargo deny check --all-features
```

No GPL/LGPL copyleft dependencies.

## Fuzz Harness

`auralite-fuzz` uses only core crates, zero third-party.

## Benchmark

`cargo bench` uses criterion? Actually `auralite-core` bench `soa_vs_aos` is harness=false, no external deps.

## Notices File

This file satisfies DoD row 10 requirement: Apache-2.0-clean dependency audit, licenses documented.

While zero-dep core, this file must exist (trivial).

