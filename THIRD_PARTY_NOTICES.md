# Third-Party Notices

Regenerated 2026-07-19 against the current `Cargo.lock` (322 packages incl. the eframe sandbox tree),
from a union of `cargo tree --workspace --all-features --edges normal` across the CI targets
`x86_64-unknown-linux-gnu`, `x86_64-pc-windows-msvc`, `aarch64-apple-darwin` (180 unique package+license rows),
audited and allowed by pinned `cargo-deny 0.20.2` (`cargo deny check` — green).

## Core Engine

All core physics crates (`auralite-math`, `auralite-core`, `auralite-geometry`, `auralite-collision`, `auralite-dynamics`, `auralite-softbody`, `auralite-particles`, `auralite-vehicles`, `auralite-serialize`, `auralite-ffi`, `auralite-gpu`) have **zero third-party Rust dependencies** — only `core`/`std`, Apache-2.0 original code.

Rust toolchain components (rustc, cargo, rustfmt, clippy, cargo-deny) are build tools, not redistributed.

## Sandbox Optional Dependencies (Feature `interactive`)

When built with `--features interactive`, `auralite-sandbox` depends on `eframe 0.32.x` and its transitive tree.
All licenses below are permissive and Apache-2.0-compatible; combined `OR` expressions are satisfied via at least one allow-listed term, `AND` expressions are satisfied term-by-term (see `docs/dependencies.md` for the full policy and per-license justifications, incl. BSL-1.0 / OFL-1.1 / Ubuntu-font-1.0).

## License Inventory (union over CI targets)

### MIT OR Apache-2.0 (67 packages)

- ahash v0.8.12
- arboard v3.6.1
- as-raw-xcb-connection v1.0.1
- bitflags v2.13.1
- cfg-if v1.0.4
- core-foundation v0.10.1
- core-foundation v0.9.4
- core-foundation-sys v0.8.7
- core-graphics v0.23.2
- core-graphics-types v0.1.3
- crc32fast v1.5.0
- displaydoc v0.2.6
- document-features v0.2.12
- ecolor v0.32.3
- eframe v0.32.3
- egui v0.32.3
- egui-winit v0.32.3
- egui_glow v0.32.3
- emath v0.32.3
- epaint v0.32.3
- fdeflate v0.3.7
- flate2 v1.1.9
- form_urlencoded v1.2.2
- getrandom v0.3.4
- half v2.7.1
- idna v1.1.0
- image v0.25.10
- libc v0.2.186
- litrs v1.0.0
- lock_api v0.4.14
- log v0.4.33
- memmap2 v0.9.11
- num-traits v0.2.19
- once_cell v1.21.4
- parking_lot v0.12.5
- parking_lot_core v0.9.12
- percent-encoding v2.3.2
- png v0.18.1
- proc-macro2 v1.0.106
- profiling v1.0.18
- quote v1.0.46
- scopeguard v1.2.0
- smallvec v1.15.2
- smol_str v0.2.2
- stable_deref_trait v1.2.1
- static_assertions v1.1.0
- syn v2.0.119
- thiserror v1.0.69
- thiserror v2.0.18
- thiserror-impl v1.0.69
- thiserror-impl v2.0.18
- ttf-parser v0.25.1
- unicode-segmentation v1.13.3
- url v2.5.8
- web-time v1.1.0
- webbrowser v1.2.1
- weezl v0.1.12
- windows-link v0.2.1
- windows-sys v0.52.0
- windows-sys v0.59.0
- windows-sys v0.60.2
- windows-targets v0.52.6
- windows-targets v0.53.5
- windows_x86_64_msvc v0.52.6
- windows_x86_64_msvc v0.53.1
- x11rb v0.13.2
- x11rb-protocol v0.13.2

### MIT/Apache-2.0 (8 packages)

- bitflags v1.3.2
- downcast-rs v1.2.1
- foreign-types v0.5.0
- foreign-types-macros v0.2.3
- foreign-types-shared v0.3.1
- quick-error v2.0.1
- scoped-tls v1.0.1
- winapi v0.3.9

### Apache-2.0 OR MIT (5 packages)

- idna_adapter v1.2.2
- nohash-hasher v0.2.0
- pin-project-lite v0.2.17
- polling v3.11.0
- utf8_iter v1.0.4

### MIT (41 packages)

- block2 v0.5.1
- calloop v0.13.0
- calloop v0.14.4
- calloop-wayland-source v0.3.0
- calloop-wayland-source v0.4.1
- dispatch v0.2.0
- dlib v0.5.3
- fax v0.2.7
- glutin-winit v0.5.0
- memoffset v0.9.1
- objc-sys v0.3.5
- objc2 v0.5.2
- objc2 v0.6.4
- objc2-app-kit v0.2.2
- objc2-encode v4.1.0
- objc2-foundation v0.2.2
- objc2-foundation v0.3.2
- quick-xml v0.39.4
- simd-adler32 v0.3.10
- slab v0.4.12
- smithay-client-toolkit v0.19.2
- smithay-client-toolkit v0.20.0
- smithay-clipboard v0.7.3
- synstructure v0.13.2
- tiff v0.11.3
- tracing v0.1.44
- tracing-core v0.1.36
- wayland-backend v0.3.15
- wayland-client v0.31.14
- wayland-csd-frame v0.3.0
- wayland-cursor v0.31.14
- wayland-protocols v0.32.13
- wayland-protocols-experimental v20250721.0.1
- wayland-protocols-misc v0.3.12
- wayland-protocols-plasma v0.3.12
- wayland-protocols-wlr v0.3.12
- wayland-scanner v0.31.10
- wayland-sys v0.31.11
- x11-dl v2.21.0
- xcursor v0.3.10
- xkbcommon-dl v0.4.2

### Apache-2.0 (9 packages)

- ab_glyph v0.2.32
- ab_glyph_rasterizer v0.1.10
- gethostname v1.1.0
- glutin v0.32.3
- glutin_egl_sys v0.7.1
- glutin_glx_sys v0.6.1
- glutin_wgl_sys v0.6.1
- owned_ttf_parser v0.25.1
- winit v0.30.13

### Unicode-3.0 (18 packages)

- icu_collections v2.2.0
- icu_locale_core v2.2.0
- icu_normalizer v2.2.0
- icu_normalizer_data v2.2.0
- icu_properties v2.2.0
- icu_properties_data v2.2.0
- icu_provider v2.2.0
- litemap v0.8.2
- potential_utf v0.1.5
- tinystr v0.8.3
- writeable v0.6.3
- yoke v0.8.3
- yoke-derive v0.8.2
- zerofrom v0.1.8
- zerofrom-derive v0.1.7
- zerotrie v0.2.4
- zerovec v0.11.6
- zerovec-derive v0.11.3

### (MIT OR Apache-2.0) AND Unicode-3.0 (1 packages)

- unicode-ident v1.0.24

### MIT OR Apache-2.0 OR Zlib (6 packages)

- cursor-icon v1.2.0
- glow v0.16.0
- raw-window-handle v0.6.2
- xkeysym v0.2.1
- zune-core v0.5.1
- zune-jpeg v0.5.15

### Zlib OR Apache-2.0 OR MIT (6 packages)

- bytemuck v1.25.1
- bytemuck_derive v1.11.0
- dispatch2 v0.3.1
- objc2-app-kit v0.3.2
- objc2-core-foundation v0.3.2
- objc2-core-graphics v0.3.2

### MIT OR Zlib OR Apache-2.0 (1 packages)

- miniz_oxide v0.8.9

### Apache-2.0 WITH LLVM-exception OR Apache-2.0 OR MIT (4 packages)

- linux-raw-sys v0.12.1
- linux-raw-sys v0.4.15
- rustix v0.38.44
- rustix v1.1.4

### BSD-3-Clause OR Apache-2.0 (2 packages)

- moxcms v0.8.1
- pxfm v0.1.30

### BSD-2-Clause OR Apache-2.0 OR MIT (2 packages)

- zerocopy v0.8.54
- zerocopy-derive v0.8.54

### ISC (1 packages)

- libloading v0.8.9

### 0BSD OR MIT OR Apache-2.0 (1 packages)

- adler2 v2.0.1

### Unlicense OR MIT (2 packages)

- byteorder-lite v0.1.0
- memchr v2.8.3

### Apache-2.0 AND MIT (1 packages)

- dpi v0.1.2

### BSL-1.0 (2 packages)

- clipboard-win v5.4.1
- error-code v3.3.2

### (MIT OR Apache-2.0) AND OFL-1.1 AND Ubuntu-font-1.0 (1 packages)

- epaint_default_fonts v0.32.3

### MIT / Apache-2.0 (1 packages)

- cgl v0.3.2

## Fuzz Harness & Benchmarks

`auralite-fuzz` and all benches use only core crates — zero third-party dependencies (bench uses `harness = false`, no criterion).

## Notices File

This file satisfies the Apache-2.0-clean dependency-audit requirement (DoD row 10): every redistributable dependency is listed with its license; the audit enforcing it (`cargo deny check`) runs pinned in CI and passes green.
