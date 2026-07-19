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
- **Audit**: `deny.toml` allows only licenses actually encountered in the full all-target/all-feature lock (validated by `cargo deny check licenses`). Current allow list: MIT, Apache-2.0, Apache-2.0 WITH LLVM-exception, BSD-2-Clause, BSD-3-Clause, ISC, Zlib, Unicode-3.0, BSL-1.0, OFL-1.1, Ubuntu-font-1.0. Unknown registries/git sources denied. See disposition section below for the 2026-07-19 additions.
- **Usage**: Only in `auralite-sandbox` binary, not library; physics never depends on it.

### New-License Justifications (2026-07-19 CI repair — every allow-list addition carries a written reason)

Three allow-list entries were added on 2026-07-19 after the full cargo-deny graph surfaced them; each is dispositioned here with reuse conditions:

- **`BSL-1.0`** — carriers: `clipboard-win v5.4.1`, `error-code v3.3.2` (Windows-target-only, pulled by `arboard 3.6.1` ← `egui-winit 0.32.3` ← `eframe 0.32.3` clipboard integration; verified `cargo tree --target x86_64-pc-windows-msvc`, host Linux/macOS builds exclude them). Boost Software License 1.0: permissive, OSI-approved, GPL-compatible, Apache-2.0-compatible in practice (no copyleft, no patent trap, attribution in binary docs only). Acceptable for all future **permissive utility crates**; NOT acceptable as justification to add new core deps (core stays zero-dep).
- **`OFL-1.1`** + **`Ubuntu-font-1.0`** — carrier: `epaint_default_fonts v0.32.3` (font *assets* embedded in the binary when the sandbox `interactive` feature enables egui's `default_fonts`: Ubuntu Mono/Proportional/Proghi fonts). Font licenses govern the embedded font data, not the code: OFL-1.1 permits embedding with reserved-font-name restrictions honored (we do not modify/rename fonts); Ubuntu-font-1.0 is likewise permissive for unmodified embedding. These apply to **bundled font assets only** — never acceptable for code or crates carrying runtime logic; reject any future OFL/Ubuntu-font entry that is not a font asset. Alternative considered: disabling `default_fonts` — rejected: egui then renders no text at all (no usable UI), and no system-font fallback exists in egui 0.32 without more deps.
- **Removed as not-encountered** (same day, hygiene): `MPL-2.0`, `CC0-1.0`, `Unicode-DFS-2016`, `CDLA-Permissive-2.0` — `cargo deny check licenses` flagged them `license-not-encountered` over the full graph; empty allowances misstate policy, so they were dropped rather than kept "just in case".

### Advisory Dispositions (2026-07-19)

- **RUSTSEC-2026-0194 / RUSTSEC-2026-0195 (`quick-xml 0.39.4`)** — ignored *with written justification and review date* in `deny.toml [advisories] ignore`. Evidence gathered for the disposition:
  - **Only consumer**: `wayland-scanner 0.31.10` (proc-macro), via `winit 0.30.13` → `smithay-client-toolkit` / `wayland-client 0.31.14`. Verified: `cargo tree -i quick-xml --all-features --all-targets` shows no other dependent.
  - **Untrusted-input analysis**: wayland-scanner parses wayland protocol XML *at build time* from the pinned, checksummed wayland-protocols crates in the local crates.io registry (Cargo.lock checksums). The vulnerable paths (quadratic duplicate-attribute scan; unbounded namespace allocation) cannot receive attacker-controlled bytes. Neither function is called at runtime; quick-xml is not linked into the produced sandbox binary... (it is a proc-macro host-side only).
  - **Upgrade impossibility (no cherry-picking)**: remediation requires `quick-xml >= 0.41.0`, but `wayland-scanner 0.31.10` — the *latest published wayland-scanner* — pins `quick-xml = "0.39"`, and `eframe 0.32.x` pins `winit ^0.30.12` which pins `sctk 0.19.x`/`wayland-client 0.31.x`. `cargo update -p quick-xml --precise 0.41.0` is rejected by the resolver. The real fix is an eframe/winit major upgrade, tracked for the next dependency refresh.
  - **Review-by: 2027-01-19** (6 months) or immediately upon any eframe/winit upgrade, whichever is first. Must be re-dispositioned then; not a permanent waiver.

## CI Audit

- `.github/workflows/ci.yml` audit job installs **pinned** `cargo-deny 0.20.2` (`cargo install cargo-deny --version 0.20.2 --locked`) — an unpinned install drifted schema and rejected `deny.toml` on 2026-07-17 (run 29583407674), one of the two causes of the red run.
- **Observed green in CI**: run `29682753719` (2026-07-19), job "Dependency Audit (cargo-deny)" — success, 133 s, https://github.com/AlexanderNyr/AuraLite-Physics-Engine/actions/runs/29682753719. Locally reproduced: `cargo deny check` exit 0 (`advisories ok, bans ok, licenses ok, sources ok`).
- Single canonical invocation: `cargo deny check` at repo root. `[graph] all-features = true` in `deny.toml` resolves the full workspace feature graph **including** the sandbox `interactive` feature (eframe/winit/wayland/quick-xml tree); the former `cargo deny check --all-features [--manifest-path ...]` steps used flags that are invalid on pinned `check` and were removed (feature selection lives in `[graph]`, not the CLI).
- Advisories, bans, licenses, sources all checked; policy text changes belong in this file + ADR-16/17.

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

## Third-Party Notices

Since core is zero-dep, third-party notices currently only include sandbox optional deps when feature enabled. File `THIRD_PARTY_NOTICES.md` (to be added in R2) will list eframe transitive licenses. While zero-dep, this file can state "No third-party Rust dependencies in core, only std/core, Apache-2.0".

## Reproducibility

- Pinned toolchain `1.97.1` via `rust-toolchain.toml`
- `Cargo.lock` committed
- `cargo-deny` config pinned
- Single canonical artifact path `docs/generated/scenes.html` generated reproducibly via `cargo run -p auralite-sandbox --release` (engine-recorded JSON, no timestamp-dependent mock)
