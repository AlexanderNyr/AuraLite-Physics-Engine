# ADR 17: Sandbox Visualization & Tool Strategy (R1 Truth Pass — 2026-07-17)

**Status:** accepted; implemented in R1 (H1 fix).

## Context
The prior "interactive sandbox studio" was a mock: `crates/auralite-sandbox/src/visualizer.rs` JS `simStep()` only incremented counters `stepCount +=1; simTime += …`, no physics executed; scenes rendered baked static `scenesData`; "live state hash" computed as `BigInt(sdata.hash) + BigInt(stepCount*1337)` annotated "Deterministic pseudo hash variation for visualization"; Snapshot/Rollback restored only counters. This violated the hardest rule (mocks must not count) and falsified DoD row 5.

Requirement DoD-5: **real visual interactive sandbox** — desktop windowed app, engine-driven, with scene browser 16 subsystems, time controls, debug toggles, inspection, editable runtime settings, profiling overlay, real determinism controls/hashes, 2D+3D views. HTML artifact may remain only as explicitly-labeled recorded-replay viewer with engine-written trajectories + real hashes, watermarked "RECORDED REPLAY — NOT LIVE SIMULATION", single output path.

Core must stay zero-dep (ADR-16); sandbox may have justified deps with license audit, default-features off, cargo-deny, CI audit.

## Decision

**Dependency chosen: `eframe` 0.32.x (egui + winit + glow) — minimal window+pixels/softbuffer-class + immediate-mode UI in one crate.**

- **Crate**: `eframe = { version = "0.32.1", default-features = false, features = ["glow", "default_fonts", "x11", "wayland"], optional = true }` in `crates/auralite-sandbox/Cargo.toml`
- **Feature gating**: `interactive = ["dep:eframe"]` — headless `cargo run -p auralite-sandbox --release` (CI) does NOT require display; interactive requires `--features interactive -- --interactive`
- **Why eframe over winit+softbuffer+egui separately**:
  - One crate bundles windowing (winit 0.30, x11+wayland), graphics (glow OpenGL, cross-platform), immediate UI (egui 0.32)
  - Reduces integration glue vs separate softbuffer+egui_winit+egui_softbuffer
  - License clean: MIT OR Apache-2.0 (all transitive deps audited, see dependencies.md + deny.toml)
  - Provides `egui::Painter` for 2D/3D custom drawing (circles, lines, rects) needed for physics debug draw
  - Supports `glow` (OpenGL) backend which is lighter than `wgpu` for sandbox, still hardware accelerated
  - Mature, widely used, no unsafe in our code (`#![forbid(unsafe_code)]` kept on visualizer and replay)
- **Alternatives considered**:
  - `winit + softbuffer + egui`: more crates, more glue, softbuffer software rendering slower, requires egui_softbuffer bridge
  - `winit + pixels`: pixels uses wgpu (heavier, more dependencies), still needs egui integration
  - `minifb`: single crate, very minimal, but no built-in UI — would require hand-rolled immediate UI and font rendering (significant work, still need to meet inspection/editable settings requirements)
  - `sdl2`: requires C SDL2 library, not pure Rust, license complications, not ideal for CI
- **Consequences of eframe**:
  - Adds ~150 transitive crates (mostly permissive MIT/Apache-2.0, BSD, ISC — audited via cargo-deny)
  - Increases compile time for sandbox only; core crates remain zero-dep, engine still compiles without sandbox
  - Headless CI can skip interactive feature; `cargo run -p auralite-sandbox --release` still passes 16/16 scene checks without display
  - Future GPU crate may also use wgpu, but sandbox uses glow to avoid duplication

**Implementation (R1):**

- `crates/auralite-sandbox/src/interactive.rs`: `SandboxApp` implements `eframe::App`
  - Scene browser: `SceneId` enum 16 variants, selectable list covers stacking, joints, CCD, triggers, replay, cloth, self-collision, particles, fluid, buoyancy, fields, vehicle 3D, char 2D/3D, serialization, stress
  - Time controls: pause/resume (bool), restart (rebuild world), single-step (step when paused), time-scale slider 0.1x-3.0x, dt slider 0.001-0.033, substeps 1-8
  - Debug toggles: struct `DebugDraw` {aabbs, contacts, normals, com, velocities, broadphase, joints, sleep, softbody, particles} — checkboxes affect painter drawing (AABB rects, velocity lines, joint lines, sleep color, softbody edge lines, particle circles)
  - Inspection: body list with pos/vel/sleep/kind, selected body highlighted (click to select nearest), joint list, sensor events
  - Editable runtime: gravity2 (DragValue x/y), gravity3 (x/y/z), solver iterations (Slider 1-50), material edit (friction/restitution) applied to selected body via `world.body_mut`
  - Profiling overlay: `Instant` timing for last step (µs), broad-phase estimate, body counts
  - Determinism controls: real `state_hash` displayed from `World2::state_hash()` / `World3::state_hash()` (no pseudo-hash), snapshot/rollback uses `world.snapshot()` / `world.restore()` (real engine), record/replay flags (placeholder for future file recording), seed display from step count
  - 2D view: transforms world coords to screen (`scale=30`, `off_x=rect.center().x`, `off_y=center+100`), draws bodies as circles, velocity lines, AABBs, joints (yellow lines), selected highlight (yellow ring)
  - 3D view: isometric projection `(x - z*0.4, y - z*0.2)` * scale, draws spheres/boxes as circles, similar selection
  - Softbody: draws particles as orange circles, edges as green lines when debug.softbody
  - Particles/fluid: draws alive particles as cyan circles, counts from `ParticleStorage::alive_count()` (real engine)

- `crates/auralite-sandbox/src/replay.rs`: zero-dep recording structures (`ReplayBody2/3`, `ReplayFrame2/3`, `SceneReplay`), `record_world2/3` captures real positions + hash, `build_replays_json` manual JSON without serde (keeps core zero-dep)

- `crates/auralite-sandbox/src/visualizer.rs`: `SvgVisualizer` unchanged (real engine SVG), `generate_recorded_replay_viewer(replays_json)` generates watermarked HTML:
  - Embeds engine JSON as `const REPLAY_DATA = {...}`
  - Controls: scene dropdown, play/pause, reset, step, frame scrub slider, speed slider
  - Displays real hash per frame (`0x` + hash from JSON), step, time, body count
  - Canvas draws bodies from recorded data (no physics in JS)
  - Watermark div `.watermark` with text "RECORDED REPLAY — NOT LIVE SIMULATION" plus badge "RECORDED REPLAY — NOT LIVE SIMULATION" and "REAL STATE HASHES"
  - Declaration panel: "This HTML viewer plays back engine-recorded per-frame trajectories and real 64-bit state hashes produced by cargo run ... No physics executes in JavaScript."

- `crates/auralite-sandbox/src/main.rs`:
  - CLI: `--interactive` launches eframe if feature enabled, else fallback to headless with message
  - Headless: runs 16 scene checks (same as before, real engine), then `generate_all_replays()` which for each of 16 scenes builds world, steps 60-180 frames recording real positions+hashes, then `build_replays_json` + `generate_recorded_replay_viewer` → writes single canonical `docs/generated/scenes.html` (2.0 MB with 16*~120 frames). No longer writes root `scenes.html` (drift fixed in R0).
  - Absolute no mocks: every displayed hash/position comes from `state_hash()` or `record_world2/3` etc., no `stepCount*1337`

**Validation:**

- `cargo fmt --all --check` PASS after R1
- `cargo build -p auralite-sandbox --no-default-features --release` PASS 16/16 headless
- `cargo build -p auralite-sandbox --features interactive` PASS (with x11+wayland)
- `cargo run -p auralite-sandbox --release` PASS 16/16 + generates `docs/generated/scenes.html` with watermark and real hashes (verified via `grep -c "RECORDED REPLAY"` and checking hash format not pseudo)
- Interactive launch manual: `cargo run -p auralite-sandbox --release --features interactive -- --interactive` opens 1200x800 window (requires DISPLAY/Wayland). In headless CI, this flag will fail gracefully with message "requires display", which is acceptable; CI uses headless path.
- Core crates still zero-dep: `cargo tree -p auralite-math`, `auralite-core`, `auralite-dynamics` show no third-party deps.
- License audit: `cargo deny check` (with deny.toml) passes for licenses (see CI job).
- Dependencies documented in `docs/dependencies.md` with justification, default-features off, license table.

## Alternatives revisited

- Dear ImGui via `imgui-rs`: requires C++ ImGui, more unsafe, license MIT but heavier
- Custom UI on raw wgpu: more work, less flexible

## Consequences

- DoD row 5 now satisfied: real interactive desktop sandbox + watermarked recorded-replay viewer, no fabricated values
- Single output path: `docs/generated/scenes.html` canonical, root `scenes.html` deleted and gitignored (or not regenerated)
- Sandbox is downstream only: physics never depends on eframe
- CI must have `cargo-deny` audit job (added in `.github/workflows/ci.yml` cross_check or new job)

## Validation links

- `crates/auralite-sandbox/src/interactive.rs:1` — real engine stepping, hash display
- `crates/auralite-sandbox/src/replay.rs` — engine recording, no mock
- `crates/auralite-sandbox/src/visualizer.rs: generate_recorded_replay_viewer` — watermark, real JSON
- `crates/auralite-sandbox/Cargo.toml` — optional eframe with default-features false
- `deny.toml` — license allow list
- `docs/dependencies.md` — justification + license audit
- `docs/generated/scenes.html` — contains "RECORDED REPLAY — NOT LIVE SIMULATION" and real hashes

## Addendum (2026-07-19 — CI lint-truth pass)

Surfaced when the blanket `#![allow(clippy::all, ...)]` suppressions were removed from the sandbox (they had hidden 70 lints incl. dead code):

- **Record/Replay implemented for real** (was `recorded_frames` placeholder field): `SandboxApp::start_recording()` captures an engine snapshot (`World2/3::snapshot()` or soft-body clone); every stepped frame appends its real `state_hash()` to `recorded_hashes` (bounded by `MAX_RECORD_FRAMES = 100_000`); **Replay & verify** restores the start snapshot and re-steps, comparing each frame's hash to the trace and displaying a divergence report (frame index + both hashes) on mismatch. Particle-only scenes are honestly excluded (no engine snapshot API); the UI says so instead of implying support.
- **`SvgVisualizer` now genuinely exercised**: headless `generate_visual_report()` renders `docs/generated/snapshot-2d.svg` + `snapshot-3d.svg` from real stepped worlds every run (previously the struct was never constructed). `generate_interactive_sandbox_app()` (H1-era stub emitting `{"scenes":[]}`) was deleted.
- **`SceneReplay::name()/frame_count()` now used** for per-scene frame logging in the headless run output.
- **World enum payloads boxed** (`ActiveWorld::World2(Box<World2>)` etc.) — resolves `clippy::large_enum_variant` (World2 = 456 B inline) without suppression.
- **`ActiveWorld::Particles` dead variant removed**; `show_inspection`/`show_settings` are real top-bar toggles gating those panels.
- **Dependency audit disposition**: `quick-xml 0.39.4` advisories RUSTSEC-2026-0194/0195 remain unfixable within the pinned `eframe 0.32.x`/`winit 0.30.x` stack (latest wayland-scanner 0.31.10 pins quick-xml "0.39"); build-time-only, trusted-XML justification + review-by 2027-01-19 recorded in `deny.toml` and `docs/dependencies.md`. Any future egui/eframe major upgrade must revisit this addendum.
