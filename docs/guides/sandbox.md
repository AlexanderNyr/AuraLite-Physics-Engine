# Sandbox Guide (Real Interactive Sandbox — H1 Fixed)

## Headless (CI, 16/16 scenes)

```sh
cargo run -p auralite-sandbox --release
# Output: 16 scenes PASS + generating docs/generated/scenes.html (2.0 MB, watermarked recorded-replay viewer)
```

Generates `docs/generated/scenes.html` single canonical path (root `scenes.html` removed, gitignored). Viewer is **recorded replay**: engine-written per-frame trajectories + real 64-bit state hashes exported as JSON, playback scrub, watermark "RECORDED REPLAY — NOT LIVE SIMULATION", badge "REAL STATE HASHES". No physics in JS. Regenerate via same command.

## Interactive (Desktop Windowed App, DoD-5)

Requires feature `interactive` and display (X11/Wayland):

```sh
cargo run -p auralite-sandbox --release --features interactive -- --interactive
# Opens 1200x800 window, real engine stepping, no mocks
```

### Dependency Justification (ADR-17)

`eframe 0.32.1` (winit 0.30 + glow 0.16 + egui 0.32) optional, default-features off (`glow, default_fonts, x11, wayland`), MIT/Apache-2.0, `deny.toml` audit, CI job `audit` runs `cargo deny check`. Core remains zero-dep.

### Features

- **Scene browser**: 16 subsystems (stacking, joints, CCD, triggers, replay, cloth 8x8, self-collision 6x6, particles, fluid, buoyancy, fields, vehicle 3D, char 2D/3D, serialization, stress)
- **Time controls**: pause/resume (bool), restart (rebuild world), single-step (step when paused), time-scale slider 0.1x-3.0x, dt slider 0.001-0.033, substeps 1-8
- **Debug toggles**: AABBs (rects), contacts (not yet visualized as points, placeholder), normals, COMs, velocities (cyan lines), broad-phase bounds, joints (yellow lines), sleep colors (gray vs blue/orange), softbody (green edge lines), particles (cyan circles)
- **Inspection**: body list with pos/vel/sleep/kind, selected highlight (click nearest body <20px, yellow ring), joints list, sensor events (Begin/Stay/End)
- **Editable runtime**: gravity2 (DragValue x/y), gravity3 (x/y/z), solver iterations Slider 1-50, material friction/restitution DragValue applied to selected body via `body_mut`
- **Profiling overlay**: last step µs via `Instant`, broad-phase estimate, body counts
- **Determinism controls**: real `state_hash` display (`World2/3::state_hash()`), snapshot/rollback via `world.snapshot()/restore()`, record/replay flags, seed from step count. No pseudo-hash.
- **2D view**: world coords → screen (scale 30, off center+100), circles for bodies, velocity lines, AABB rects (dark green), joints, selected yellow
- **3D view**: isometric projection `(x - z*0.4, y - z*0.2)*scale`, circles, selection
- **Softbody**: orange particle circles, green edge lines (first 300)
- **Particles/fluid**: cyan circles, alive count

### Implementation Links

- `crates/auralite-sandbox/src/interactive.rs:1` — `SandboxApp` real engine
- `replay.rs` — recording structs, `record_world2/3`, `build_replays_json` manual JSON (no serde)
- `visualizer.rs::generate_recorded_replay_viewer()` — watermarked HTML, embeds `REPLAY_DATA` JSON, playback scrub, hash display
- `main.rs::generate_all_replays()` — builds 16 scenes, steps 60-180 frames recording real positions+hashes

### CI

Headless path runs in CI (`cargo run -p auralite-sandbox --release`). Interactive build checked via `cargo build -p auralite-sandbox --features interactive` (no run, requires display). Fails gracefully in headless with message.

