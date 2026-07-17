# Performance / Tuning Guide

## Methodology (H9)

- Repeated independent process runs median+range, env capture (CPU/OS/toolchain/flags), smoke vs rigorous labeling.
- See `docs/benchmark-report.md` for SoA vs AoS median 1.02x/1.20x, allocation budget zero realloc, smoke timings 16 scenes.
- Commands: `cargo bench -p auralite-core`, `cargo run -p auralite-sandbox --release`, `cargo test --lib steady_state_step_allocation_budget_2d`.

## Tuning

- Solver iterations: `world.solver_iterations` (default 10) — increase for stacking stability, decrease for speed.
- Sleep threshold: `world.sleep_threshold` (default 1e-4 2D, 1e-6 3D) — lower → less sleeping, more CPU.
- Damping: `linear_damping`, `angular_damping` on bodies — reduces velocity, helps stability.
- Broadphase: `DynamicTree2/3` rebuild-on-mutation, O(n log n) average, deterministic fixed-order merging.
- SIMD: architecture-gated SSE2 (`_mm_set_ps`) and NEON (`vld1q_f32`), scalar fallback, differential-tested f32/f64.

## Allocation Budget

Scratch buffers pre-allocated, zero realloc after warmup 50 frames. Check via `capacity()` in test.

## Profiling

Sandbox profiling overlay: broad-phase tree (µs), narrow-phase solver, joint solve, integration, allocation counts, pair counts. Use `Instant` timing in `SandboxApp::step()`.

## Performance Adjectives Mapping

Every adjective must map to measurement in benchmark-report or be reworded — see benchmark-report Section 5.

