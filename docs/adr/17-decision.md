# ADR 17: sandbox visualization and tool strategy
**Status:** accepted; revisit during M11.

## Context
The interactive sandbox is the primary visualization and debugging tool for all engine subsystems. It must be a downstream consumer of the physics library (one-way dependency).

## Decision
- **Windowing/Graphics**: Justified dependency selected during M11 implementation (options include `winit` + `wgpu` for cross-platform, or `sdl2` for simplicity). The dependency must be purely downstream — the physics library never depends on sandbox types.
- **Scene browser**: Tree/list of demo scenes covering every subsystem (stacking, joints, ragdoll, breakables, CCD, triggers/fields, soft bodies, cloth with self-collision, particles, fluids/buoyancy, vehicles 2D+3D, controllers 2D+3D, determinism/rollback, stress scenes).
- **Controls**: Pause/resume/restart, single-step, time-scale, seed manipulation.
- **Debug draw toggles**: AABBs, contacts, normals, centers of mass, velocities, broad-phase bounds, joint limits, sleep colors, soft/cloth wireframe, particle/fluid debug.
- **Inspection panels**: Live body/constraint/joint data, collider properties, simulation stats.
- **Profiling overlay**: Per-phase timings, allocation counts, pair counts, solver iterations.
- **Determinism controls**: Seed display, record/replay, snapshot/restore, rollback UI with live state hash display.

## Alternatives
- Integrate Dear ImGui: provides immediate-mode UI with minimal setup for debug overlays and controls.
- Build custom UI on raw wgpu: more work, less flexible for rapid iteration.

## Consequences
- Sandbox is a standalone binary, not a library.
- Dependency addition is deferred until M11 implementation.
- Sandbox can serve as the primary demonstration and QA tool.

## Validation
- Every major subsystem has at least one demo in the scene browser.
- All debug-view toggles function correctly.
- Determinism controls demonstrate replay hash equality.
- The engine compiles and functions without the sandbox present.
