# Changelog

## [1.0.0-rc1] - 2026-07-16
### Added
- Real 3D Joint Solver for `World3` (Weld, BallSocket, Distance, Slider, Hinge).
- `ThreadPoolScheduler` using `std::thread::scope` for multi-threading.
- SSE2 SIMD implementation in `auralite-math`.
- `SpatialHash` acceleration for PBF Fluids and Soft Body self-collision.
- World ray-casting for 3D Vehicles and Character Controllers.
- SVG visualizer and HTML reporting for the sandbox.
- C FFI extensions for World3 and body manipulation.

### Fixed
- Fixed solver pipeline order: Integrate Velocities -> Solve -> Integrate Positions.
- Fixed 2D joint identity and removal bugs.
- Stabilized contact feature IDs for warm-starting.
- Reconciled all ADRs and progress documentation with reality.

### Changed
- Refactored `auralite-gpu` to provide a functional CPU-reference mode.
- Expanded test suite to 133 unit tests.
- Hardened sandbox with 16 validated scenes.
