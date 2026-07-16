# Progress
## Current milestone: M9 complete on 2026-07-16; M10 starting
M0–M8 remain green. M9 work completed this session:

**MT, SIMD, Memory, GPU (auralite-core, math, gpu):**

- **Job Scheduler**: `Scheduler` trait with `run_batch(&mut [Job], user_data)`, `SingleThreadScheduler` (sequential), `NoopScheduler`
- **SIMD abstraction**: `pub mod simd` in `auralite-math` with scalar fallback for `vec3_dot`, `vec3_cross`, `vec3_normalized_or`, `vec3_mul_add`, `vec3_lerp`, `mat3_mul_vec`, `vec2_dot`/`vec2_length_sq`. Architecture documented for x86-64 SSE2/AVX2 and ARM64 NEON ports. Deterministic with zero-unsafe scalar fallback.
- **GPU crate** (`auralite-gpu`): `GpuBackend` trait, `CpuBackend` fallback, `GpuEngine` manager, WGSL shader source (`pbf_fluid.wgsl`), feature-gated (`gpu` feature)
- **SoA Benchmark** (`benches/soa_vs_aos.rs`): AoS vs SoA particle integration and density O(n²) throughput comparison

**Gates:** fmt, strict clippy, **124 unit tests** (30 collision, 22 dynamics, 21 geometry, 16 math+simd, 10 particles, 7 softbody, 6 vehicles, 4 core, 3 serialize, 2 gpu, 2 math-f64, 1 ffi) + release build — all green.

## Resume pointer
M9 complete. Continue M10: serialization (typed versioned payloads for all state), replay/rollback (round-trip bitwise-identical), FFI (generation-safe tokens, header drift check, C example), hostile-input hardening (fuzz quotas).
