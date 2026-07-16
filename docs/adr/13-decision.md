# ADR 13: GPU abstraction strategy
**Status:** accepted; revisit during M9.

## Context
GPU compute can accelerate particle/fluid simulation, soft-body constraint solves, and broad-phase tasks. A portable, optional GPU path must not increase complexity for systems without GPU support.

## Decision
- **Feature-gated `gpu` crate** (planned): Must be explicitly enabled via Cargo feature. No GPU code in non-GPU crates.
- **WGSL shaders**: Stored in the repository as `.wgsl` source files, compiled to SPIR-V at build time or loaded at runtime.
- **Isolated backend**: The GPU crate uses a trait-based backend abstraction (planned: `wgpu` for cross-platform support). CPU fallback is mandatory and automatically selected when the GPU backend is unavailable.
- **GPU vs CPU correctness**: GPU results must agree with CPU results within documented numerical tolerance. Determinism limits of GPU execution are explicitly documented (GPU floating-point non-associativity).

## Alternatives
- CUDA-only: rejected for portability.
- Vulkan compute: more portable but significantly more verbose than wgpu.
- Compute shader injection into existing graphics API: precludes engine-independence.
- No GPU path: acceptable but reduces performance potential.

## Consequences
- WGSL shaders are portable across Vulkan/Metal/DX12 via wgpu.
- CPU fallback ensures the engine works without GPU hardware.
- GPU determinism is explicitly Tier C (behavioral within tolerance) rather than Tier A.
- Adding the GPU crate is purely additive; the core engine never depends on GPU availability.
- Readback overhead may reduce effective speedup for small workloads.

## Validation
- GPU results match CPU results within 1% for identical input scenes.
- CPU fallback tested without GPU device present.
- Sync/readback overhead measured and documented.
- Before/after benchmarks for GPU-accelerated workloads.
