# ADR 13: GPU abstraction strategy
**Status:** accepted.

## Context
GPU compute can accelerate particle/fluid simulation, soft-body constraint solves, and broad-phase tasks. A portable, optional GPU path must not increase complexity for systems without GPU support.

## Decision
- **Feature-gated `gpu` crate**: Enabled via Cargo feature. Implemented as a trait-based abstraction.
- **CPU Reference**: Due to the zero-dependency requirement for the core engine, `auralite-gpu` provides a high-performance CPU reference mode that fulfills the compute interface.
- **Roadmap**: Full `wgpu` backend planned for production integration.

## Alternatives
- No GPU path: acceptable but reduces performance potential.

## Consequences
- Engine remains dependency-free while being interface-ready for GPU acceleration.

## Validation (2026-07-16)
- CPU reference validated by `cpu_backend_works_as_reference` test.
