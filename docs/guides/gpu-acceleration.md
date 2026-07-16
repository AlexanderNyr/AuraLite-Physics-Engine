# GPU Acceleration Guide

AuraLite includes an optional GPU compute layer for massive particle and cloth workloads.

## CPU Reference Mode
By default, the `auralite-gpu` crate operates in CPU reference mode. This ensures the engine remains zero-dependency in its core while providing a consistent interface for compute tasks.

## Fluid Acceleration
Accelerates the PBF density and position correction passes.

## Integration
To enable, use the `gpu` feature flag in your `Cargo.toml`.
Currently, the GPU layer provides the architecture for `wgpu` integration.
