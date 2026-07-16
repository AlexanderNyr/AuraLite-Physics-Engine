# Known Limitations (2026-07-16)

The AuraLite Physics Engine has been significantly hardened. The following limitations remain:

## Low Severity
- **3D Mesh vs. Support GJK**: Support-based GJK/EPA doesn't natively handle large `TriangleMesh` efficiently without mid-phase decomposition.
- **Weld Joint Orientation**: 3D Weld joint currently performs strict point-to-point constraint but uses a simplified snap for orientation rather than a full angular impulse solve.
- **Hull Builder Performance**: The M2 hull builder remains O(n⁴), suitable for runtime creation of small shapes only.
- **GPU Backends**: The `auralite-gpu` crate provides a CPU reference implementation. Full `wgpu` integration for hardware acceleration is in the roadmap.

## Platform Support
- **Verification Range**: Only Linux x86-64 is fully verified. Windows and macOS are configured in CI but have not been executed on real hardware.
- **Mobile Targets**: Android and iOS support is at the "cross-compile configured" stage without on-device verification.

## Performance
- **Allocation**: While steady-state simulation is low-allocation, some broad-phase rebuilds and manifold updates perform small vector reallocations.
- **SIMD**: SSE2 is the primary acceleration path for x86-64. AVX2 and ARM NEON paths are planned.
