# ADR 05: project decision
**Status:** accepted for foundation / revisit before dependent milestone.

## Context
The product requires correctness, deterministic ordering, portability, and measured optimization.
## Decision
Use dependency-free stable Rust reference paths; f32 default with opt-in f64 math; separate native 2D/3D worlds; +Y-up right-handed metres; generational pools and stable IDs; bounded/canonical algorithms; XPBD planned for deformables, PBF planned for fluid; CPU reference before SIMD/GPU; little-endian versioned serialization; isolated C ABI unsafe; sandbox remains downstream.
## Alternatives
External physics engines are forbidden. Premature GPU-only, nightly SIMD, pointer APIs, and unordered simulation maps were rejected.
## Consequences
More implementation work, but auditable behavior and portable fallback. This ADR must be specialized with measurements during its owning milestone.
## Validation
Strict build/tests, differential/property tests, state hashes, allocation/performance measurements, and honest platform matrix.
