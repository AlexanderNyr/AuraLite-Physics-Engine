# ADR 02: native 2D/3D sharing
**Status:** accepted; validated at M1.

## Context
Types must prevent accidental dimensional mixing without making 2D a 3D wrapper.
## Decision
Use distinct concrete `Vec2/Vec3`, `Rot2/Quat`, `Mat2/Mat3`, `Transform2/3`, primitives, handles and worlds. Small implementation patterns may use private macros, but solver/collision data paths remain native. Shared core contains only dimension-neutral handles, IDs, RNG, time and hashes.
## Alternatives
Const-generic vectors weaken semantic API distinctions; embedding z=0 wastes work and permits mistakes; total duplication loses maintainability.
## Consequences
Some deliberate API duplication. Rust's type checker rejects cross-dimensional calls.
## Validation
Both native suites compile; APIs have no dimension tag or runtime wrong-dimension error.
