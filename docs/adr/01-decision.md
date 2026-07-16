# ADR 01: scalar and precision policy
**Status:** accepted.

## Context
Real-time physics benefits from f32 bandwidth, while large-world/offline users need f64. Stable Rust and a clean public scalar type are required.
## Decision
`auralite-math::Real` is f32 by default. Building that crate with `--no-default-features --features f64` selects f64 without public casts. Core identity/time uses fixed-width integers/f64 independently. Current dynamics/collision crates consume default f32; the f64-supported scope at M1 is math, predicates, transforms, primitives, and mass foundations. Both features together intentionally select f32 so workspace `--all-features` remains a valid test configuration. No fast-math.
## Alternatives
Full generic scalar traits increased API/code size; f64-only misses real-time targets; nightly SIMD was rejected.
## Consequences
Downstream full-world f64 propagation remains a later integration task and is not claimed today. Transcendentals can vary across platforms.
## Validation
The complete 11-test math suite passes under default f32 and isolated f64; seeded 10,000-case transform properties run in both.
