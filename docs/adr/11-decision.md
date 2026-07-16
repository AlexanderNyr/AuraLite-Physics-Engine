# ADR 11: memory layout and data organization
**Status:** accepted; revisit during M9.

## Context
Performance-critical subsystems (broad phase, narrow phase, solver, particles, fluids) benefit from cache-friendly data layouts. The engine must balance this with development simplicity.

## Decision
- **Bodies**: Stored in `Pool<T>` (generational slot map) for AoS (Array of Structs) access — sufficient for current body counts (hundreds to low thousands).
- **Particles/fluids**: SoA (Struct of Arrays) layout planned for fluid particles where contiguous position/velocity/weight access matters.
- **Broad-phase pairs**: Produced as `Vec<(u64, u64)>` on demand; caller filters and processes pairs.
- **Constraint/contact data**: Stored per-frame in temporary `Vec`s rather than persistent structures.
- **Memory allocation**: All allocations go through `std::Vec`/`std::collections::VecDeque`; no custom allocators at this stage.

## Alternatives
- Full ECS (Entity Component System): would add a heavy dependency and architectural complexity not justified by current scale.
- Custom slab allocators: premature for current codebase maturity.
- Persistent contact graph: adds complexity; per-frame temporary arrays are simpler for deterministic ordering.

## Consequences
- AoS for bodies is simple but cache-misses on `Pool` iteration across unrelated fields.
- SoA for particles will be a measured improvement rather than a speculative design.
- No custom allocator means allocation-dependent performance may vary.
- Steady-state allocation budgets should be validated with tests.

## Validation
- Benchmark comparisons of AoS vs SoA for fluid particle storage (planned M9).
- Allocation budget tests ensure no unbounded growth in steady-state simulation.
