# ADR 05: broad-phase algorithm and structure
**Status:** accepted; validated at M3.

## Context
The engine needs efficient broad-phase pair discovery for up to thousands of dynamic objects. Deterministic ordering is required for Tier A replay.

## Decision
Two parallel implementations:
- **Reference `BroadPhase2/3`**: O(n²) brute force over stable-ID sorted entries. Deterministic, simple, used for differential correctness checks.
- **Production `DynamicTree2/3`**: Height-balanced AABB tree rebuilt from scratch on every mutation (add/remove/update). Fat velocity-predicted leaf AABBs reduce missed pairs. Rebuilding from the sorted stable-ID leaf list guarantees deterministic output invariant of insertion order. Tree height is O(log n). The rebuild-per-mutation design was chosen over incremental insert/remove/rebalance (e.g. traditional dynamic AABB tree) because:
  - Elimination of incremental subtree merge variation that could break determinism.
  - Simpler correct implementation with fewer edge cases.
  - Sufficient for the target scale (thousands of bodies); rebuild cost is dominated by broad-phase query time at that scale.

## Alternatives
- Incremental insert/remove/rebalance (common in box2d/bullet): risks insertion-order-dependent tree shape and nondeterministic pair output.
- Spatial hash/grid: less suitable for heterogeneous object sizes and dynamic scaling.
- Sweep-and-prune: requires sorted endpoint list per frame; still O(n²) worst case at narrow margin.

## Consequences
- Rebuild cost is O(n) per mutation (sort + build), which is acceptable for current target scales.
- Tree output matches brute-force reference exactly (validated by 100,128-pair differential test).
- The reference O(n²) broad phase is kept for testing but not used in production paths.

## Validation
Differential test compares DynamicTree2 pairs against brute-force O(n²) over 448 random AABBs (100,128 possible pairs). Insertion-order invariance test confirms different insertion sequences produce identical pair sets. Height test confirms O(log n) balance.
