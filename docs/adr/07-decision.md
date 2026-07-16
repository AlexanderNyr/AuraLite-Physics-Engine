# ADR 07: contact manifolds and persistence
**Status:** accepted; validated at M3.

## Context
Stable contact solving requires persistent contact points across frames to enable warm starting, reduce jitter, and produce correct stacking behavior.

## Decision
- **Manifold2**: Stores up to two contact points with per-point normal and tangent impulses cached by `FeatureId`.
- **FeatureId**: u64 derived from shape features (e.g. vertex index, edge index, face index on compound parent) — never from pointers, allocation order, or mutable state.
- **Update procedure**: Fresh contact candidates are sorted by `FeatureId`, deduplicated, and truncated to 2 points. Old manifold points with matching `FeatureId` donate their cached impulses to the new manifold.
- **3D manifold**: Planned with up to 4 contact points and the same feature-ID persistence pattern.

## Alternatives
- Discard-and-rebuild every frame: loses warm-start data, causes convergence slowdown.
- Closest-point-only: insufficient for area contacts (box faces, polygon edges).

## Consequences
- Warm starting reduces solver iterations for stacking.
- Feature-ID stability depends on deterministic shape feature ordering (e.g. edge indices, vertex ordering in hulls).
- The 2-point limit for 2D matches the maximal expected contact count for 2D polygon-polygon contacts.

## Validation
- Manifold update preserves normal_impulse and tangent_impulse across feature-matched update cycles.
- Feature ID deduplication prevents duplicate contact points.
