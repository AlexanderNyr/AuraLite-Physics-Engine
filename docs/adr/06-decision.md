# ADR 06: narrow-phase convex algorithms
**Status:** accepted.

## Context
The narrow phase must produce contact normals, depths, and witness points for all convex-convex shape pairs. Determinism, bounded iteration, and degeneration handling are required.

## Decision
- **GJK distance (2D/3D)**: Signed distance and witness points for separated/touching convex shapes using the Gilbert–Johnson–Keerthi algorithm. Iteration count is bounded; degenerate support (zero/non-finite) is handled explicitly. A duplicated-simplex check terminates early. Both 2D (triangle simplex) and 3D (tetrahedron simplex with face-adjacency closest-point) variants are implemented.
- **SAT (2D only)**: Separating Axis Theorem for polygon-polygon penetration. Provides exact minimal-separation normal for 2D convex polygons with edge-normal axes.
- **EPA (2D only)**: Expanding Polytope Algorithm for general convex-convex penetration depth. Bounded iteration with fallback to the best finite edge at the cap. Used as fallback when SAT is not available (non-polygonal shapes 2D).
- **3D penetration**: Not yet implemented. Planned as either 3D SAT expansion or a GJK–EPA hybrid with bounded termination.
- **Manifold `FeatureId` persistence**: Stable feature identifiers derived from shape geometry (not pointer/order state) enable warm-start impulse carryover across frames.

## Alternatives
- Full GJK-EPA in all dimensions: GJK+EPA in 3D is planned but not yet validated; SAT is preferred for polyhedra when available.
- Machine-learning or approximate methods: rejected for determinism and correctness requirements.

## Consequences
- Analytic circle/sphere contacts handle those primitives with exact solutions.
- GJK handles any convex pair and naturally handles touching/overlap detection via the origin-in-simplex check.
- 2D SAT is exact for polygons but requires extension for 3D polyhedra.
- EPA in 2D agrees with SAT within 2e-4 over 1,000 box depth comparisons.

## Validation
- 1,000 analytic circle-circle distance comparisons against GJK2 (tolerance 2e-4).
- 1,000 EPA depth comparisons against SAT for offset boxes (tolerance 2e-4).
- Degenerate support and zero-size simplex inputs produce finite results.
- Manifold warm-start carries cached impulses across feature-ID-matched updates.
