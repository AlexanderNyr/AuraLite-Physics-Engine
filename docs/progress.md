# Progress
## Current milestone: M3 complete on 2026-07-16; M4 starting
M0–M2 remain green. M3 work completed this session:

**Added in auralite-collision narrow phase:**
- `sat_box3_box3`: 15-axis 3D OBB-OBB SAT (3 face normals from each box + 9 edge cross products) with proper separation detection
- `epa_penetration3`: 3D EPA from initial tetrahedron, bounded with closest-face fallback, degenerate input handling
- `sat_convex3`: generic convex-vs-convex 3D using GJK separation check + EPA fallback
- `Manifold3`/`ManifoldPoint3`: 3D persistent contact points with warm-start impulse carryover via FeatureId
- `clip_contacts2`: 2D reference-face/incident-edge clipping producing up to 2 contact points with stable feature IDs
- `ShapeType2`/`ShapeType3`: pair dispatch discriminators for the full shape catalog
- Robustness battery (8 tests): deep penetration, mm-scale, km-scale, degenerate/near-zero, plate stacking, coincident, EPA degenerate first iteration

**Added in auralite-collision tree:**
- `query_with_aabbs`: overlap query returning candidate IDs and their AABBs
- `shape_cast`: swept AABB query for broad-phase shape casting

**Added in auralite-collision ccd:**
- `conservative_advancement_toi`: convex-pair CCD using iterative closest-point advancement
- `CaParams`: configurable max iterations and tolerance

**Added in auralite-geometry advanced:**
- `TriangleMesh::ray_t_bvh`: BVH-accelerated ray hit query (verified equal to brute force)
- `TriangleMesh::closest_point_bvh`: BVH-accelerated closest point query (verified equal to brute force)
- `closest_point_aabb3`, `ray_aabb3_interval`: BVH traversal helper functions

**ADRs written:** 05 (broad phase), 06 (narrow-phase convex algorithms), 07 (contact manifolds), 08 (solver strategy), 09 (soft-body/cloth), 10 (particles/fluids), 11 (memory layout), 12 (scheduling), 13 (GPU), 14 (serialization), 15 (FFI/ABI), 16 (dependencies), 17 (sandbox).

**Benchmarks recorded:** Still baseline only. Pre-M4 benchmark targets: tree operation throughput, narrow-phase pair dispatch, manifold update, CCD sweep.

**Gates:** fmt, strict clippy, 73 unit tests (30 collision, 21 geometry, 11 math, 4 core, 3 dynamics, 3 serialize, 1 ffi), f64 math (11 tests), release build, sandbox + falling example — all green.

Mark M3 complete. Resume M4: full rigid-body worlds with rotation, inertia, joints, solver, stacking.

## Resume pointer
Continue M3 from `auralite-collision`: dynamic trees/filtering/analytic sphere CCD are green. GJK plus 2D SAT/EPA and manifold persistence are green. **M3 implementation complete** as of this commit: 3D SAT for OBB-OBB (15-axis) + 3D EPA fallback, contact clipping (2D reference-face/incident-edge), Manifold3 with feature persistence, shape pair dispatch types, BVH-accelerated ray/closest-point mesh queries (mid-phase), scene queries (overlap/distance/ray/shape cast) on dynamic trees, conservative advancement CCD for convex pairs, robustness battery (deep pen, mm-scale, km-scale, degenerate, coincident, touching). All algorithms bounded and degrade gracefully.

Remaining pre-M4: record M3 benchmarks (tree insert/remove/pairs/query, narrow-phase pair throughputs, manifold update, CCD sweeps). Then proceed to M4: full rigid-body worlds 2D+3D with rotation, inertia, forces, solver, stacking.
