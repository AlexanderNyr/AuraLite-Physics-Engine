# Architecture
Dependency direction: `math` + `core` → `geometry` and `collision` → `dynamics` → `{serialize, ffi, sandbox}`. Physics never depends on the sandbox. `World2`/`BodyHandle2` and `World3`/`BodyHandle3` are distinct native types. Pure crates forbid unsafe; the FFI crate isolates and inventories it.

**Broad phase:** Two parallel implementations exist: reference `BroadPhase2/3` (deterministic O(n²) stable-ID ordered brute-force pairs) and `DynamicTree2/3` (height-balanced rebuild-on-mutation AABB tree with fat velocity-predicted leaves). The tree chooses deterministic rebuild over incremental insert/remove/rebalance for simplicity and deterministic output; the O(n²) reference is kept for differential testing. See ADR-05.

**Narrow phase:** Analytic circle/sphere contacts, bounded GJK distance 2D/3D, bounded 2D SAT, bounded 2D EPA with degenerate fallback, persistent `Manifold2` with stable `FeatureId` warm-start persistence, analytic CCD TOIs for circle/sphere and circle-plane.

**Collision pipeline (planned):** Broad phase → mid-phase (mesh/heightfield BVH traversal) → narrow phase → manifold clipping → CCD.
