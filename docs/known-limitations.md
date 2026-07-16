# Known limitations
## Critical / blocks Definition of Done
The requested complete engine is not implemented. Missing mandatory production systems include the dynamic AABB tree; SAT/GJK/EPA/manifolds/CCD and accelerated mesh-BVH traversal; body-body rigid contacts and rotational solver; all joints; soft body/cloth; physical particles/PBF fluids/buoyancy/fields; vehicles/controllers; scheduler/SIMD/GPU; complete serialization; complete FFI callbacks/batches; and graphical interactive sandbox. Mobile packaging is guidance-only.

## High
M2 hull building is O(n^4), mesh query traversal is still brute-force despite a built BVH, and arbitrary hull/mesh inertia is low-order numerical. Current worlds support circle/sphere translation against an infinite y=0 ground only. Snapshots assume unchanged identity sets. FFI tokens are slot indices without generations and the last-error accessor/header drift check are not yet complete. Broad phase is O(n²) reference implementation.

## Evidence limits
Only Linux x86-64 is verified. Tier A is tested for one rollback scene but the required 10,000-step ×3 suite is absent. No cross-platform determinism claim. Benchmark scope is narrow.
