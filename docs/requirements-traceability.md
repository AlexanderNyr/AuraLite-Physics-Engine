# Requirements traceability (living, 2026-07-16)
Status: **2 complete and 5 partial Section-5 areas**; M3 active.
| Requirement | Implementation | Tests/evidence | Status |
|---|---|---|---|
| 5.1 math/core | `auralite-math`, `auralite-core` | 9 math/core unit tests including inverse/transform/primitives | **Complete at M1**: full foundational catalog, predicates, mass foundations, finite/time/identity policy, seeded/extreme suites, f32/f64 evidence |
| 5.2 shapes | `auralite-geometry::{lib,advanced}` full native catalog, hulls, compounds, mesh/BVH, heightfields | 20 geometry tests including 10k-direction support differential, analytic mass, hull/mesh regressions | **Complete at M2**; undefined infinite/static mass and numerical accuracy documented |
| 5.3 collision/queries | analytic circle/sphere, `BroadPhase2/3` | touching/coincident/pair-order tests | Partial: dynamic trees/filtering/analytic TOI added with 100,128-check differential; convex/manifold/query algorithms remain |
| 5.4 rigid dynamics | `World2`, `World3` | falling/rest, invalid dt | Partial vertical slice only |
| 5.5 joints | none | none | Not started |
| 5.6 soft/cloth | none | none | Not started |
| 5.7 particles/fluids | none | none | Not started |
| 5.8 vehicles/character | none | none | Not started |
| 5.9 fields | none | none | Not started |
| 5.10 determinism/rollback | RNG, stable IDs/order, hashes, snapshots | rollback hash test | Partial |
| 5.11 jobs/memory/SIMD | generational pool only | pool tests | Partial |
| 5.12 GPU | none | none | Optional but requested acceptance path not started |
| 7.2 serialization | `auralite-serialize` envelope/quota | hostile/truncation tests | Partial |
| 7.3 FFI | `auralite-ffi` lifecycle/step | lifecycle test | Partial |
| 8 sandbox | headless executable | manual run recorded | Partial; visual tools missing |
| 9 platforms/CI | workflow + guides | Linux local only | Partial |
