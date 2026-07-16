# ADR 12: scheduling and determinism architecture
**Status:** accepted.

## Context
Multi-threaded execution must preserve bitwise-identical deterministic results compared to the single-threaded execution path.

## Decision
- **Job abstraction**: The engine exposes a `Scheduler` trait with `run_batch(jobs)` allowing the caller to inject their own scheduler (e.g. rayon, thread-pool, single-thread).
- **Default scheduler**: A built-in `SingleThreadScheduler` that runs jobs sequentially. All results from this scheduler must be bitwise-identical to deterministic multi-threaded results.
- **Parallelization strategy** (planned):
  - Broad phase: parallel tree traversal/query.
  - Narrow phase: parallel pair dispatch.
  - Solver: parallel constraint preparation with deterministic reduction.
  - Soft-body/fluid: parallel constraint solves.
- **Deterministic reductions**: Parallel work products (impulses, forces) use commutative associative accumulation to ensure order-independence.

## Alternatives
- Rayon as mandatory dependency: rejected because users may need custom thread-pooling or embedded single-thread mode.
- Shared-state locks: rejected for performance and determinism concerns.
- Work-stealing without determinism guarantee: rejected because Tier A determinism is a hard requirement.

## Consequences
- Single-thread mode serves as reference for deterministic MT results.
- External scheduler integration requires callback-based API.
- Reduction determinism limits some parallel patterns (e.g. atomic increment for accumulation).

## Validation
- `single-thread` mode hash == deterministic MT mode hash (Tier A).
- Parallel vs sequential speedup measured at 1/2/4/8 threads.
- No race conditions detected by TSan (where available).
