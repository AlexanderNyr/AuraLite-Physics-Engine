# ADR 15: FFI and ABI Strategy (R3 Update — H7 Allocator & Scheduler)

**Status:** accepted; updated in R3 for H7 (allocator story + scheduler callback).

## Context
The engine exposes a C ABI for use from other languages (C, C++, C#, Python via ctypes, etc.). Safety, lifetime management, deterministic ABI, and embedder control over allocation and scheduling are critical.

## Decision

- **Opaque handles**: Generation-safe `(index, generation)` packed into u64 token (index+1 in high 32 bits, generation low 32). Stale token detection via generation check.
- **Lifecycle**: `*_create`, `*_destroy` manage lifetime. Double-destroy safe (returns error). `world_count` tracks live worlds.
- **Error handling**: Functions return `i32` error codes (0 success, -1 invalid input/stale token, -2 panic). Thread-local `last_error` retrieves message.
- **Callbacks**:
  - **Log**: `AuraliteLogCallback(level, msg)` set via `auralite_set_log_callback` — Rust logs forwarded to C.
  - **Debug-draw**: `AuraliteDebugDrawLineCallback(x1,y1,z1,x2,y2,z2,color)` set via `auralite_set_debug_draw_line_callback`.
  - **Scheduler (H7)**: `AuraliteSchedulerCallback(chunk_count)` set via `auralite_set_scheduler_callback`. Implemented as `ExternalCScheduler` in `auralite-ffi/src/lib.rs:292` that implements `auralite_core::Scheduler` trait, invokes C callback with chunk count, then runs tasks sequentially (fallback to single-thread). Verified via test `ffi_scheduler_callback_invoked` (creates 20 overlapping bodies to trigger >16 pairs → scheduler path, asserts callback invoked with count>0). New FFI exports: `auralite_world2_step_with_external_scheduler`, `auralite_world3_step_with_external_scheduler` that use `step_with_scheduler` with external scheduler. Header updated in `CANONICAL_HEADER`.
  - **Allocator**: Per-library allocator callback is **not technically safe** in Rust because global allocator is process-wide (`#[global_allocator]`). Rust's `#[global_allocator]` replaces allocation for entire process, not per library. Per-object allocator callback would require custom allocators for every `Vec`, `Pool`, etc., which is invasive and unsafe to expose via C ABI without risking UB (use-after-free across allocators). **Decision**: Provide documented embedder-level `#[global_allocator]` guidance instead of per-library callback. Embedder can define its own global allocator (e.g., mimalloc, jemalloc) that will be used by entire process including AuraLite. Documented in this ADR and in `docs/guides/ffi-guide.md` (to be added). If per-library allocator is feasible in future (e.g., using `Allocator` trait nightly), it can be added as `AuraliteAllocator { alloc, dealloc, user_data }` but currently not safe.
- **Batched calls**: `auralite_world3_batch_query_positions` accepts arrays for reduced overhead.
- **Panic containment**: All exports wrap in `catch_unwind`, convert panic to -2.
- **Header drift check**: CI compiles C example against published header; `auralite_verify_header` checks canonical header at runtime; `auralite_header_string` returns canonical header pointer.

## Allocator Story (H7)

- **Why not per-library callback**: Rust's allocator model: `GlobalAlloc` trait is global. Replacing it affects all allocations in process. Per-library `alloc`/`dealloc` callbacks would require passing allocator to every allocation site (Vec, Box, etc.) which Rust std does not support stably. Attempting to use different allocators for different libraries in same process risks mixing allocators (allocate with one, deallocate with another) → UB.
- **Safe alternative**: Embedder defines `#[global_allocator]` in its Rust binary or uses C-level allocator that process-wide replaces malloc (e.g., `mimalloc` via `#[global_allocator]` or LD_PRELOAD). AuraLite will then use that allocator for all its allocations. Guidance:
  ```rust
  // In embedder's main.rs or lib.rs (Rust side)
  use mimalloc::MiMalloc;
  #[global_allocator]
  static GLOBAL: MiMalloc = MiMalloc;
  ```
  ```c
  // In C embedder, link with mimalloc or jemalloc that overrides malloc
  // AuraLite's Rust allocations will go through system allocator which is overridden
  ```
- **Documentation**: Provide in FFI guide and this ADR that per-library allocator callback is intentionally not provided for safety; embedder-level global allocator is the supported path. If embedder needs tracking, it can implement global allocator that logs.

## Alternatives

- COM/XPCOM: platform-specific
- C++ ABI: unstable
- C-ABI with `#[no_mangle]` extern "C": stable, portable
- For allocator: custom `BumpAllocator` per world — would require rewriting all allocations to use custom allocator, not feasible without `Allocator` API nightly; rejected for safety.

## Consequences

- Scheduler callback allows embedder to observe and potentially dispatch work to its own thread pool (currently sequential fallback, but callback receives chunk count and can be used to verify integration). Future enhancement: pass task function pointer and user_data to C for true external dispatch (requires more unsafe, but doable).
- Allocator guidance is embedder-wide, not per-library, which is safe and matches Rust's model.
- Header now includes scheduler callback typedef and setters; drift check still passes (`header_self_verify` test).

## Validation

- Lifecycle: create → step → destroy succeeds; stale token rejected (`stale_token_rejected` test)
- Scheduler: `ffi_scheduler_callback_invoked` test — sets callback, creates 20 overlapping bodies (to ensure >16 pairs → scheduler path), calls `auralite_world2_step_with_external_scheduler`, asserts callback invoked with count>0
- C example compiles and runs (`gcc crates/auralite-ffi/c_example/main.c target/release/libauralite_ffi.a -lpthread -ldl -lm -o /tmp/c_verify && /tmp/c_verify` PASS)
- Header drift: `header_self_verify` test compares `CANONICAL_HEADER` with embedded header
- Panic containment: `catch_unwind` in `boundary`

## Evidence Links

- `crates/auralite-ffi/src/lib.rs:288` type `AuraliteSchedulerCallback`, `290` static, `294` `ExternalCScheduler`, `341` setter `auralite_set_scheduler_callback`, `357` step with external scheduler, `373` World3 variant
- `crates/auralite-ffi/src/lib.rs:799` test `ffi_scheduler_callback_invoked`
- `crates/auralite-ffi/Cargo.toml` dependencies include `auralite-core`, `auralite-collision`, `auralite-geometry` for collider creation
- `docs/adr/15-decision.md` this file
- Header: `CANONICAL_HEADER` includes `AuraliteSchedulerCallback`, `auralite_set_scheduler_callback`, `auralite_world2/3_step_with_external_scheduler`
