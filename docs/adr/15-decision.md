# ADR 15: FFI and ABI strategy
**Status:** accepted.

## Context
The engine exposes a C ABI for use from other languages (C, C++, C#, Python via ctypes, etc.). Safety, lifetime management, and deterministic ABI are critical.

## Decision
- **Opaque handles**: Public API returns and accepts opaque pointer-sized tokens (currently raw indices; planned generation-safe `(index, generation)` packed into a u64).
- **Lifecycle**: `*_create`, `*_destroy` functions manage object lifetime. Destroy on invalid handle returns an error code; double-destroy is safe (marks handle invalid).
- **Error handling**: Functions return `i32` error codes. A thread-local `last_error` accessor retrieves the most recent error message string.
- **Callbacks**: Planned: allocate, deallocate, log, debug-draw, scheduler dispatch, and filter decision functions passed as function pointers with user-data `void*`.
- **Batched calls**: Planned: `step_batch` accepts arrays of operations for reduced FFI overhead.
- **Panic containment**: All exported functions wrap body logic in `std::panic::catch_unwind`. Panics are converted to error codes.
- **Header drift check**: CI compiles a C program against the published header to detect drift between Rust definitions and the header.

## Alternatives
- COM/XPCOM interfaces: too platform-specific.
- C++ ABI: not stable across compilers/versions.
- C-ABI with `#[no_mangle]` extern "C": the stable, portable choice.

## Consequences
- C ABI limits API expressiveness (no generics, no overloading).
- Every API function is a potential panic boundary; catch_unwind overhead is acceptable for physics-cost-per-frame.
- Header generation is manual; automated generation would reduce drift risk.
- Thread-local storage adds minor overhead for error retrieval.

## Validation
- Lifecycle test: create → step → destroy succeeds; operations on destroyed handle fail.
- C example compiled and tested (where C compiler available).
- Header drift CI check.
- Panic containment: test panicking inside callback is caught and returned as error.
