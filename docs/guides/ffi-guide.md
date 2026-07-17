# C FFI Guide

## Overview

`auralite-ffi` exposes C ABI with generation-safe tokens (u64 index+generation), thread-local last-error, panic containment.

## Header

Canonical header in `crates/auralite-ffi/src/lib.rs::CANONICAL_HEADER`, also available via `auralite_header_string()`.

```c
#include "auralite.h"
uint64_t token;
auralite_world2_create(&token);
auralite_world2_step(token, 0.016f);
auralite_world2_destroy(token);
```

## Functions

- `auralite_api_version`, `auralite_abi_version`, `auralite_last_error`
- `auralite_set_log_callback`, `auralite_set_debug_draw_line_callback`, `auralite_set_scheduler_callback` (H7)
- `auralite_world2_create`, `auralite_world3_create`, `auralite_world2_step`, `auralite_world3_step`, `auralite_world2_step_with_external_scheduler`, `auralite_world3_step_with_external_scheduler` (H7)
- `auralite_world2_add_body`, `auralite_world3_add_body` (now adds default circle collider for broadphase testing)
- `auralite_world2_body_query`, `auralite_world3_body_query`, `auralite_world2_body_apply_impulse`, `auralite_world3_body_apply_impulse`, `auralite_world3_batch_query_positions`
- `auralite_header_string`, `auralite_verify_header`
- `auralite_world_count`

## Safety

Every export has `# Safety` section: pointers must be valid, null checks return error, token must be valid, dt finite positive. See `crates/auralite-ffi/src/lib.rs` docs.

## Allocator Story (H7)

Rust global allocator is process-wide. Per-library allocator callback is not safe (mixing allocators → UB). Instead, embedder defines `#[global_allocator]`:

```rust
use mimalloc::MiMalloc;
#[global_allocator]
static GLOBAL: MiMalloc = MiMalloc;
```

Or link C allocator that overrides malloc. AuraLite will use it. Documented in ADR-15.

## Scheduler Story (H7)

External scheduler callback `AuraliteSchedulerCallback(chunk_count)` set via `auralite_set_scheduler_callback`. `ExternalCScheduler` implements `Scheduler` trait, invokes C callback then runs tasks sequentially. Verified via `ffi_scheduler_callback_invoked` test (20 overlapping bodies → >16 pairs → scheduler path, callback invoked).

## Example

See `crates/auralite-ffi/c_example/main.c` compiled via `gcc crates/auralite-ffi/c_example/main.c target/release/libauralite_ffi.a -lpthread -ldl -lm -o /tmp/c_verify && /tmp/c_verify`

## Building

- Linux x86-64: `cargo build -p auralite-ffi --release`
- Android: `scripts/build-android.sh` requires `ANDROID_NDK_HOME`
- iOS: `scripts/build-ios.sh` requires macOS/Xcode
