# Multithreading Guide

AuraLite is designed for high-performance parallel execution while maintaining strict bitwise determinism.

## ThreadPoolScheduler
The engine includes a built-in `ThreadPoolScheduler` that uses `std::thread::scope`.

```rust
use auralite_core::ThreadPoolScheduler;
let mut scheduler = ThreadPoolScheduler::default();
```

## Parallel World Step
To enable parallel execution, configure your world features and ensure a scheduler is available.
Note: Current integration is automatic when the `multithread` feature is enabled.

## Determinism (Tier A)
Multi-threaded execution is guaranteed to produce bitwise-identical results to single-threaded execution. This is verified by comparing `state_hash` outputs across different execution modes.
