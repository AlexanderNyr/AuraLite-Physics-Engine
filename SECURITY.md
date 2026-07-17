# Security Policy

## Supported Versions

| Version | Supported |
|---------|-----------|
| 0.1.0-rc1 (current) | ✅ |
| <0.1.0 | ❌ |

## Reporting a Vulnerability

Please do not report security vulnerabilities via public GitHub issues.

Instead, email the maintainers via GitHub Security Advisory or contact via repository issues with label `security` (private).

We will acknowledge within 3 working days and provide a fix timeline.

## Security Considerations

- `auralite-math`: uses `x86_64` SSE2 and `aarch64` NEON intrinsics via `std::arch`, gated by `target_arch` and `is_x86_feature_detected!` where applicable. Intrinsics are wrapped in safe abstractions with `// SAFETY:` comments in `simd.rs`.
- `auralite-core`: `#![forbid(unsafe_code)]`, zero unsafe, uses `std::thread::scope` and disjoint `chunks_mut`.
- `auralite-ffi`: isolated unsafe for C pointer boundaries, null checks, generation-safe tokens, panic containment via `catch_unwind`, thread-local last-error.
- `auralite-serialize`: quota-bounded (`MAX_PAYLOAD = 64 MiB`), checksum-verified (FNV-1a), hostile-input hardened (tests `checksum_detects_corruption`, `truncated_fails`, `typed_payload_tag_check`), fuzz harness `crates/auralite-fuzz` verifies no panic on mutated envelopes.
- `auralite-sandbox`: optional dep `eframe` (MIT/Apache-2.0) audited via `cargo-deny`, no unsafe in our code (`#![forbid(unsafe_code)]` on visualizer/replay).

## Fuzzing

Stable fuzz harness `crates/auralite-fuzz` runs 1350 iterations (serialization, shape constructors, GJK, world-step ops) with 0 panics, corpus hash `c16e2c7d35b19f5d`. Run via `cargo run -p auralite-fuzz --release`.

Miri and sanitizers require nightly toolchain (current stable 1.97.0) — unavailable in this env, recorded as exact unavailability in `docs/test-report.md`.

