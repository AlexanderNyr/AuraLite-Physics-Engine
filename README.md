# AuraLite Physics Engine

AuraLite is an Apache-2.0 Rust physics SDK under active development. The repository currently provides a compiling and tested foundation (M0-M11 milestones claimed, but audited with gaps). It features dimension-safe math, GJK/EPA/SAT collision, rigid bodies, PBF fluids, and a headless sandbox.

**Current Status (2026-07-16):** Project resumed to address verified defects (D1-D20) and reach full Definition of Done.

```sh
cargo test --workspace
cargo run -p auralite-sandbox
```

Documentation: [architecture](docs/architecture.md) · [progress](docs/progress.md) · [test report](docs/test-report.md) · [known limitations](docs/known-limitations.md)
