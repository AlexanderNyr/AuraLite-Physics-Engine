# AuraLite Physics Engine

AuraLite is an Apache-2.0 Rust physics SDK under active implementation. The repository currently provides a dependency-free, dimension-safe math/core layer, generational storage, deterministic broad phase, native 2D and 3D rigid-body vertical slices, state hashing/snapshots, serialization framing, a C ABI proof, and a headless sandbox. **It does not yet satisfy the complete product brief; see [known limitations](docs/known-limitations.md).**

```sh
cargo test --workspace
cargo run -p auralite-sandbox
cargo run -p auralite-dynamics --example falling
```

Documentation: [architecture](docs/architecture.md) · [progress](docs/progress.md) · [test report](docs/test-report.md) · [platforms](docs/platform-support.md)
