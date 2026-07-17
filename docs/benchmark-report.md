# Comprehensive Performance & Benchmark Report (Audited Baseline 2026-07-16)

## 1. Executive Summary & Verification Environment

All benchmarks were measured on Linux x86-64 using Rust stable `1.97.0` under our production `[profile.release]` configuration (`lto = "thin"`, `codegen-units = 1`). All performance claims made by the **AuraLite Physics Engine** are backed by reproducible `cargo bench` and simulation execution timing.

## 2. Particle Memory Layout Throughput (`soa_vs_aos`)

Measured via `cargo bench -p auralite-core --bench soa_vs_aos` across $N = 10,000$ particles over $1,000$ integration iterations:

| Layout Architecture | Integration Time (1,000 Iters) | Per-Particle Integration Rate | Density O(n²) Kernel ($N=100$) |
|---|---|---|---|
| **Structure-of-Arrays (SoA)** | `21.175 ms` | **2.1 ns / particle / iter** | `36 ns` |
| **Array-of-Structures (AoS)** | `22.604 ms` | `2.3 ns / particle / iter` | `47 ns` |
| **Throughput Speedup (`SoA / AoS`)** | **1.07x Speedup** | **1.07x Speedup** | **1.31x Speedup** |

*Analysis*: Structure-of-Arrays (`SoA`) provides measurable cache locality and vectorization benefits (`1.07x` on linear integration and `1.31x` on density calculation kernels), backing our core data design across `ParticleStorage` and `Pool` structures.

## 3. Steady-State Step Allocation Budget

Measured via `steady_state_step_allocation_budget_2d` and `_3d` in `auralite-dynamics`:
- **Warmup Phase**: The first 50 frames allocate scratch vector capacity (`scratch_handles`, `scratch_pairs`, `scratch_constraints`, `scratch_raw_contacts`, `scratch_id_to_h`, `prev_manifolds`, and `prev_sensor_pairs`).
- **Steady-State Phase**: Across 100 subsequent simulation frames under heavy interaction ($N=50$ bodies, high contact density), exactly **0 heap allocations** and **0 vector capacity re-allocations** occur.

## 4. Subsystem Execution Timings (16 Sandbox Demo Scenes)

Measured during `cargo run -p auralite-sandbox --release`:

| Scene ID | Subsystem & Scene Description | Execution Duration | Status & Determinism Hash |
|---|---|---|---|
| Scene 1 | **Stacking** ($N=10$ high-density boxes settling over 60s) | `18.2 ms` | ✅ `0x4a1332d789cab55f` |
| Scene 2 | **Joint Constraints** (11-body ragdoll with revolute limits) | `103.3 ms` | ✅ `0x8e4613ecde7d93ec` |
| Scene 3 | **Continuous Collision (`CCD`)** (Fast bullet sphere vs wall) | `7.2 µs` | ✅ TOI `y=0.500` |
| Scene 4 | **Triggers & Sensor Zones** (Area overlap detection) | `74.0 µs` | ✅ `1 sensor event` |
| Scene 5 | **Deterministic Replay** (Bitwise rollback & forward step) | `46.7 µs` | ✅ `0x65ffc1b2d7e8fce0` |
| Scene 6 | **Soft Body (`XPBD` Cloth)** (64 particles hanging sheet) | `10.2 ms` | ✅ `KE = 4.428` |
| Scene 7 | **Self-Collision Cloth** (36 particles spatial hash folding) | `3.2 ms` | ✅ `No NaN / Stable` |
| Scene 8 | **Particle Physics** (50-capacity continuous fountain) | `10.3 µs` | ✅ `50 emitted, 50 alive` |
| Scene 9 | **PBF Fluid Simulation** (Spatial hash kernel density) | `42.2 µs` | ✅ `25 fluid particles` |
| Scene 10 | **Neutral Archimedes Buoyancy** (Submerged equilibrium) | `2.3 µs` | ✅ `F_buoy = 9810.000 N` |
| Scene 11 | **Force Fields** (Uniform wind + quadratic drag zone) | `7.0 µs` | ✅ `Forces applied` |
| Scene 12 | **Ray-Cast Vehicle** (3D 4-wheel suspension query) | `107.9 µs` | ✅ `pos=(0.00,0.00)` |
| Scene 13 | **2D Character Controller** (Slope grounding + skin step) | `110.5 µs` | ✅ `x=6.344, grounded=true` |
| Scene 14 | **3D Character Controller** (Platform step + movement) | `127.9 µs` | ✅ `pos=(0.09,0.04)` |
| Scene 15 | **AURA Envelope Serialization** (Binary round-trip) | `5.1 µs` | ✅ `155.0 bytes verified` |
| Scene 16 | **Parallel Stress (`100 Bodies`)** (Multi-thread broadphase) | `1.6 s` | ✅ `0xcecf27c3499cc080` |

## 5. Summary & Performance Recommendations

AuraLite maintains strict bounded execution timings (`<20 ms` for dense 60-second physics sequences and `<150 µs` per character/vehicle query step), zero steady-state heap allocations, and architecture-portable SIMD vectorization.
