# Benchmark report — 2026-07-16
Linux x86-64 container; CPU model/host allocation is not exposed reliably; Rust 1.97.0; release profile thin-LTO, codegen-units=1; 1000 independent 2D circle bodies, gravity/ground integration, 1000 steps at dt=1/60, and canonical hash each step. Five process runs: 39.638, 37.573, 38.504, 39.729, 39.109 ns/body-step. Median **39.109 ns/body-step**, range 37.573–39.729; 1,000,000 body-steps/run. Identical final hash `19de7b8ce4aee464`. Raw data: `benches/results/linux-x86_64-rigid.txt`.

This is a vertical-slice kernel measurement, not a full contact solver benchmark. No claims exist yet for dynamic-tree, GJK/EPA, full solver, fluid, SIMD, MT, or GPU performance.

## M1 math microbenchmark
Same environment/profile. `math_benchmark` applies 5,000,000 `Transform2::transform_point` operations per process, seven process runs, with `black_box` and a deterministic checksum. Results (ns/transform): 9.315, 9.090, 9.307, 9.136, 9.355, 9.650, 9.165. Median **9.307 ns/transform**, range 9.090–9.650. Raw output: `benches/results/linux-x86_64-math.txt`. This measures a narrow f32 kernel and makes no whole-engine throughput claim.

## M2 geometry microbenchmark
Same Linux x86-64/Rust 1.97/release profile. Seven separate runs. Box3 support mapping: 2,000,000 calls/run; results 20.640, 14.631, 18.547, 14.560, 24.135, 14.382, 14.949 ns/call; median **14.949 ns/support**, range 14.382–24.135. Deterministic BVH construction over 7,938 triangles generated 4,095 nodes; build times 19.644, 16.454, 27.152, 16.379, 21.433, 16.342, 17.714 ms; median **17.714 ms**, range 16.342–27.152. Host scheduling noise is visible, so these are descriptive rather than regression thresholds. Raw data: `benches/results/linux-x86_64-geometry-m2.txt`.
