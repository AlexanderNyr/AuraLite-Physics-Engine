# Known Limitations (2026-07-19 — R4 / CI-green refresh)

The following limitations are current and measured. Severity reflects user impact.

## Determinism Scope (Tier-B measured, 2026-07-19)

- **Cross-platform / cross-build emergent trajectories are not bitwise-identical.** Tier-A guarantees (same build and platform: ST=MT, replay, snapshot round-trip, rollback) are bitwise and CI-verified on 3 OSes. Tier-B identical-input runs on *different* architectures/codegen paths diverge in chaotic regimes via normal IEEE rounding differences — measured on the marginal 10-box tower (1000 steps): max residual speed x86-64 dev < 1.0 m/s, ARM64 NEON dev 1.077478 m/s (CI run `29682146269`), x86-64 release 1.1123444 m/s (same machine, different profile), KE ≈ 3.0 J. Consequence for users: lockstep/replays are portable per build, not across builds; multi-platform netcode must exchange state, not only inputs (this is the documented Tier-B/C contract, unchanged).

## Low Severity

- **3D manifold is single-point** per contact vs 2D multi-point clipping (`clip_contacts2`); 3D stacking deep-persistence quality is lower (known since R4 planning; functionally stable in the 100-body stress scene 16/16).
- **3D Mesh vs. Support GJK**: support-based GJK/EPA doesn't natively handle large `TriangleMesh` efficiently without mid-phase decomposition.
- **Weld Joint Orientation**: 3D Weld uses a simplified snap for orientation rather than a full angular impulse solve.
- **Hull Builder Performance**: the hull builder is O(n⁴) — runtime creation of small shapes only.
- **GPU Backends**: `auralite-gpu` is a CPU reference per ADR-13 resolved outcome; hardware `wgpu` is roadmap.
- **Marginal-stack residual jitter**: fixed-iteration solver leaves ~O(1 m/s) transient residual motion in marginally stable towers (measured values above); sleeping thresholds absorb it in typical scenes, but crisp `v < 1.0`-style settle assertions are not portable — the affected smoke test now asserts the physical stability envelope.
- **Allocation**: steady-state stepping is realloc-free (`steady_state_step_allocation_budget_2d`); some broad-phase rebuilds/manifold updates still perform small vector reallocations.
- **SIMD coverage**: SSE2 (x86-64) and NEON (aarch64) paths are real and differential-tested (`simd_fallback_*`, macOS ARM64 CI green); AVX2/AVX-512 widening is not implemented.

## Platform / Tooling

- **Windows/macOS**: CI-verified (run `29682753719`, all steps) — no local hardware observed; interactive GUI not run in CI (build-only gate, explicitly).
- **Linux ARM64**: compilation CI-verified (`cargo check --target aarch64-unknown-linux-gnu --all-features`); **no test execution** (no qemu/hardware) — compile-only guarantee.
- **Android/iOS**: guidance/scripts only (`scripts/build-android.sh` needs `ANDROID_NDK_HOME`; `build-ios.sh` needs macOS/Xcode). No SDK/device ever observed — no support claim beyond configuration.
- **Miri/TSan/ASan**: unavailable — require nightly; repo pins stable 1.97.1. Fuzz coverage is the stable self-owned harness instead (1350 iterations, CI-executed).
- **quick-xml 0.39.4 (RUSTSEC-2026-0194/0195)**: present in the sandbox tree via winit→wayland-scanner (build-time proc-macro, trusted registry XML only). Unfixable without an eframe/winit major upgrade; dispositioned with justification + **review-by 2027-01-19** in `deny.toml`/`docs/dependencies.md`. Engine and games linking the engine are unaffected (sandbox-only, not linked into shipped binaries).
- **cargo-deny schema drift**: mitigated by pinning `cargo-deny 0.20.2` in CI and `scripts/ci-local.sh` (the 2026-07-17 red run was schema drift on an unpinned install).
