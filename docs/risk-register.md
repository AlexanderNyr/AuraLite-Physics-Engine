# Risk Register — R4 / CI-Green Refresh (2026-07-19)

## Closed M-Era Risks (from M1/M2) — Evidence Links

| Risk | Likelihood | Impact | Status / Evidence |
|---|---|---|---|
| Scope exceeds one execution session | Certain | Critical | **CLOSED** — continuity via progress.md resume pointers; phased commits; no completion claims while items open. |
| Robust convex collision degeneracy | High | High | **CLOSED** — bounded GJK (32 iter), EPA degenerate fallback, SAT, `clip_contacts2`, robustness test suite (30 collision tests). |
| Floating-point cross-platform drift | High | High | **CLOSED as managed** — Tier-A bitwise suite (ST=MT, 10k×3, rollback, round-trip); Tier-B divergence now *measured + documented* (see open-item row TK-2 for the residue management). |
| FFI memory safety | Medium | Critical | **CLOSED** — 2 unsafe sites inventoried with `// SAFETY:`; generation-safe handles; header drift test; C example CI-green (ubuntu/macOS); ADR-15 allocator story; scheduler callback shipped (H7). |
| Mobile/GPU SDK unavailable | Certain here | Medium | **CLOSED as Guidance-only** — Android/iOS honestly scoped (scripts exist, never executed). |
| Performance architecture premature | Medium | High | **CLOSED** — SoA ≈ AoS measured (5-run medians), zero-realloc budget test. |

## R0–R4 Risks — Disposition

| Risk | Status 2026-07-19 | Evidence |
|---|---|---|
| Single-platform verification (Windows/macOS unverified) | **CLOSED** | Run `29682753719`: Verify (windows-latest) success 240 s, Verify (macos-latest ARM64) success 147 s — all 17 steps each. Failures/cancellations that previously hid evidence (29583407674) disclosed in platform-support.md. |
| Sandbox dependency introduction (eframe) supply-chain | **CLOSED as managed** | 322-pkg lock fully audited by pinned cargo-deny 0.20.2 (`cargo deny check` exit 0, CI audit job green 133 s); every license allow-list entry written-justified; notices regenerated; core zero-dep. |
| 3D manifold multi-point persistence depth | OPEN (low) | Known-limitations; 2D has multi-point clipping, 3D single-point — accepted low-sev with stress-scene coverage (16/16). |
| Extreme mass ratios (km/mm scale) | OPEN (low) | Robustness tests finite; jitter documented in known-limitations. |
| Missing docs / blanket lint suppressions (H3) | **CLOSED** | Real rustdoc everywhere; doctests 9 (serialize/particles/vehicles covered); **all 4 sandbox blanket `#![allow(clippy::all,...)]` removed 2026-07-19**, 70 hidden lints fixed; clippy `-D warnings` green with zero suppressions. |
| Cone-twist joint limits enforcement (H5) | **CLOSED (R2)** | `joint3_cone_twist_limits_never_exceeded`, `joint3_cone_twist_stability_long_run` green (CI all OSes). |
| Sensor stay event (H6) | **CLOSED (R2)** | `SensorEvent::is_stay`, deterministic sorted emission. |
| FFI callback incomplete (H7) | **CLOSED (R3)** | `auralite_set_scheduler_callback` + `ExternalCScheduler`; `ffi_scheduler_callback_invoked` green. |
| Fuzzing / sanitizer absent (H8) | **CLOSED (R3/R4)** | Stable seeded harness 1350 iters 0 panics, CI step on 3 OSes; fuzz crate passes strict clippy (the 2026-07-17 escape fixed); Miri/TSan unavailability recorded with exact reasons. |
| Benchmark rigor (H9) | **CLOSED (R3)** | 5-run median+range + env capture; smoke vs rigorous labeled. |
| Lockstep API (H10) | **CLOSED (R3)** | `lockstep.rs` `InputRecorder` + `lockstep_replay_hash_equals`. |
| Doc-set incomplete (H11) | **CLOSED (R3)** | Guides + SECURITY + CONTRIBUTING + notices in tree. |
| Final report alignment (H12) | **CLOSED (R4)** | final-report DoD rows re-graded with run-linked evidence incl. green run 29682753719. |
| CI red-run 29583407674 (deny parse + fuzz lints + cancelled jobs) | **CLOSED (R4)** | Root causes fixed (pinned cargo-deny 0.20.2, schema-valid deny.toml, fuzz lints), `fail-fast: false` prevents hidden cancellations, `scripts/ci-local.sh` prevents gate/CI flag drift. |
| macOS stacking-test fragility (run 29682146269) | **CLOSED (R4)** | Assertion re-anchored to measured physical envelope; run 29682753719 macOS green. |

## Current Open Risks (tracked)

| ID | Risk | Likelihood | Impact | Owner | Mitigation / Trigger |
|---|---|---|---|---|---|
| TK-1 | quick-xml 0.39.4 advisories (RUSTSEC-2026-0194/0195) in sandbox tree | Low (build-time-only, trusted XML) | Medium (if XML ever becomes untrusted) | QA/CI + Sandbox | dispositioned with justification in deny.toml/dependencies.md; **hard trigger: review-by 2027-01-19 or any eframe/winit upgrade → remove ignores, verify `cargo deny check`** |
| TK-2 | Tier-B emergent drift surprises another assertion | Medium | Medium | QA | suite sweep done 2026-07-19 (only marginal-stack threshold was fragile); new tests rule: emergent-value assertions must use physical envelopes with measured headroom, never crisp chaos-band thresholds |
| TK-3 | eframe/winit major upgrade (API churn in interactive.rs; clears TK-1) | Certain (eventually) | Medium | Sandbox | do on a branch; run ci-local.sh + interactive smoke; update ADR-17 addendum |
| TK-4 | ARM64-native engine test gaps (compile-only verification) | Medium | Low | QA/CI | optional qemu-user or self-hosted ARM runner; NEON covered today by macOS ARM64 CI tests |
| TK-5 | cargo-deny schema drift on future upgrades | Low | Medium | QA/CI | pinned 0.20.2 everywhere (CI + ci-local.sh contributing note); intentionally upgrade only after reading changelog |
| TK-6 | Interactive GUI never *run* in CI (build-only) | Certain | Low | Sandbox | documented explicitly; manual DISPLAY smoke is the verification path (`--features interactive -- --interactive`) |

## Ownership

- Architect: DoD truth, ADR-16/17, TK-3
- Simulation/Numerics/Collision/Solver: TK-2, manifold/mass-ratio rows
- SDK/FFI: safety docs, allocator/scheduler callbacks, header drift
- QA/Fuzz/Benchmark: TK-1 (review-by), TK-2, TK-5, fuzz harness, benchmark medians
- CI/Release: platform matrix, deny audit pinning, changelog, final-report
- Tech Writer: risk-register, guides, notices, continuity docs freshness
