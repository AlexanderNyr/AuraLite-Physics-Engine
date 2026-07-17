# Progress & Phase Completion Log

## Current Phase: Phase R2 Partial — Phase R3 Next (Final Verification & Truth Pass)

### Phase Checklist & Status (Original Q0-Q5 + R0-R4)

- [x] **Phase Q0-Q4 Legacy** (historical, closed)
- [x] **Phase R0 — Verify (Completed 2026-07-17)**
  - Gates measured, platform matrix rewritten truthfully (H2), artifact drift fixed (single output path), test-report corrected (138 lib +6 doctests =144), flagship tests spot-verified, CI observation run 29574448824 failure.
- [x] **Phase R1 — Sandbox Truth (Completed 2026-07-17, commit 8f68d41)**
  - H1 fixed: removed mocked JS counter*1337 pseudo-hash, built real interactive desktop sandbox with eframe 0.32.1 (glow+default_fonts+x11+wayland, optional feature, default-features off, MIT/Apache-2.0, deny.toml + CI audit + ADR-17 + dependencies.md same commit)
  - Headless 16/16 PASS + generates 2.0 MB `docs/generated/scenes.html` watermarked recorded-replay viewer (real engine trajectories + real 64-bit hashes, no physics in JS, playback scrub)
  - Interactive: `cargo run -p auralite-sandbox --release --features interactive -- --interactive` opens 1200x800 window with scene browser 16, time controls, debug toggles, inspection, editable runtime, profiling, real determinism controls/hashes, 2D/3D views
- [x] **Phase R2 — API/Document Integrity Partial (2026-07-17, current)**
  - H5 cone-twist: **DONE** — Added `JointType3::ConeTwist { axis_local, swing_limit, twist_limit }` in `joints.rs:491` with swing/twist decomposition and enforcement, tests `joint3_cone_twist_limits_never_exceeded` and `stability_long_run` PASS
  - H6 sensor stay: **DONE** — Added `is_stay` to `SensorEvent` (`lib.rs:909`), emits stay for ongoing pairs in deterministic sorted order (`lib.rs:1266`), methods `is_begin`/`is_end`/`is_stay`
  - H4 risk-register: **DONE** — Rewrote `docs/risk-register.md` closing M-era risks with evidence links and adding current risks (single-platform, sandbox dep, 3D manifold depth, extreme mass ratios, H3-H12 gaps) with owners/status
  - H3 docs: **PARTIAL** — Removed blanket allow from `auralite-gpu`, added docs for joints.rs ConeTwist and many fields, but 257 missing_docs remain in dynamics + blanket allows still in ffi, particles, serialize, softbody, vehicles. Clippy FAIL. Doctests still only 6, need serialize/particles/vehicles at least one each.
  - H7 FFI callbacks: **NOT DONE** — log+debug-draw present, allocator story pending ADR-15 update, scheduler callback not yet implemented
  - H10 lockstep: **NOT DONE** — no input-recording/replay helper yet
  - H11 doc-set: **NOT DONE** — many guides missing
  - H12 report: **PARTIAL** — Updated `final-report.md` to honest interim with per-row evidence links, corrected rows 3 and 5, marked 7/8/9 as not green. Progress, traceability, known-limitations need final sync.

- [ ] **Phase R3 — QA Completion (Next — H8/H9 + holes)**
  - H8 fuzzing: Add stable self-owned fuzz harness (seeded deterministic mutators over serialization parsers, shape constructors, narrow-phase, world-step ops), wire fuzz-smoke CI, record corpus/outcomes in test-report, Miri/sanitizer/TSan attempts with exact unavailability reasons
  - H9 benchmarks: Upgrade methodology — repeated independent runs median+range, env capture (CPU/OS/toolchain/flags), label smoke, map performance adjectives to measurements
  - Additional holes: 3D manifold multi-point persistence, World3 ground/static parity, kinematic-platform behavior in controllers

- [ ] **Phase R4 — Final Report & Presentation**
  - Regenerate DoD evidence table with per-row links (file:line/test-name/command-output) — done partially in this interim final-report, needs final polish
  - Changelog entry, refresh platform matrix + risk register, present repo entry points and final report
  - After R3/R4, if all rows green, restore PRODUCTION COMPLETE status; else honest interim remains

---

### Resume Pointer (Exact Next File / Task / Command)

1. **Target Files**:
   - `crates/auralite-dynamics/src/lib.rs` (missing docs 257, sensor stay)
   - `crates/auralite-dynamics/src/joints.rs` (cone-twist done, still missing docs for some methods)
   - `crates/auralite-ffi/src/lib.rs` (H3 Safety docs + H7 scheduler callback)
   - `crates/auralite-particles/src/lib.rs`, `serialize`, `softbody`, `vehicles` (remove blanket allow, add docs)
   - `docs/final-report.md` (this file, now honest interim)
   - `docs/risk-register.md` (done), `docs/known-limitations.md`, `docs/requirements-traceability.md`, `docs/progress.md`
   - `docs/guides/*`, `SECURITY.md`, `CONTRIBUTING.md`, `THIRD_PARTY_NOTICES.md` (H11)
   - `fuzz/` harness (H8), `benches/` methodology (H9), lockstep helper (H10)

2. **Next Tasks (R3)**:
   - H8: Create `fuzz/` directory with stable harness: `fuzz_serialization`, `fuzz_shape`, `fuzz_narrowphase`, `fuzz_world_ops` — each seeded deterministic, mutates bytes, calls parsers/constructors, checks for panic/UB, records corpus in `docs/test-report.md`. Wire `fuzz-smoke` step into `.github/workflows/ci.yml` (bounded 60s).
   - H9: Update `docs/benchmark-report.md` methodology section with repeated runs (e.g., 5 independent process runs median+range), env capture (`lscpu`, `uname -a`, `rustc --version`, `cargo --version`, profile flags), label sandbox scene timings as "smoke".
   - H3: Continue removing blanket allows and adding docs crate by crate; add doctests for serialize (`encode`/`decode` round-trip), particles (`PbfFluid` density), vehicles (`Vehicle3` creation).
   - H7: Implement `AuraliteSchedulerCallback` typedef, `auralite_set_scheduler_callback`, `ExternalCScheduler` implementing `Scheduler` trait that calls C callback, add step functions `auralite_world2_step_with_external_scheduler`, update `CANONICAL_HEADER`, add test `ffi_scheduler_callback_invoked`.
   - H10: Add `crates/auralite-dynamics/src/lockstep.rs` helper: `InputRecorder` records `(step, input)` streams (e.g., Vec<(u64, Vec2)>), re-apply deterministically, hash-compare, with example/test `lockstep_replay_hash_equals`.

3. **Verification Commands**:
   ```sh
   export PATH="$HOME/.cargo/bin:$PATH"
   cargo fmt --all --check
   cargo clippy --workspace --all-targets --all-features -- -D warnings  # expected FAIL until H3 complete, record
   cargo test --workspace --all-features
   cargo test --doc --workspace
   cargo run -p auralite-sandbox --release  # 16/16 + generate docs/generated/scenes.html
   cargo build -p auralite-sandbox --features interactive  # no run in CI
   cargo check --workspace --target aarch64-unknown-linux-gnu --all-features
   cargo deny check --all-features
   ```

## Current Git Status (R2 Interim)

- HEAD: R1 8f68d41, plus uncommitted R2 partial (cone-twist, sensor stay, risk-register, final-report)
- Next commit: `R2 - API Integrity Partial (H4/H5/H6 + honest final-report)`
- Gates: fmt PASS, tests PASS (144), headless sandbox PASS (16/16 + 2.0 MB real replay), bench PASS, C FFI PASS, aarch64 check PASS, clippy FAIL (H3 257 missing_docs)
