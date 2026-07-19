# Toolchain

Current pin (single source of truth = `rust-toolchain.toml`): **Rust stable 1.97.1**, profile `minimal`, components `rustfmt`, `clippy`.

Verified 2026-07-19: `rustc 1.97.1 (8bab26f4f 2026-07-14)`, `cargo 1.97.1 (c980f4866 2026-06-30)`, host `x86_64-unknown-linux-gnu`, LLVM 22.1.6. CI installs exactly this pin on all jobs via `rustup show` (no second hardcoded copy of the version in `.github/workflows/ci.yml` — see the pin-drift incident below).

Previously verified 2026-07-16: `rustc 1.97.0 (2d8144b78 2026-07-07)`, `cargo 1.97.0 (c980f4866 2026-06-30)`, host `x86_64-unknown-linux-gnu`, LLVM 22.1.6.

## Pin-drift incident (2026-07-19, CI run 29684150926 — honest record)

For run `29684150926` the repository carried an inconsistent pin state: `.github/workflows/ci.yml` requested toolchain **1.97.1** in all jobs while `rust-toolchain.toml` still pinned **1.97.0**. `rust-toolchain.toml` takes precedence over the toolchain installed as "default" by the CI action, so every job actually resolved to 1.97.0:

- The three Verify jobs and Dependency Audit **passed, but silently on 1.97.0** (their host targets ship with any toolchain) — the green cells of that run must not be cited as 1.97.1 evidence.
- The Cross-Target Parity job **failed in 27 s** with `error[E0463]: can't find crate for 'core'` (`the 'aarch64-unknown-linux-gnu' target may not be installed`): the action had installed the aarch64 std only onto its own 1.97.1 toolchain, never onto the resolved 1.97.0.

Root-caused by exact local reproduction of the CI toolchain matrix (default 1.97.1 + aarch64 std installed, `rust-toolchain.toml` = 1.97.0): identical E0463 within seconds; the same `cargo check --workspace --target aarch64-unknown-linux-gnu --all-features` passes in 34.7 s once the pin files agree at 1.97.1.

Fix: completed the pin bump (`rust-toolchain.toml`, CONTRIBUTING, README, SECURITY, this file) and removed the hardcoded toolchain version from `ci.yml` — jobs now install via `rustup show`, so the file and the workflow cannot drift apart again.
