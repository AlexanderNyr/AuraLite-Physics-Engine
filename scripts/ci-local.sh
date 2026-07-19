#!/usr/bin/env bash
# ci-local.sh — run the EXACT CI command list locally (pre-push gate).
# Mirrors .github/workflows/ci.yml (verify job + cross_check job + audit job),
# introduced 2026-07-19 after run 29583407674 escaped to CI (fuzz lints + deny
# schema drift) because local gates had been run without --all-targets/--all-features.
#
# Skippable (CI-conditional) steps:
#   - C example link/run is skipped on Windows in CI; here it runs when gcc exists.
#   - Interactive sandbox build is Linux-only in CI; here it runs when pkg-config
#     probing succeeds (or always on Linux).
# Requires: rustup toolchain 1.97.0 (rust-toolchain.toml), cargo-deny 0.20.2 for the
# audit section (install: cargo install cargo-deny --version 0.20.2 --locked),
# rustup target aarch64-unknown-linux-gnu for the cross section (optional, auto-skip).
#
# Usage: scripts/ci-local.sh            — full local CI battery (fails fast on error)
set -euo pipefail

step() { printf '\n\033[1m== %s ==\033[0m\n' "$*"; }

# --- Verify job (ubuntu-latest steps; windows/macos run the same list) ---
step "Verify Code Formatting"
cargo fmt --all --check

step "Verify Strict Clippy Lints (All Targets & Features)"
cargo clippy --workspace --all-targets --all-features -- -D warnings

step "Execute Full Workspace Test Suite (All Features)"
cargo test --workspace --all-features

step "Execute Workspace Doctests"
cargo test --doc --workspace

step "Verify Double-Precision (f64) Math Configuration"
cargo test -p auralite-math --no-default-features --features f64

step "Verify Single-Thread Dynamics Configuration"
cargo build -p auralite-dynamics --no-default-features --features single-thread

step "Build Release Artifacts"
cargo build --workspace --release

step "Run Automated Sandbox Scene Verifications (Headless)"
cargo run -p auralite-sandbox --release

step "Run Fuzz Smoke (Stable Harness, H8)"
cargo run -p auralite-fuzz --release

step "Verify Benchmark Compilation (Smoke Check)"
cargo bench --workspace --no-run

if command -v gcc >/dev/null 2>&1; then
  step "Verify C FFI Compilation and Header Drift (Linux/macOS)"
  gcc crates/auralite-ffi/c_example/main.c target/release/libauralite_ffi.a -lpthread -ldl -lm -o c_verify
  ./c_verify
  rm -f c_verify
else
  echo "(skipped: gcc not found — matches CI Windows behavior)"
fi

step "Build Interactive Sandbox (Feature Gated, No Run)"
cargo build -p auralite-sandbox --features interactive

# --- cross_check job ---
if rustup target list --installed 2>/dev/null | grep -q '^aarch64-unknown-linux-gnu$'; then
  step "Check ARM64 SIMD (NEON) Parity"
  cargo check --workspace --target aarch64-unknown-linux-gnu --all-features
else
  echo "(skipped: rustup target aarch64-unknown-linux-gnu not installed —"
  echo "         install it to reproduce the CI cross_check job)"
fi

# --- audit job ---
if command -v cargo-deny >/dev/null 2>&1; then
  step "Check Licenses, Bans, Advisories, Sources (cargo-deny $(cargo deny --version | awk '{print $2}'))"
  cargo deny check
else
  echo "(skipped: cargo-deny not installed — CI pins: cargo install cargo-deny --version 0.20.2 --locked)"
fi

step "LOCAL CI BATTERY COMPLETE — all executed steps green"
