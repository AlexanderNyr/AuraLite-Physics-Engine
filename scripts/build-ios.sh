#!/bin/sh
set -eu
[ "$(uname)" = Darwin ] || { echo 'requires macOS/Xcode' >&2; exit 1; }
rustup target add aarch64-apple-ios aarch64-apple-ios-sim
cargo build --release -p auralite-ffi --target aarch64-apple-ios
cargo build --release -p auralite-ffi --target aarch64-apple-ios-sim
