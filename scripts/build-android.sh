#!/bin/sh
set -eu
: "${ANDROID_NDK_HOME:?install Android NDK}"
rustup target add aarch64-linux-android
cargo build --release -p auralite-ffi --target aarch64-linux-android
