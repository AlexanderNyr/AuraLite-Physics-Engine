================================================================================
          AURALITE PHYSICS ENGINE — RELEASE DISTRIBUTION PACKAGE
                 Version 1.0.0-rc2 (Verified & Interactive)
================================================================================

This package contains prebuilt release binaries, C-ABI SDK headers and libraries,
automated verification tools, and visual snapshots of the AuraLite Physics Engine.

DIRECTORY STRUCTURE & CONTENTS
--------------------------------------------------------------------------------
1. bin/
   - auralite-sandbox : Dual-mode engine application:
       • Interactive Mode: run `./bin/auralite-sandbox --interactive` to launch
         the desktop eframe/egui Studio & Scene Editor with live property editing,
         object spawning, joint wiring, impulse dragging, and snapshot rollback.
       • Headless Mode: run `./bin/auralite-sandbox --headless` to execute all
         16 automated physics verification checks and generate fresh replay files.
   - auralite-fuzz : High-speed stress harness executing 1,350 iterations of
     randomized, extreme, and hostile inputs to verify zero panics.

2. sdk/
   - auralite.h : C-ABI header exporting generation-safe opaque u64 tokens,
     step functions, impulse applicators, and callback registrations.
   - libauralite_ffi.a : Prebuilt static library containing the complete
     dimension-safe (2D+3D) engine compiled with release optimizations.
   - libauralite_ffi.so : Dynamic shared library (on Linux targets).
   - main.c : Standalone C integration example demonstrating world creation,
     stepping, and error handling over the C-ABI.
   - c_example_verify : Compiled C binary verifying the header against the library.

3. visualizer/
   - scenes.html : Standalone HTML trajectory viewer with watermarked display
     and engine-generated 64-bit state hashes.
   - snapshot-2d.svg / snapshot-3d.svg : Exact vector snapshots of live engine worlds.

USAGE GUIDANCE & EXAMPLES
--------------------------------------------------------------------------------
# Launch the full interactive desktop editor studio:
./bin/auralite-sandbox --interactive

# Run automated headless verification across all 16 physical subsystems:
./bin/auralite-sandbox --headless

# Compile and link custom C application against the AuraLite SDK:
gcc -O3 your_app.c -I./sdk ./sdk/libauralite_ffi.a -lpthread -ldl -lm -o your_app

VERIFICATION & QUALITY ASSURANCE
--------------------------------------------------------------------------------
- All binaries compiled via Rust stable 1.97.0 / 1.97.1 with zero unsafe in core.
- Local CI battery (`scripts/ci-local.sh`) and cross-target checks (`aarch64`, `win64`,
  `darwin`, `android`, `ios`, `wasm32`) executed cleanly.
- Definition of Done verified across all 16 rows.

================================================================================
