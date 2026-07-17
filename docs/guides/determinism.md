# Determinism / Replay / Rollback Guide (Tier A)

## Tier Definitions

- **Tier A (bitwise)**: Same binary, same platform, same inputs → identical `state_hash` (u64 FNV-1a). Required+tested.
- **Tier B (within tolerance)**: Cross-platform (x86-64 vs ARM64) may have small float differences within epsilon, but no NaN/explosion.
- **Tier C (non-deterministic)**: GPU path, not guaranteed.

## State Hash

`World2::state_hash()` hashes `step + id + pos.x + pos.y + rot_angle + vel.x + vel.y + ang_vel + sleeping + kind`

`World3::state_hash()` hashes `step + id + pos.x/y/z + rot x/y/z/w + vel x/y/z + ang_vel x/y/z + sleeping + kind`

Full dynamic state mirror, fixed in Q1 (G1).

## Snapshot / Rollback

```rust
let snap = world.snapshot(); // Snapshot2 { states: Vec<(id, pos, rot, vel, ang, rest, fric, sleeping)>, step }
world.step(dt).unwrap();
world.restore(&snap).unwrap();
assert_eq!(world.state_hash(), snap_hash);
```

Tests `rollback_replays_bitwise` (3D) and `_2d` clone & assert_eq!.

## 10k-step Suite

`long_run_determinism_suite_10k_steps_2d/_3d`: build scene 5 circles/5 spheres, run 10k steps continuous, save hashes at 2500,5000,7500,10000; run independent reproduction, assert equal; run to 5000, snapshot, run to 7500, rollback to 5000, run to 10000, assert hash equals continuous.

## ST=MT

`test_multithreaded_determinism`: build world with 100 bodies, step with `ThreadPoolScheduler` vs `SingleThreadScheduler`, assert state_hash identical Tier A.

`step_with_scheduler` wired into `World2/3::step` via `#[cfg(feature="multithread")]`.

## Lockstep Helper (H10)

`crates/auralite-dynamics/src/lockstep.rs`: `InputRecorder` records `(step, force)` stream, `replay` reapplies deterministically:

```rust
use auralite_dynamics::{InputRecorder, World2, BodyBuilder2};
use auralite_math::Vec2;
let mut world1 = World2::default();
let b1 = world1.add_body(BodyBuilder2::dynamic().position(Vec2 { x: 0.0, y: 5.0 })).unwrap();
let mut rec = InputRecorder::new();
rec.record(10, Vec2 { x: 1.0, y: 0.0 });
let hash1 = rec.replay(&mut world1, b1, 1.0/60.0, 100);
// Second world same inputs → hash2 == hash1 bitwise
```

Test `lockstep_replay_hash_equals`.

## Record/Replay/Sandbox

Sandbox interactive: real seed display, record/replay buttons that invoke actual engine, snapshot/rollback controls, live hash display. Headless generates `docs/generated/scenes.html` with engine-recorded trajectories + real hashes (watermarked replay viewer).

