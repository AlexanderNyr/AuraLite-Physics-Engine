# Serialization Guide

Simulation state can be fully serialized for snapshots, replay, or networking.

## Envelopes
All AuraLite data is wrapped in a versioned, checksummed binary envelope.

```rust
use auralite_serialize::{encode, decode};
let bytes = serialize_body2(&body);
let envelope = encode(&bytes);
```

## Worlds
Serialize entire worlds to byte buffers:

```rust
let data = world.serialize_bodies();
let joints = world.serialize_joints();
```

## Snapshot/Rollback
For rapid state recovery (e.g., in networking or rewind):

```rust
let snap = world.snapshot();
world.restore(&snap).unwrap();
```
