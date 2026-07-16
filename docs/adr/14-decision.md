# ADR 14: serialization format and strategy
**Status:** accepted; revisit during M10.

## Context
Simulation state must be serialized for replay, rollback, networking, and debugging. The format must be versioned, hostile-input-hardened, and support round-trip determinism.

## Decision
- **Binary format**: Little-endian (native on x86-64, explicit conversion on big-endian). Versioned envelope with magic bytes `AURA`, version u32, payload length u64, followed by a quota-bounded payload.
- **Typed payloads** (planned): Separate serialization traits for each subsystem:
  - World state (bodies, handles, timers)
  - Shape geometry
  - Joint state
  - Soft/cloth state
  - Particle/fluid state
  - RNG seed and step count
  - Full snapshots and incremental deltas
- **Quota bounds**: All decode paths enforce size/iteration limits derived from the envelope length to prevent hostile memory exhaustion.
- **Deterministic round-trip**: serialize(deserialize(state)) == state (Tier A bitwise).
- **Rollback API**: Public `save_snapshot()` / `restore(snapshot)` on worlds.

## Alternatives
- JSON/TOML: human-readable but inefficient for binary state and harder to bound sizes.
- FlatBuffers/Cap'n Proto: external schema dependency; adds build complexity.
- bincode: external dependency; reduces auditability.

## Consequences
- Own binary format requires manual serialization code but maximizes auditability and control.
- Quota enforcement prevents OOM on hostile input.
- Versioned envelopes allow forward compatibility.

## Validation
- Hostile input (truncated, oversized, corrupted) all fail with documented error codes.
- Round-trip tests for all typed payloads produce bitwise-identical state.
- Fuzz targets validate parser/hardening (planned).
