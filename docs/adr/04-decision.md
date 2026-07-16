# ADR 04: object storage and identity
**Status:** accepted; validated at M1.

## Decision
Objects live in slot pools addressed by `(u32 index,u32 generation)` handles. Removal increments generation before free-list reuse. Stable monotonic u64 IDs provide serialized identity and canonical ordering. Pool exhaustion fails explicitly rather than truncating an index.
## Alternatives
Pointers violate safe lifecycle/FFI goals; plain indices resurrect stale references; UUIDs add entropy/dependency and are unnecessary per-world.
## Consequences
Generation wrap is theoretically possible after 2^32 reuses of one slot and is documented as an operational limit. Stable IDs are never reused.
## Validation
Deterministic stale-handle tests include 10,000 seeded remove/reinsert cycles.
