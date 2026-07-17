//! Lockstep-oriented API helper (H10).
//! Records (step, input) streams, re-applies deterministically, hash-compares.
//! Demonstrates how existing seed+snapshot+deterministic-step constitutes lockstep.

use crate::{BodyHandle2, World2};
use auralite_math::{Real, Vec2};

/// Input event for lockstep: at given step, apply force/impulse to a body.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct InputEvent {
    /// Step index.
    pub step: u64,
    /// Force to apply (for simplicity).
    pub force: Vec2,
}

/// Recorder for lockstep inputs.
#[derive(Clone, Debug, Default)]
pub struct InputRecorder {
    /// Recorded (step, input) stream in deterministic order.
    pub events: Vec<InputEvent>,
}

impl InputRecorder {
    /// Creates new empty recorder.
    pub fn new() -> Self {
        Self { events: Vec::new() }
    }

    /// Records an input at given step.
    pub fn record(&mut self, step: u64, force: Vec2) {
        self.events.push(InputEvent { step, force });
        // Keep deterministic order by step
        self.events.sort_by_key(|e| e.step);
    }

    /// Re-applies inputs to world deterministically, stepping with given dt.
    /// `body` is the body to apply forces to.
    /// Returns final state hash.
    pub fn replay(&self, world: &mut World2, body: BodyHandle2, dt: Real, total_steps: u64) -> u64 {
        let mut event_idx = 0;
        for step in 0..total_steps {
            while event_idx < self.events.len() && self.events[event_idx].step == step {
                let ev = self.events[event_idx];
                let _ = world.apply_force(body, ev.force);
                event_idx += 1;
            }
            let _ = world.step(dt);
        }
        world.state_hash()
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::{BodyBuilder2, Collider2, ColliderShape2, Material};
    use auralite_collision::CollisionFilter;
    use auralite_geometry::Circle2;
    use auralite_math::Vec2;

    #[test]
    fn lockstep_replay_hash_equals() {
        let mut world1 = World2::default();
        let b1 = world1
            .add_body(
                BodyBuilder2::dynamic()
                    .position(Vec2 { x: 0.0, y: 5.0 })
                    .add_collider(Collider2 {
                        shape: ColliderShape2::Circle(Circle2::new(0.5).unwrap()),
                        offset: Vec2::ZERO,
                        material: Material::default(),
                        filter: CollisionFilter::default(),
                    }),
            )
            .unwrap();

        let mut recorder = InputRecorder::new();
        recorder.record(10, Vec2 { x: 1.0, y: 0.0 });
        recorder.record(30, Vec2 { x: 0.0, y: 2.0 });
        recorder.record(50, Vec2 { x: -1.0, y: 0.0 });

        let hash1 = recorder.replay(&mut world1, b1, 1.0 / 60.0, 100);

        // Second world, independent, same inputs
        let mut world2 = World2::default();
        let b2 = world2
            .add_body(
                BodyBuilder2::dynamic()
                    .position(Vec2 { x: 0.0, y: 5.0 })
                    .add_collider(Collider2 {
                        shape: ColliderShape2::Circle(Circle2::new(0.5).unwrap()),
                        offset: Vec2::ZERO,
                        material: Material::default(),
                        filter: CollisionFilter::default(),
                    }),
            )
            .unwrap();

        let hash2 = recorder.replay(&mut world2, b2, 1.0 / 60.0, 100);

        assert_eq!(
            hash1, hash2,
            "Lockstep replay of same (step,input) stream must be bitwise deterministic"
        );
    }
}
