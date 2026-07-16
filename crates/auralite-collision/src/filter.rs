//! Symmetric deterministic collision filtering.
/// Pair decision after callback override.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum PairDecision {
    /// Generate contacts.
    Collide,
    /// Emit overlap events without contacts.
    Trigger,
    /// Suppress the pair.
    Ignore,
}
/// Collision filter record.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct CollisionFilter {
    /// Object layer bit (one or more bits).
    pub layers: u64,
    /// Layers accepted by this object.
    pub mask: u64,
    /// Nonzero positive group forces collision; equal negative group forces ignore.
    pub group: i32,
    /// Sensor flag.
    pub sensor: bool,
}
impl Default for CollisionFilter {
    fn default() -> Self {
        Self {
            layers: 1,
            mask: u64::MAX,
            group: 0,
            sensor: false,
        }
    }
}
impl CollisionFilter {
    /// Symmetric base decision. Both masks must accept; equal nonzero groups override masks by sign.
    #[must_use]
    pub fn decide(self, rhs: Self) -> PairDecision {
        if self.group != 0 && self.group == rhs.group {
            return if self.group > 0 {
                if self.sensor || rhs.sensor {
                    PairDecision::Trigger
                } else {
                    PairDecision::Collide
                }
            } else {
                PairDecision::Ignore
            };
        }
        if self.layers & rhs.mask == 0 || rhs.layers & self.mask == 0 {
            PairDecision::Ignore
        } else if self.sensor || rhs.sensor {
            PairDecision::Trigger
        } else {
            PairDecision::Collide
        }
    }
}
/// Applies a user callback in canonical stable-ID order so argument order cannot alter semantics.
#[must_use]
pub fn decide_with_callback(
    a: (u64, CollisionFilter),
    b: (u64, CollisionFilter),
    callback: impl FnOnce(u64, u64, PairDecision) -> PairDecision,
) -> PairDecision {
    let (x, y) = if a.0 <= b.0 { (a, b) } else { (b, a) };
    callback(x.0, y.0, x.1.decide(y.1))
}
#[cfg(test)]
mod tests {
    use super::*;
    #[test]
    fn symmetric_semantics() {
        let a = CollisionFilter {
            layers: 1,
            mask: 2,
            group: 0,
            sensor: false,
        };
        let b = CollisionFilter {
            layers: 2,
            mask: 1,
            group: 0,
            sensor: true,
        };
        assert_eq!(a.decide(b), b.decide(a));
        assert_eq!(a.decide(b), PairDecision::Trigger);
    }
    #[test]
    fn groups_override() {
        let a = CollisionFilter {
            group: -2,
            ..Default::default()
        };
        assert_eq!(a.decide(a), PairDecision::Ignore);
        let b = CollisionFilter {
            group: 2,
            mask: 0,
            ..Default::default()
        };
        assert_eq!(b.decide(b), PairDecision::Collide);
    }
}
