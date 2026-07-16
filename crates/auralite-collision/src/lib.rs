//! Native 2D/3D collision primitives and deterministic broad phases.
#![forbid(unsafe_code)]
use auralite_math::{ABS_EPSILON, Aabb2, Aabb3, Real, Vec2, Vec3};

/// A 2D circle with validated radius.
#[derive(Clone, Copy, Debug)]
pub struct Circle {
    /// Radius.
    pub radius: Real,
}
impl Circle {
    /// Constructs a positive finite circle.
    pub fn new(radius: Real) -> Result<Self, ShapeError> {
        if radius > 0.0 && radius.is_finite() {
            Ok(Self { radius })
        } else {
            Err(ShapeError)
        }
    }
}
/// A 3D sphere with validated radius.
#[derive(Clone, Copy, Debug)]
pub struct Sphere {
    /// Radius.
    pub radius: Real,
}
impl Sphere {
    /// Constructs a positive finite sphere.
    pub fn new(radius: Real) -> Result<Self, ShapeError> {
        if radius > 0.0 && radius.is_finite() {
            Ok(Self { radius })
        } else {
            Err(ShapeError)
        }
    }
}
/// Invalid shape dimensions.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct ShapeError;
/// Contact returned by analytic narrow phase.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct Contact2 {
    /// Unit normal from A to B.
    pub normal: Vec2,
    /// Nonnegative overlap.
    pub penetration: Real,
    /// World-space point.
    pub point: Vec2,
}
/// Contact returned by analytic narrow phase.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct Contact3 {
    /// Unit normal from A to B.
    pub normal: Vec3,
    /// Nonnegative overlap.
    pub penetration: Real,
    /// World-space point.
    pub point: Vec3,
}
/// Circle-circle narrow phase, including coincident centers.
#[must_use]
pub fn circle_circle(a: Vec2, ar: Real, b: Vec2, br: Real) -> Option<Contact2> {
    let d = b - a;
    let r = ar + br;
    let d2 = d.length_squared();
    if d2 > r * r {
        return None;
    }
    let dist = d2.sqrt();
    let n = if dist > ABS_EPSILON {
        d / dist
    } else {
        Vec2::X
    };
    Some(Contact2 {
        normal: n,
        penetration: r - dist,
        point: a + n * (ar - (r - dist) * 0.5),
    })
}
/// Sphere-sphere narrow phase, including coincident centers.
#[must_use]
pub fn sphere_sphere(a: Vec3, ar: Real, b: Vec3, br: Real) -> Option<Contact3> {
    let d = b - a;
    let r = ar + br;
    let d2 = d.length_squared();
    if d2 > r * r {
        return None;
    }
    let dist = d2.sqrt();
    let n = if dist > ABS_EPSILON {
        d / dist
    } else {
        Vec3::X
    };
    Some(Contact3 {
        normal: n,
        penetration: r - dist,
        point: a + n * (ar - (r - dist) * 0.5),
    })
}

/// Deterministic 2D broad phase. Entries are kept in stable-ID order; pair output is canonical.
#[derive(Default)]
pub struct BroadPhase2 {
    entries: Vec<(u64, Aabb2)>,
}
impl BroadPhase2 {
    /// Adds or replaces an entry.
    pub fn update(&mut self, id: u64, aabb: Aabb2) {
        if let Some(e) = self.entries.iter_mut().find(|e| e.0 == id) {
            e.1 = aabb;
        } else {
            self.entries.push((id, aabb));
            self.entries.sort_unstable_by_key(|e| e.0);
        }
    }
    /// Removes an entry.
    pub fn remove(&mut self, id: u64) {
        self.entries.retain(|e| e.0 != id);
    }
    /// Collects canonical overlapping pairs.
    #[must_use]
    pub fn pairs(&self) -> Vec<(u64, u64)> {
        let mut out = Vec::new();
        for (i, a) in self.entries.iter().enumerate() {
            for b in &self.entries[i + 1..] {
                if a.1.overlaps(b.1) {
                    out.push((a.0, b.0));
                }
            }
        }
        out
    }
}
/// Deterministic 3D broad phase.
#[derive(Default)]
pub struct BroadPhase3 {
    entries: Vec<(u64, Aabb3)>,
}
impl BroadPhase3 {
    /// Adds or replaces an entry.
    pub fn update(&mut self, id: u64, aabb: Aabb3) {
        if let Some(e) = self.entries.iter_mut().find(|e| e.0 == id) {
            e.1 = aabb;
        } else {
            self.entries.push((id, aabb));
            self.entries.sort_unstable_by_key(|e| e.0);
        }
    }
    /// Collects canonical overlapping pairs.
    #[must_use]
    pub fn pairs(&self) -> Vec<(u64, u64)> {
        let mut out = Vec::new();
        for (i, a) in self.entries.iter().enumerate() {
            for b in &self.entries[i + 1..] {
                if a.1.overlaps(b.1) {
                    out.push((a.0, b.0));
                }
            }
        }
        out
    }
}
#[cfg(test)]
mod tests {
    use super::*;
    #[test]
    fn touching_is_contact() {
        let c = circle_circle(Vec2::ZERO, 1.0, Vec2 { x: 2.0, y: 0.0 }, 1.0).unwrap();
        assert_eq!(c.penetration, 0.0);
    }
    #[test]
    fn coincident_is_finite() {
        let c = sphere_sphere(Vec3::ZERO, 1.0, Vec3::ZERO, 1.0).unwrap();
        assert!(c.normal.is_finite());
    }
    #[test]
    fn deterministic_pairs() {
        let mut b = BroadPhase2::default();
        for id in [9, 2, 5] {
            b.update(id, Aabb2::new(Vec2::ZERO, Vec2 { x: 1.0, y: 1.0 }).unwrap());
        }
        assert_eq!(b.pairs(), vec![(2, 5), (2, 9), (5, 9)]);
    }
}

pub mod tree;
pub use tree::*;

pub mod ccd;
pub mod filter;
pub use ccd::*;
pub use filter::*;

pub mod gjk;
pub use gjk::*;

pub mod narrow;
pub use narrow::*;
