//! SAT penetration, bounded 2D EPA, contact manifolds, and stable feature persistence.
use auralite_math::{ABS_EPSILON, CONTACT_SLOP, Real, Vec2, Vec3};
/// Penetration query output.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct Penetration2 {
    /// A-to-B unit normal.
    pub normal: Vec2,
    /// Nonnegative depth.
    pub depth: Real,
    /// Iterations.
    pub iterations: u16,
    /// Final improvement/error.
    pub residual: Real,
    /// Whether the primary algorithm converged.
    pub converged: bool,
}
/// SAT for two counter-clockwise convex polygons in world space.
#[must_use]
pub fn sat_polygon2(a: &[Vec2], b: &[Vec2]) -> Option<Penetration2> {
    if a.len() < 3 || b.len() < 3 {
        return None;
    }
    let ca = a.iter().copied().fold(Vec2::ZERO, |x, y| x + y) / (a.len() as Real);
    let cb = b.iter().copied().fold(Vec2::ZERO, |x, y| x + y) / (b.len() as Real);
    let mut best = Real::INFINITY;
    let mut normal = Vec2::X;
    for p in [a, b] {
        for i in 0..p.len() {
            let e = p[(i + 1) % p.len()] - p[i];
            if e.length_squared() <= ABS_EPSILON * ABS_EPSILON {
                continue;
            }
            let mut n = Vec2 { x: e.y, y: -e.x }.normalized_or(Vec2::X);
            let (min_a, max_a) = project2(a, n);
            let (min_b, max_b) = project2(b, n);
            let overlap = max_a.min(max_b) - min_a.max(min_b);
            if overlap < -CONTACT_SLOP {
                return None;
            }
            if overlap < best {
                if n.dot(cb - ca) < 0.0 {
                    n = -n;
                }
                best = overlap.max(0.0);
                normal = n;
            }
        }
    }
    best.is_finite().then_some(Penetration2 {
        normal,
        depth: best,
        iterations: (a.len() + b.len()) as u16,
        residual: 0.0,
        converged: true,
    })
}
fn project2(p: &[Vec2], n: Vec2) -> (Real, Real) {
    p.iter()
        .map(|x| x.dot(n))
        .fold((Real::INFINITY, Real::NEG_INFINITY), |(a, b), x| {
            (a.min(x), b.max(x))
        })
}
#[derive(Clone, Copy)]
struct EpaPoint {
    p: Vec2,
}
/// Bounded 2D EPA penetration using convex support functions. Falls back to the best finite edge at the cap.
#[must_use]
pub fn epa_penetration2(
    mut sa: impl FnMut(Vec2) -> Vec2,
    mut sb: impl FnMut(Vec2) -> Vec2,
    max_iterations: u16,
) -> Option<Penetration2> {
    let mut poly = Vec::new();
    for i in 0..3 {
        let q = i as Real * core::f64::consts::TAU as Real / 3.0;
        let d = Vec2 {
            x: q.cos(),
            y: q.sin(),
        };
        let a = sa(d);
        let b = sb(-d);
        if !a.is_finite() || !b.is_finite() {
            return None;
        }
        poly.push(EpaPoint { p: a - b });
    }
    if (poly[1].p - poly[0].p).cross(poly[2].p - poly[0].p) < 0.0 {
        poly.swap(1, 2);
    }
    let mut last = Real::INFINITY;
    for it in 1..=max_iterations.max(1) {
        let mut best = (Real::INFINITY, 0, Vec2::X);
        for i in 0..poly.len() {
            let j = (i + 1) % poly.len();
            let e = poly[j].p - poly[i].p;
            let mut n = Vec2 { x: e.y, y: -e.x }.normalized_or(Vec2::X);
            let mut d = n.dot(poly[i].p);
            if d < 0.0 {
                n = -n;
                d = -d;
            }
            if d < best.0 {
                best = (d, i, n);
            }
        }
        let a = sa(best.2);
        let b = sb(-best.2);
        let p = a - b;
        let distance = best.2.dot(p);
        let residual = (distance - best.0).abs();
        if residual <= ABS_EPSILON * (1.0 + distance.abs()) {
            return Some(Penetration2 {
                normal: best.2,
                depth: distance.max(0.0),
                iterations: it,
                residual,
                converged: true,
            });
        }
        if poly
            .iter()
            .any(|x| (x.p - p).length_squared() <= ABS_EPSILON * ABS_EPSILON)
        {
            return Some(Penetration2 {
                normal: best.2,
                depth: best.0.max(0.0),
                iterations: it,
                residual,
                converged: false,
            });
        }
        last = residual;
        poly.insert(best.1 + 1, EpaPoint { p });
    }
    let mut best = (Real::INFINITY, Vec2::X);
    for i in 0..poly.len() {
        let e = poly[(i + 1) % poly.len()].p - poly[i].p;
        let mut n = Vec2 { x: e.y, y: -e.x }.normalized_or(Vec2::X);
        let mut d = n.dot(poly[i].p);
        if d < 0.0 {
            d = -d;
            n = -n;
        }
        if d < best.0 {
            best = (d, n)
        }
    }
    Some(Penetration2 {
        normal: best.1,
        depth: best.0.max(0.0),
        iterations: max_iterations.max(1),
        residual: last,
        converged: false,
    })
}
/// Stable feature identifier derived from shape features, never pointer/order state.
#[derive(Clone, Copy, Debug, PartialEq, Eq, PartialOrd, Ord, Hash)]
pub struct FeatureId(pub u64);
/// Persistent 2D contact point.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct ManifoldPoint2 {
    /// Position.
    pub position: Vec2,
    /// Penetration.
    pub penetration: Real,
    /// Stable feature.
    pub feature: FeatureId,
    /// Cached normal impulse.
    pub normal_impulse: Real,
    /// Cached tangent impulse.
    pub tangent_impulse: Real,
}
/// Up to two native 2D contact points.
#[derive(Clone, Debug, Default, PartialEq)]
pub struct Manifold2 {
    /// A-to-B normal.
    pub normal: Vec2,
    /// Contact points sorted by feature.
    pub points: Vec<ManifoldPoint2>,
}
impl Manifold2 {
    /// Updates contacts while carrying warm-start impulses for matching feature IDs.
    pub fn update(&mut self, normal: Vec2, mut fresh: Vec<ManifoldPoint2>) {
        fresh.sort_by_key(|x| x.feature);
        fresh.dedup_by_key(|x| x.feature);
        fresh.truncate(2);
        for p in &mut fresh {
            if let Some(old) = self.points.iter().find(|x| x.feature == p.feature) {
                p.normal_impulse = old.normal_impulse;
                p.tangent_impulse = old.tangent_impulse;
            }
        }
        self.normal = normal.normalized_or(Vec2::X);
        self.points = fresh;
    }
}
/// Closest points between 3D segments, robust for parallel and degenerate inputs.
#[must_use]
pub fn segment_segment_closest3(
    p1: Vec3,
    q1: Vec3,
    p2: Vec3,
    q2: Vec3,
) -> (Vec3, Vec3, Real, Real) {
    let d1 = q1 - p1;
    let d2 = q2 - p2;
    let r = p1 - p2;
    let a = d1.dot(d1);
    let e = d2.dot(d2);
    let f = d2.dot(r);
    let (mut s, mut t);
    if a <= ABS_EPSILON && e <= ABS_EPSILON {
        return (p1, p2, 0.0, 0.0);
    }
    if a <= ABS_EPSILON {
        s = 0.0;
        t = (f / e).clamp(0.0, 1.0);
    } else {
        let c = d1.dot(r);
        if e <= ABS_EPSILON {
            t = 0.0;
            s = (-c / a).clamp(0.0, 1.0);
        } else {
            let b = d1.dot(d2);
            let den = a * e - b * b;
            s = if den.abs() > ABS_EPSILON {
                ((b * f - c * e) / den).clamp(0.0, 1.0)
            } else {
                0.0
            };
            t = (b * s + f) / e;
            if t < 0.0 {
                t = 0.0;
                s = (-c / a).clamp(0.0, 1.0)
            } else if t > 1.0 {
                t = 1.0;
                s = ((b - c) / a).clamp(0.0, 1.0)
            }
        }
    }
    (p1 + d1 * s, p2 + d2 * t, s, t)
}
#[cfg(test)]
mod tests {
    use super::*;
    fn square(c: Vec2, h: Real) -> Vec<Vec2> {
        vec![
            c + Vec2 { x: -h, y: -h },
            c + Vec2 { x: h, y: -h },
            c + Vec2 { x: h, y: h },
            c + Vec2 { x: -h, y: h },
        ]
    }
    #[test]
    fn epa_agrees_with_sat_for_boxes() {
        for i in 0..1000 {
            let x = i as Real * 0.0005;
            let a = square(Vec2::ZERO, 1.0);
            let b = square(Vec2 { x: 1.0 + x, y: 0.0 }, 1.0);
            let sat = sat_polygon2(&a, &b).unwrap();
            let epa = epa_penetration2(
                |d| {
                    *a.iter()
                        .max_by(|x, y| x.dot(d).total_cmp(&y.dot(d)))
                        .unwrap()
                },
                |d| {
                    *b.iter()
                        .max_by(|x, y| x.dot(d).total_cmp(&y.dot(d)))
                        .unwrap()
                },
                64,
            )
            .unwrap();
            assert!(
                (sat.depth - epa.depth).abs() < 2.0e-4,
                "sat={} epa={}",
                sat.depth,
                epa.depth
            );
        }
    }
    #[test]
    fn manifold_preserves_feature_impulses() {
        let mut m = Manifold2::default();
        m.update(
            Vec2::X,
            vec![ManifoldPoint2 {
                position: Vec2::ZERO,
                penetration: 1.0,
                feature: FeatureId(7),
                normal_impulse: 3.0,
                tangent_impulse: 2.0,
            }],
        );
        m.update(
            Vec2::X,
            vec![ManifoldPoint2 {
                position: Vec2 { x: 0.01, y: 0.0 },
                penetration: 0.9,
                feature: FeatureId(7),
                normal_impulse: 0.0,
                tangent_impulse: 0.0,
            }],
        );
        assert_eq!(m.points[0].normal_impulse, 3.0);
    }
    #[test]
    fn parallel_segments_are_finite() {
        let (a, b, _, _) = segment_segment_closest3(
            Vec3::ZERO,
            Vec3::X,
            Vec3 {
                x: 0.0,
                y: 1.0,
                z: 0.0,
            },
            Vec3 {
                x: 1.0,
                y: 1.0,
                z: 0.0,
            },
        );
        assert!(a.is_finite() && b.is_finite());
        assert!((a - b).length() - 1.0 < 1.0e-6);
    }
}
