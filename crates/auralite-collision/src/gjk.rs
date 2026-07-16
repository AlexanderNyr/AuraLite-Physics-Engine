//! Bounded GJK distance/witness solvers with degenerate-simplex reduction.
use auralite_math::{ABS_EPSILON, Real, Vec2, Vec3};
/// GJK termination reason.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum GjkStatus {
    /// Shapes overlap or touch within tolerance.
    Intersecting,
    /// A separating closest pair converged.
    Separated,
    /// Iteration cap reached; finite best estimate returned.
    IterationLimit,
    /// Support mapping returned non-finite data.
    InvalidSupport,
}
/// 2D distance and witness result.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct GjkResult2 {
    /// Euclidean separation.
    pub distance: Real,
    /// Witness on A.
    pub point_a: Vec2,
    /// Witness on B.
    pub point_b: Vec2,
    /// Unit A-to-B normal or +X fallback.
    pub normal: Vec2,
    /// Iterations.
    pub iterations: u16,
    /// Status.
    pub status: GjkStatus,
    /// Final squared-distance improvement.
    pub residual: Real,
}
/// 3D distance and witness result.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct GjkResult3 {
    /// Euclidean separation.
    pub distance: Real,
    /// Witness on A.
    pub point_a: Vec3,
    /// Witness on B.
    pub point_b: Vec3,
    /// Unit A-to-B normal.
    pub normal: Vec3,
    /// Iterations.
    pub iterations: u16,
    /// Status.
    pub status: GjkStatus,
    /// Final residual.
    pub residual: Real,
}
#[derive(Clone, Copy)]
struct S2 {
    p: Vec2,
    a: Vec2,
    b: Vec2,
}
#[derive(Clone, Copy)]
struct S3 {
    p: Vec3,
    a: Vec3,
    b: Vec3,
}
fn closest2(simplex: &mut Vec<S2>) -> (Vec2, Vec2, Vec2, bool) {
    match simplex.len() {
        1 => {
            let s = simplex[0];
            (s.p, s.a, s.b, false)
        }
        2 => {
            let x = simplex[0];
            let y = simplex[1];
            let d = y.p - x.p;
            let den = d.length_squared();
            if den <= ABS_EPSILON * ABS_EPSILON {
                simplex.truncate(1);
                return (x.p, x.a, x.b, false);
            }
            let t = ((-x.p).dot(d) / den).clamp(0.0, 1.0);
            if t <= 0.0 {
                simplex.truncate(1);
                return (x.p, x.a, x.b, false);
            }
            if t >= 1.0 {
                simplex[0] = y;
                simplex.truncate(1);
                return (y.p, y.a, y.b, false);
            }
            let p = x.p + d * t;
            (p, x.a + (y.a - x.a) * t, x.b + (y.b - x.b) * t, false)
        }
        _ => {
            let x = simplex[0];
            let y = simplex[1];
            let z = simplex[2];
            let area = (y.p - x.p).cross(z.p - x.p);
            if area.abs() <= ABS_EPSILON {
                let mut best = (
                    Real::INFINITY,
                    Vec::new(),
                    Vec2::ZERO,
                    Vec2::ZERO,
                    Vec2::ZERO,
                );
                for pair in [[x, y], [y, z], [z, x]] {
                    let mut q = pair.to_vec();
                    let (c, a, b, _) = closest2(&mut q);
                    if c.length_squared() < best.0 {
                        best = (c.length_squared(), q, a, b, c);
                    }
                }
                *simplex = best.1;
                return (best.4, best.2, best.3, false);
            }
            let u = y.p.cross(z.p) / area;
            let v = z.p.cross(x.p) / area;
            let w = x.p.cross(y.p) / area;
            if u >= -ABS_EPSILON && v >= -ABS_EPSILON && w >= -ABS_EPSILON {
                return (
                    Vec2::ZERO,
                    x.a * u + y.a * v + z.a * w,
                    x.b * u + y.b * v + z.b * w,
                    true,
                );
            }
            let mut best = (
                Real::INFINITY,
                Vec::new(),
                Vec2::ZERO,
                Vec2::ZERO,
                Vec2::ZERO,
            );
            for pair in [[x, y], [y, z], [z, x]] {
                let mut q = pair.to_vec();
                let (c, a, b, _) = closest2(&mut q);
                if c.length_squared() < best.0 {
                    best = (c.length_squared(), q, a, b, c);
                }
            }
            *simplex = best.1;
            (best.4, best.2, best.3, false)
        }
    }
}
/// Computes 2D convex distance using support functions. `support_a/b(d)` return each shape's farthest world point along `d`.
#[must_use]
pub fn gjk_distance2(
    mut support_a: impl FnMut(Vec2) -> Vec2,
    mut support_b: impl FnMut(Vec2) -> Vec2,
    max_iterations: u16,
) -> GjkResult2 {
    let mut direction = Vec2::X;
    let mut simplex: Vec<S2> = Vec::with_capacity(3);
    let mut previous = Real::INFINITY;
    let mut witness_a = Vec2::ZERO;
    let mut witness_b = Vec2::ZERO;
    let mut residual = Real::INFINITY;
    for iteration in 1..=max_iterations.max(1) {
        let a = support_a(direction);
        let b = support_b(-direction);
        if !a.is_finite() || !b.is_finite() {
            return GjkResult2 {
                distance: 0.0,
                point_a: Vec2::ZERO,
                point_b: Vec2::ZERO,
                normal: Vec2::X,
                iterations: iteration,
                status: GjkStatus::InvalidSupport,
                residual: Real::INFINITY,
            };
        }
        let s = S2 { p: a - b, a, b };
        if simplex
            .iter()
            .any(|x| (x.p - s.p).length_squared() <= ABS_EPSILON * ABS_EPSILON)
        {
            let delta = witness_b - witness_a;
            return GjkResult2 {
                distance: delta.length(),
                point_a: witness_a,
                point_b: witness_b,
                normal: delta.normalized_or(Vec2::X),
                iterations: iteration,
                status: GjkStatus::Separated,
                residual: 0.0,
            };
        }
        simplex.push(s);
        let (c, wa, wb, inside) = closest2(&mut simplex);
        witness_a = wa;
        witness_b = wb;
        let d2 = c.length_squared();
        residual = (previous - d2).abs();
        if inside || d2 <= ABS_EPSILON * ABS_EPSILON {
            return GjkResult2 {
                distance: 0.0,
                point_a: wa,
                point_b: wb,
                normal: (wb - wa).normalized_or(Vec2::X),
                iterations: iteration,
                status: GjkStatus::Intersecting,
                residual,
            };
        }
        if previous.is_finite() && residual <= ABS_EPSILON * ABS_EPSILON * (1.0 + previous.abs()) {
            let delta = wb - wa;
            return GjkResult2 {
                distance: d2.sqrt(),
                point_a: wa,
                point_b: wb,
                normal: delta.normalized_or(-c),
                iterations: iteration,
                status: GjkStatus::Separated,
                residual,
            };
        }
        previous = d2;
        direction = -c;
    }
    let delta = witness_b - witness_a;
    GjkResult2 {
        distance: delta.length(),
        point_a: witness_a,
        point_b: witness_b,
        normal: delta.normalized_or(Vec2::X),
        iterations: max_iterations.max(1),
        status: GjkStatus::IterationLimit,
        residual,
    }
}
fn tri_closest3(x: S3, y: S3, z: S3) -> (Vec3, Vec3, Vec3, Vec<S3>) {
    let ab = y.p - x.p;
    let ac = z.p - x.p;
    let ap = -x.p;
    let d1 = ab.dot(ap);
    let d2 = ac.dot(ap);
    if d1 <= 0.0 && d2 <= 0.0 {
        return (x.p, x.a, x.b, vec![x]);
    }
    let bp = -y.p;
    let d3 = ab.dot(bp);
    let d4 = ac.dot(bp);
    if d3 >= 0.0 && d4 <= d3 {
        return (y.p, y.a, y.b, vec![y]);
    }
    let vc = d1 * d4 - d3 * d2;
    if vc <= 0.0 && d1 >= 0.0 && d3 <= 0.0 {
        let t = d1 / (d1 - d3);
        return (
            x.p + ab * t,
            x.a + (y.a - x.a) * t,
            x.b + (y.b - x.b) * t,
            vec![x, y],
        );
    }
    let cp = -z.p;
    let d5 = ab.dot(cp);
    let d6 = ac.dot(cp);
    if d6 >= 0.0 && d5 <= d6 {
        return (z.p, z.a, z.b, vec![z]);
    }
    let vb = d5 * d2 - d1 * d6;
    if vb <= 0.0 && d2 >= 0.0 && d6 <= 0.0 {
        let t = d2 / (d2 - d6);
        return (
            x.p + ac * t,
            x.a + (z.a - x.a) * t,
            x.b + (z.b - x.b) * t,
            vec![x, z],
        );
    }
    let va = d3 * d6 - d5 * d4;
    if va <= 0.0 && (d4 - d3) >= 0.0 && (d5 - d6) >= 0.0 {
        let t = (d4 - d3) / ((d4 - d3) + (d5 - d6));
        return (
            y.p + (z.p - y.p) * t,
            y.a + (z.a - y.a) * t,
            y.b + (z.b - y.b) * t,
            vec![y, z],
        );
    }
    let den = 1.0 / (va + vb + vc);
    let v = vb * den;
    let w = vc * den;
    let u = 1.0 - v - w;
    (
        x.p * u + y.p * v + z.p * w,
        x.a * u + y.a * v + z.a * w,
        x.b * u + y.b * v + z.b * w,
        vec![x, y, z],
    )
}
fn closest3(s: &mut Vec<S3>) -> (Vec3, Vec3, Vec3, bool) {
    if s.len() == 1 {
        let x = s[0];
        return (x.p, x.a, x.b, false);
    }
    if s.len() == 2 {
        let x = s[0];
        let y = s[1];
        let d = y.p - x.p;
        let t = (-x.p).dot(d).clamp(0.0, d.length_squared())
            / d.length_squared().max(ABS_EPSILON * ABS_EPSILON);
        if t <= 0.0 {
            s.truncate(1);
            return (x.p, x.a, x.b, false);
        }
        if t >= 1.0 {
            s[0] = y;
            s.truncate(1);
            return (y.p, y.a, y.b, false);
        }
        return (
            x.p + d * t,
            x.a + (y.a - x.a) * t,
            x.b + (y.b - x.b) * t,
            false,
        );
    }
    if s.len() == 3 {
        let (p, a, b, q) = tri_closest3(s[0], s[1], s[2]);
        *s = q;
        return (p, a, b, false);
    }
    let q = [s[0], s[1], s[2], s[3]];
    let volume = (q[1].p - q[0].p).dot((q[2].p - q[0].p).cross(q[3].p - q[0].p));
    if volume.abs() > ABS_EPSILON {
        let mut same = true;
        for face in [[0, 1, 2, 3], [0, 3, 1, 2], [0, 2, 3, 1], [1, 3, 2, 0]] {
            let n = (q[face[1]].p - q[face[0]].p).cross(q[face[2]].p - q[face[0]].p);
            let so = n.dot(-q[face[0]].p);
            let sd = n.dot(q[face[3]].p - q[face[0]].p);
            if so * sd > ABS_EPSILON {
                same = false;
                break;
            }
        }
        if same {
            return (Vec3::ZERO, Vec3::ZERO, Vec3::ZERO, true);
        }
    }
    let mut best = (
        Real::INFINITY,
        Vec3::ZERO,
        Vec3::ZERO,
        Vec3::ZERO,
        Vec::new(),
    );
    for f in [[0, 1, 2], [0, 3, 1], [0, 2, 3], [1, 3, 2]] {
        let (p, a, b, v) = tri_closest3(q[f[0]], q[f[1]], q[f[2]]);
        if p.length_squared() < best.0 {
            best = (p.length_squared(), p, a, b, v)
        }
    }
    *s = best.4;
    (best.1, best.2, best.3, false)
}
/// Computes bounded 3D convex distance and witnesses.
#[must_use]
pub fn gjk_distance3(
    mut sa: impl FnMut(Vec3) -> Vec3,
    mut sb: impl FnMut(Vec3) -> Vec3,
    max_iterations: u16,
) -> GjkResult3 {
    let mut d = Vec3::X;
    let mut simplex: Vec<S3> = Vec::with_capacity(4);
    let mut prev = Real::INFINITY;
    let (mut wa, mut wb) = (Vec3::ZERO, Vec3::ZERO);
    let mut residual = Real::INFINITY;
    for it in 1..=max_iterations.max(1) {
        let a = sa(d);
        let b = sb(-d);
        if !a.is_finite() || !b.is_finite() {
            return GjkResult3 {
                distance: 0.0,
                point_a: wa,
                point_b: wb,
                normal: Vec3::X,
                iterations: it,
                status: GjkStatus::InvalidSupport,
                residual,
            };
        }
        let x = S3 { p: a - b, a, b };
        if simplex
            .iter()
            .any(|s| (s.p - x.p).length_squared() <= ABS_EPSILON * ABS_EPSILON)
        {
            let delta = wb - wa;
            return GjkResult3 {
                distance: delta.length(),
                point_a: wa,
                point_b: wb,
                normal: delta.normalized_or(Vec3::X),
                iterations: it,
                status: GjkStatus::Separated,
                residual: 0.0,
            };
        }
        simplex.push(x);
        let (c, a, b, inside) = closest3(&mut simplex);
        wa = a;
        wb = b;
        let d2 = c.length_squared();
        residual = (prev - d2).abs();
        if inside || d2 <= ABS_EPSILON * ABS_EPSILON {
            return GjkResult3 {
                distance: 0.0,
                point_a: wa,
                point_b: wb,
                normal: (wb - wa).normalized_or(Vec3::X),
                iterations: it,
                status: GjkStatus::Intersecting,
                residual,
            };
        }
        if prev.is_finite() && residual <= ABS_EPSILON * ABS_EPSILON * (1.0 + prev.abs()) {
            let delta = wb - wa;
            return GjkResult3 {
                distance: d2.sqrt(),
                point_a: wa,
                point_b: wb,
                normal: delta.normalized_or(-c),
                iterations: it,
                status: GjkStatus::Separated,
                residual,
            };
        }
        prev = d2;
        d = -c;
    }
    let delta = wb - wa;
    GjkResult3 {
        distance: delta.length(),
        point_a: wa,
        point_b: wb,
        normal: delta.normalized_or(Vec3::X),
        iterations: max_iterations.max(1),
        status: GjkStatus::IterationLimit,
        residual,
    }
}
#[cfg(test)]
mod tests {
    use super::*;
    #[test]
    fn circles_match_analytic_distance() {
        for i in 0..1000 {
            let x = 2.1 + i as Real * 0.01;
            let r = gjk_distance2(
                |d| d.normalized_or(Vec2::X),
                |d| Vec2 { x, y: 0.0 } + d.normalized_or(Vec2::X),
                32,
            );
            assert!(
                (r.distance - (x - 2.0)).abs() < 2.0e-4,
                "x={x} got={}",
                r.distance
            );
            assert_ne!(r.status, GjkStatus::IterationLimit);
        }
    }
    #[test]
    fn spheres_overlap_and_degenerate_support_is_bounded() {
        let r = gjk_distance3(
            |d| d.normalized_or(Vec3::X),
            |d| {
                Vec3 {
                    x: 1.0,
                    y: 0.0,
                    z: 0.0,
                } + d.normalized_or(Vec3::X)
            },
            32,
        );
        assert_eq!(r.status, GjkStatus::Intersecting, "{r:?}");
        let p = gjk_distance3(
            |_| Vec3::ZERO,
            |_| Vec3 {
                x: 2.0,
                y: 0.0,
                z: 0.0,
            },
            8,
        );
        assert!(p.distance.is_finite());
    }
}
