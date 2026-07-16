//! Bounded analytic and conservative continuous collision detection.
use auralite_math::{ABS_EPSILON, Real, Vec2, Vec3};
/// Time-of-impact diagnostic.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct Toi {
    /// Normalized time in `[0,max_time]`.
    pub time: Real,
    /// Contact normal from A to B.
    pub normal3: Vec3,
    /// Iterations used.
    pub iterations: u16,
    /// Final nonnegative separation residual.
    pub residual: Real,
    /// Whether the algorithm converged.
    pub converged: bool,
}
/// Exact swept circle-circle TOI for constant linear velocities.
#[must_use]
pub fn circle_circle_toi(
    pa: Vec2,
    va: Vec2,
    ra: Real,
    pb: Vec2,
    vb: Vec2,
    rb: Real,
    max_time: Real,
) -> Option<Toi> {
    let p = pb - pa;
    let v = vb - va;
    let r = ra + rb;
    let c = p.length_squared() - r * r;
    if c <= 0.0 {
        return Some(Toi {
            time: 0.0,
            normal3: Vec3 {
                x: p.normalized_or(Vec2::X).x,
                y: p.normalized_or(Vec2::X).y,
                z: 0.0,
            },
            iterations: 0,
            residual: 0.0,
            converged: true,
        });
    }
    let a = v.length_squared();
    if a <= ABS_EPSILON {
        return None;
    }
    let b = p.dot(v);
    if b >= 0.0 {
        return None;
    }
    let d = b * b - a * c;
    if d < 0.0 {
        return None;
    }
    let t = (-b - d.sqrt()) / a;
    if t < 0.0 || t > max_time {
        return None;
    }
    let n = (p + v * t).normalized_or(Vec2::X);
    Some(Toi {
        time: t,
        normal3: Vec3 {
            x: n.x,
            y: n.y,
            z: 0.0,
        },
        iterations: 1,
        residual: 0.0,
        converged: true,
    })
}
/// Exact swept sphere-sphere TOI for constant linear velocities.
#[must_use]
pub fn sphere_sphere_toi(
    pa: Vec3,
    va: Vec3,
    ra: Real,
    pb: Vec3,
    vb: Vec3,
    rb: Real,
    max_time: Real,
) -> Option<Toi> {
    let p = pb - pa;
    let v = vb - va;
    let r = ra + rb;
    let c = p.length_squared() - r * r;
    if c <= 0.0 {
        return Some(Toi {
            time: 0.0,
            normal3: p.normalized_or(Vec3::X),
            iterations: 0,
            residual: 0.0,
            converged: true,
        });
    }
    let a = v.length_squared();
    if a <= ABS_EPSILON {
        return None;
    }
    let b = p.dot(v);
    if b >= 0.0 {
        return None;
    }
    let d = b * b - a * c;
    if d < 0.0 {
        return None;
    }
    let t = (-b - d.sqrt()) / a;
    if !(0.0..=max_time).contains(&t) {
        return None;
    }
    Some(Toi {
        time: t,
        normal3: (p + v * t).normalized_or(Vec3::X),
        iterations: 1,
        residual: 0.0,
        converged: true,
    })
}
/// Swept circle against an infinite line/half-space boundary. Prevents tunneling through thin planar walls.
#[must_use]
pub fn circle_plane_toi(
    center: Vec2,
    velocity: Vec2,
    radius: Real,
    normal: Vec2,
    offset: Real,
    max_time: Real,
) -> Option<Toi> {
    let n = normal.normalized_or(Vec2::X);
    let separation = n.dot(center) - offset - radius;
    if separation <= 0.0 {
        return Some(Toi {
            time: 0.0,
            normal3: Vec3 {
                x: n.x,
                y: n.y,
                z: 0.0,
            },
            iterations: 0,
            residual: 0.0,
            converged: true,
        });
    }
    let speed = -n.dot(velocity);
    if speed <= ABS_EPSILON {
        return None;
    }
    let t = separation / speed;
    if t <= max_time {
        Some(Toi {
            time: t,
            normal3: Vec3 {
                x: n.x,
                y: n.y,
                z: 0.0,
            },
            iterations: 1,
            residual: 0.0,
            converged: true,
        })
    } else {
        None
    }
}
#[cfg(test)]
mod tests {
    use super::*;
    #[test]
    fn velocity_sweep_never_tunnels_plane() {
        for speed in [10.0, 100.0, 1_000.0, 10_000.0] {
            let hit = circle_plane_toi(
                Vec2 { x: 1.0, y: 0.0 },
                Vec2 { x: -speed, y: 0.0 },
                0.01,
                Vec2::X,
                0.0,
                1.0 / 60.0,
            );
            if speed * 1.0 / 60.0 >= 0.99 {
                assert!(hit.is_some(), "speed={speed}");
            }
        }
    }
    #[test]
    fn sphere_toi_analytic() {
        let h = sphere_sphere_toi(
            Vec3::ZERO,
            Vec3::ZERO,
            1.0,
            Vec3 {
                x: 10.0,
                y: 0.0,
                z: 0.0,
            },
            Vec3 {
                x: -2.0,
                y: 0.0,
                z: 0.0,
            },
            1.0,
            10.0,
        )
        .unwrap();
        assert!((h.time - 4.0).abs() < 1.0e-6);
    }
}
