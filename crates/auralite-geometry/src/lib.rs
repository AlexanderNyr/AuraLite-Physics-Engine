//! Validated, dimension-specific geometry and analytic mass properties.
#![forbid(unsafe_code)]
use auralite_math::{
    ABS_EPSILON, Aabb2, Aabb3, MathError, Ray3, Real, Transform2, Transform3, Vec2, Vec3,
};

/// Geometry construction or query error.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum GeometryError {
    /// A scalar/vector was invalid.
    InvalidInput,
    /// A polygon was non-convex, clockwise, or degenerate.
    InvalidPolygon,
}
impl From<MathError> for GeometryError {
    fn from(_: MathError) -> Self {
        Self::InvalidInput
    }
}
/// 2D mass, center of mass, and scalar moment of inertia about the center.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct MassProperties2 {
    /// Total mass.
    pub mass: Real,
    /// Local center of mass.
    pub center: Vec2,
    /// Rotational inertia.
    pub inertia: Real,
}
/// 3D mass, center, and diagonal inertia for axis-aligned symmetric primitives.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct MassProperties3 {
    /// Total mass.
    pub mass: Real,
    /// Local center.
    pub center: Vec3,
    /// Diagonal inertia tensor.
    pub inertia_diagonal: Vec3,
}
fn valid_positive(v: Real) -> bool {
    v.is_finite() && v > 0.0
}

/// Validated 2D axis-aligned box in local space.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct Box2 {
    half: Vec2,
}
impl Box2 {
    /// Constructs from positive finite half extents.
    pub fn new(half: Vec2) -> Result<Self, GeometryError> {
        if half.is_finite() && valid_positive(half.x) && valid_positive(half.y) {
            Ok(Self { half })
        } else {
            Err(GeometryError::InvalidInput)
        }
    }
    /// Half extents.
    #[must_use]
    pub const fn half_extents(self) -> Vec2 {
        self.half
    }
    /// Convex support point in local space.
    #[must_use]
    pub fn support(self, d: Vec2) -> Vec2 {
        Vec2 {
            x: if d.x >= 0.0 {
                self.half.x
            } else {
                -self.half.x
            },
            y: if d.y >= 0.0 {
                self.half.y
            } else {
                -self.half.y
            },
        }
    }
    /// Inclusive point containment.
    #[must_use]
    pub fn contains(self, p: Vec2) -> bool {
        p.is_finite() && p.x.abs() <= self.half.x && p.y.abs() <= self.half.y
    }
    /// Closest local point.
    #[must_use]
    pub fn closest_point(self, p: Vec2) -> Vec2 {
        Vec2 {
            x: p.x.clamp(-self.half.x, self.half.x),
            y: p.y.clamp(-self.half.y, self.half.y),
        }
    }
    /// Local bounds.
    #[must_use]
    pub fn aabb(self) -> Aabb2 {
        Aabb2::new(-self.half, self.half).expect("validated extents")
    }
    /// Analytic area-density mass properties.
    pub fn mass_properties(self, density: Real) -> Result<MassProperties2, GeometryError> {
        if !valid_positive(density) {
            return Err(GeometryError::InvalidInput);
        }
        let w = 2.0 * self.half.x;
        let h = 2.0 * self.half.y;
        let mass = w * h * density;
        Ok(MassProperties2 {
            mass,
            center: Vec2::ZERO,
            inertia: mass * (w * w + h * h) / 12.0,
        })
    }
    /// Validated component-wise scaling.
    pub fn scaled(self, scale: Vec2) -> Result<Self, GeometryError> {
        Self::new(Vec2 {
            x: self.half.x * scale.x,
            y: self.half.y * scale.y,
        })
    }
}
/// Validated 3D axis-aligned box in local space.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct Box3 {
    /// Half extents.
    pub half: Vec3,
}
impl Box3 {
    /// Constructs from positive finite half extents.
    pub fn new(half: Vec3) -> Result<Self, GeometryError> {
        if half.is_finite()
            && valid_positive(half.x)
            && valid_positive(half.y)
            && valid_positive(half.z)
        {
            Ok(Self { half })
        } else {
            Err(GeometryError::InvalidInput)
        }
    }
    /// Half extents.
    #[must_use]
    pub const fn half_extents(self) -> Vec3 {
        self.half
    }
    /// Convex support point.
    #[must_use]
    pub fn support(self, d: Vec3) -> Vec3 {
        Vec3 {
            x: if d.x >= 0.0 {
                self.half.x
            } else {
                -self.half.x
            },
            y: if d.y >= 0.0 {
                self.half.y
            } else {
                -self.half.y
            },
            z: if d.z >= 0.0 {
                self.half.z
            } else {
                -self.half.z
            },
        }
    }
    /// Inclusive containment.
    #[must_use]
    pub fn contains(self, p: Vec3) -> bool {
        p.is_finite()
            && p.x.abs() <= self.half.x
            && p.y.abs() <= self.half.y
            && p.z.abs() <= self.half.z
    }
    /// Closest local point.
    #[must_use]
    pub fn closest_point(self, p: Vec3) -> Vec3 {
        Vec3 {
            x: p.x.clamp(-self.half.x, self.half.x),
            y: p.y.clamp(-self.half.y, self.half.y),
            z: p.z.clamp(-self.half.z, self.half.z),
        }
    }
    /// Local bounds.
    #[must_use]
    pub fn aabb(self) -> Aabb3 {
        Aabb3::new(-self.half, self.half).expect("validated extents")
    }
    /// Analytic volume-density mass properties.
    pub fn mass_properties(self, density: Real) -> Result<MassProperties3, GeometryError> {
        if !valid_positive(density) {
            return Err(GeometryError::InvalidInput);
        }
        let x = 2.0 * self.half.x;
        let y = 2.0 * self.half.y;
        let z = 2.0 * self.half.z;
        let m = x * y * z * density;
        Ok(MassProperties3 {
            mass: m,
            center: Vec3::ZERO,
            inertia_diagonal: Vec3 {
                x: m * (y * y + z * z) / 12.0,
                y: m * (x * x + z * z) / 12.0,
                z: m * (x * x + y * y) / 12.0,
            },
        })
    }
    /// Slab ray intersection interval, including rays originating inside.
    #[must_use]
    pub fn ray_interval(self, ray: Ray3) -> Option<(Real, Real)> {
        let mut lo: Real = 0.0;
        let mut hi = Real::INFINITY;
        for (o, d, h) in [
            (ray.origin.x, ray.direction.x, self.half.x),
            (ray.origin.y, ray.direction.y, self.half.y),
            (ray.origin.z, ray.direction.z, self.half.z),
        ] {
            if d.abs() <= ABS_EPSILON {
                if o < -h || o > h {
                    return None;
                }
            } else {
                let mut a = (-h - o) / d;
                let mut b = (h - o) / d;
                if a > b {
                    core::mem::swap(&mut a, &mut b);
                }
                lo = lo.max(a);
                hi = hi.min(b);
                if lo > hi {
                    return None;
                }
            }
        }
        Some((lo, hi))
    }
    /// Exact shape ray intersection with true outward normal.
    #[must_use]
    pub fn ray_intersection(self, ray: Ray3) -> Option<(Real, Vec3)> {
        let (lo, hi) = self.ray_interval(ray)?;
        if hi < 0.0 || lo > hi {
            return None;
        }
        let t = if lo >= 0.0 { lo } else { hi };
        let p = ray.origin + ray.direction * t;
        let dx = self.half.x - p.x.abs();
        let dy = self.half.y - p.y.abs();
        let dz = self.half.z - p.z.abs();
        let normal = if dx <= dy && dx <= dz {
            Vec3 {
                x: p.x.signum(),
                y: 0.0,
                z: 0.0,
            }
        } else if dy <= dz {
            Vec3 {
                x: 0.0,
                y: p.y.signum(),
                z: 0.0,
            }
        } else {
            Vec3 {
                x: 0.0,
                y: 0.0,
                z: p.z.signum(),
            }
        };
        Some((t, normal))
    }
    /// Validated component-wise scaling.
    pub fn scaled(self, scale: Vec3) -> Result<Self, GeometryError> {
        Self::new(Vec3 {
            x: self.half.x * scale.x,
            y: self.half.y * scale.y,
            z: self.half.z * scale.z,
        })
    }
}

/// Validated 2D capsule aligned with local Y.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct Capsule2 {
    /// Radius.
    pub radius: Real,
    /// Half distance between cap centers.
    pub half_height: Real,
}
impl Capsule2 {
    /// Constructs from positive radius and nonnegative half-height.
    pub fn new(radius: Real, half_height: Real) -> Result<Self, GeometryError> {
        if valid_positive(radius) && half_height.is_finite() && half_height >= 0.0 {
            Ok(Self {
                radius,
                half_height,
            })
        } else {
            Err(GeometryError::InvalidInput)
        }
    }
    /// Support point.
    #[must_use]
    pub fn support(self, d: Vec2) -> Vec2 {
        let n = d.normalized_or(Vec2::X);
        Vec2 {
            x: 0.0,
            y: if d.y >= 0.0 {
                self.half_height
            } else {
                -self.half_height
            },
        } + n * self.radius
    }
    /// Local bounds.
    #[must_use]
    pub fn aabb(self) -> Aabb2 {
        Aabb2::new(
            Vec2 {
                x: -self.radius,
                y: -self.half_height - self.radius,
            },
            Vec2 {
                x: self.radius,
                y: self.half_height + self.radius,
            },
        )
        .expect("valid capsule")
    }
}
/// Validated 3D capsule aligned with local Y.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct Capsule3 {
    /// Radius.
    pub radius: Real,
    /// Half distance between cap centers.
    pub half_height: Real,
}
impl Capsule3 {
    /// Constructs from positive radius and nonnegative half-height.
    pub fn new(radius: Real, half_height: Real) -> Result<Self, GeometryError> {
        if valid_positive(radius) && half_height.is_finite() && half_height >= 0.0 {
            Ok(Self {
                radius,
                half_height,
            })
        } else {
            Err(GeometryError::InvalidInput)
        }
    }
    /// Support point.
    #[must_use]
    pub fn support(self, d: Vec3) -> Vec3 {
        let n = d.normalized_or(Vec3::X);
        Vec3 {
            x: 0.0,
            y: if d.y >= 0.0 {
                self.half_height
            } else {
                -self.half_height
            },
            z: 0.0,
        } + n * self.radius
    }
    /// Local bounds.
    #[must_use]
    pub fn aabb(self) -> Aabb3 {
        Aabb3::new(
            Vec3 {
                x: -self.radius,
                y: -self.half_height - self.radius,
                z: -self.radius,
            },
            Vec3 {
                x: self.radius,
                y: self.half_height + self.radius,
                z: self.radius,
            },
        )
        .expect("valid capsule")
    }
}

/// Counter-clockwise strictly convex 2D polygon.
#[derive(Clone, Debug, PartialEq)]
pub struct ConvexPolygon {
    vertices: Vec<Vec2>,
}
impl ConvexPolygon {
    /// Validates 3 or more finite, CCW, strictly convex vertices.
    pub fn new(vertices: Vec<Vec2>) -> Result<Self, GeometryError> {
        if vertices.len() < 3 || vertices.iter().any(|v| !v.is_finite()) {
            return Err(GeometryError::InvalidPolygon);
        }
        let n = vertices.len();
        let mut area = 0.0;
        for i in 0..n {
            area += vertices[i].cross(vertices[(i + 1) % n]);
            let e0 = vertices[(i + 1) % n] - vertices[i];
            let e1 = vertices[(i + 2) % n] - vertices[(i + 1) % n];
            if e0.cross(e1) <= ABS_EPSILON {
                return Err(GeometryError::InvalidPolygon);
            }
        }
        if area <= ABS_EPSILON {
            return Err(GeometryError::InvalidPolygon);
        }
        Ok(Self { vertices })
    }
    /// Vertices in canonical input order.
    #[must_use]
    pub fn vertices(&self) -> &[Vec2] {
        &self.vertices
    }
    /// Brute-force support mapping with stable lowest-index tie break.
    #[must_use]
    pub fn support(&self, d: Vec2) -> Vec2 {
        let mut best = self.vertices[0];
        let mut dot = best.dot(d);
        for &v in &self.vertices[1..] {
            let x = v.dot(d);
            if x > dot {
                best = v;
                dot = x;
            }
        }
        best
    }
    /// Half-space point containment.
    #[must_use]
    pub fn contains(&self, p: Vec2) -> bool {
        p.is_finite()
            && (0..self.vertices.len()).all(|i| {
                (self.vertices[(i + 1) % self.vertices.len()] - self.vertices[i])
                    .cross(p - self.vertices[i])
                    >= -ABS_EPSILON
            })
    }
    /// Shoelace area, centroid and polygon inertia.
    pub fn mass_properties(&self, density: Real) -> Result<MassProperties2, GeometryError> {
        if !valid_positive(density) {
            return Err(GeometryError::InvalidInput);
        }
        let mut twice_area = 0.0;
        let mut centroid = Vec2::ZERO;
        let mut inertia_origin = 0.0;
        for i in 0..self.vertices.len() {
            let a = self.vertices[i];
            let b = self.vertices[(i + 1) % self.vertices.len()];
            let c = a.cross(b);
            twice_area += c;
            centroid += (a + b) * c;
            inertia_origin += c * (a.dot(a) + a.dot(b) + b.dot(b));
        }
        let area = twice_area * 0.5;
        centroid /= 3.0 * twice_area;
        let mass = area * density;
        let inertia = inertia_origin * density / 12.0 - mass * centroid.length_squared();
        Ok(MassProperties2 {
            mass,
            center: centroid,
            inertia: inertia.max(0.0),
        })
    }
}

/// Computes a world AABB for a transformed 2D box.
#[must_use]
pub fn transformed_box_aabb2(shape: Box2, t: Transform2) -> Aabb2 {
    let h = shape.half_extents();
    let m = t.rotation.matrix();
    let e = Vec2 {
        x: m.x.x.abs() * h.x + m.y.x.abs() * h.y,
        y: m.x.y.abs() * h.x + m.y.y.abs() * h.y,
    };
    Aabb2::new(t.translation - e, t.translation + e).expect("finite transform")
}
/// Computes a world AABB for a transformed 3D box.
#[must_use]
pub fn transformed_box_aabb3(shape: Box3, t: Transform3) -> Aabb3 {
    let h = shape.half_extents();
    let m = t.rotation.matrix();
    let e = Vec3 {
        x: m.x.x.abs() * h.x + m.y.x.abs() * h.y + m.z.x.abs() * h.z,
        y: m.x.y.abs() * h.x + m.y.y.abs() * h.y + m.z.y.abs() * h.z,
        z: m.x.z.abs() * h.x + m.y.z.abs() * h.y + m.z.z.abs() * h.z,
    };
    Aabb3::new(t.translation - e, t.translation + e).expect("finite transform")
}

#[cfg(test)]
mod tests {
    use super::*;
    #[test]
    fn box_mass_matches_reference() {
        let b = Box3::new(Vec3 {
            x: 1.0,
            y: 2.0,
            z: 3.0,
        })
        .unwrap();
        let p = b.mass_properties(2.0).unwrap();
        assert_eq!(p.mass, 96.0);
        assert!((p.inertia_diagonal.x - 416.0).abs() < 1.0e-4);
    }
    #[test]
    fn support_matches_all_vertices() {
        let p = ConvexPolygon::new(vec![
            Vec2 { x: -1.0, y: -1.0 },
            Vec2 { x: 1.0, y: -1.0 },
            Vec2 { x: 1.0, y: 1.0 },
            Vec2 { x: -1.0, y: 1.0 },
        ])
        .unwrap();
        for i in 0..100 {
            let a = i as Real * 0.061;
            let d = Vec2 {
                x: a.cos(),
                y: a.sin(),
            };
            let s = p.support(d);
            assert!(p.vertices().iter().all(|v| s.dot(d) >= v.dot(d)));
        }
    }
    #[test]
    fn polygon_rejects_degeneracy() {
        assert!(ConvexPolygon::new(vec![Vec2::ZERO, Vec2::X, Vec2 { x: 2.0, y: 0.0 }]).is_err());
    }
    #[test]
    fn ray_box_boundary() {
        let b = Box3::new(Vec3 {
            x: 1.0,
            y: 1.0,
            z: 1.0,
        })
        .unwrap();
        let r = Ray3::new(
            Vec3 {
                x: -2.0,
                y: 1.0,
                z: 0.0,
            },
            Vec3::X,
        )
        .unwrap();
        assert_eq!(b.ray_interval(r), Some((1.0, 3.0)));
    }
    #[test]
    fn scaling_is_validated() {
        let b = Box2::new(Vec2 { x: 1.0, y: 2.0 }).unwrap();
        assert!(b.scaled(Vec2 { x: -1.0, y: 1.0 }).is_err());
    }
    #[test]
    fn transformed_bounds_contain_corners() {
        let b = Box2::new(Vec2 { x: 1.0, y: 2.0 }).unwrap();
        let t = Transform2::new(
            Vec2 { x: 3.0, y: 4.0 },
            auralite_math::Rot2::from_radians(0.7).unwrap(),
        )
        .unwrap();
        let a = transformed_box_aabb2(b, t);
        for x in [-1.0, 1.0] {
            for y in [-2.0, 2.0] {
                let p = t.transform_point(Vec2 { x, y });
                assert!(
                    p.x >= a.min.x - ABS_EPSILON
                        && p.x <= a.max.x + ABS_EPSILON
                        && p.y >= a.min.y - ABS_EPSILON
                        && p.y <= a.max.y + ABS_EPSILON
                );
            }
        }
    }
}

pub mod advanced;
pub use advanced::*;

impl Box2 {
    /// Radius of the local origin-centered bounding circle.
    #[must_use]
    pub fn bounding_radius(self) -> Real {
        self.half.length()
    }
    /// Slab ray intersection interval.
    #[must_use]
    pub fn ray_interval(self, ray: auralite_math::Ray2) -> Option<(Real, Real)> {
        let mut lo: Real = 0.0;
        let mut hi = Real::INFINITY;
        for (o, d, h) in [
            (ray.origin.x, ray.direction.x, self.half.x),
            (ray.origin.y, ray.direction.y, self.half.y),
        ] {
            if d.abs() <= ABS_EPSILON {
                if o < -h || o > h {
                    return None;
                }
            } else {
                let mut a = (-h - o) / d;
                let mut b = (h - o) / d;
                if a > b {
                    core::mem::swap(&mut a, &mut b);
                }
                lo = lo.max(a);
                hi = hi.min(b);
                if lo > hi {
                    return None;
                }
            }
        }
        Some((lo, hi))
    }
    /// Exact 2D box ray intersection and outward normal.
    #[must_use]
    pub fn ray_intersection(self, ray: auralite_math::Ray2) -> Option<(Real, Vec2)> {
        let (lo, hi) = self.ray_interval(ray)?;
        if hi < 0.0 || lo > hi {
            return None;
        }
        let t = if lo >= 0.0 { lo } else { hi };
        let p = ray.origin + ray.direction * t;
        let dx = self.half.x - p.x.abs();
        let dy = self.half.y - p.y.abs();
        let normal = if dx <= dy {
            Vec2 {
                x: p.x.signum(),
                y: 0.0,
            }
        } else {
            Vec2 {
                x: 0.0,
                y: p.y.signum(),
            }
        };
        Some((t, normal))
    }
}
impl Box3 {
    /// Radius of the local origin-centered bounding sphere.
    #[must_use]
    pub fn bounding_radius(self) -> Real {
        self.half.length()
    }
    /// Volume of the box.
    #[must_use]
    pub fn volume(self) -> Real {
        8.0 * self.half.x * self.half.y * self.half.z
    }
}
impl Capsule2 {
    /// Bounding-circle radius about the local origin.
    #[must_use]
    pub fn bounding_radius(self) -> Real {
        self.half_height + self.radius
    }
    /// Inclusive containment.
    #[must_use]
    pub fn contains(self, p: Vec2) -> bool {
        let y = p.y.clamp(-self.half_height, self.half_height);
        (p - Vec2 { x: 0.0, y }).length_squared() <= self.radius * self.radius
    }
    /// Closest point in/on the capsule.
    #[must_use]
    pub fn closest_point(self, p: Vec2) -> Vec2 {
        if self.contains(p) {
            return p;
        }
        let c = Vec2 {
            x: 0.0,
            y: p.y.clamp(-self.half_height, self.half_height),
        };
        c + (p - c).normalized_or(Vec2::X) * self.radius
    }
    /// Conservative exact ray hit obtained from the central box and cap circles.
    #[must_use]
    pub fn ray_t(self, r: auralite_math::Ray2) -> Option<Real> {
        let box_hit = Box2::new(Vec2 {
            x: self.radius,
            y: self.half_height,
        })
        .ok()
        .and_then(|b| b.ray_interval(r).map(|x| x.0));
        let caps = [
            Vec2 {
                x: 0.0,
                y: -self.half_height,
            },
            Vec2 {
                x: 0.0,
                y: self.half_height,
            },
        ];
        caps.into_iter()
            .filter_map(|c| {
                let m = r.origin - c;
                let b = m.dot(r.direction);
                let q = m.length_squared() - self.radius * self.radius;
                let disc = b * b - q;
                if disc < 0.0 {
                    None
                } else {
                    Some((-b - disc.sqrt()).max(0.0))
                }
            })
            .chain(box_hit)
            .min_by(Real::total_cmp)
    }
    /// Exact 2D capsule ray intersection and outward normal.
    #[must_use]
    pub fn ray_intersection(self, r: auralite_math::Ray2) -> Option<(Real, Vec2)> {
        let t = self.ray_t(r)?;
        let p = r.origin + r.direction * t;
        let cy = p.y.clamp(-self.half_height, self.half_height);
        let center = Vec2 { x: 0.0, y: cy };
        Some((t, (p - center).normalized_or(Vec2::Y)))
    }
    /// Area-density mass properties using rectangle plus full disk decomposition.
    pub fn mass_properties(self, density: Real) -> Result<MassProperties2, GeometryError> {
        if !valid_positive(density) {
            return Err(GeometryError::InvalidInput);
        }
        let rectangle_mass = 4.0 * self.radius * self.half_height * density;
        let disk_mass = core::f64::consts::PI as Real * self.radius * self.radius * density;
        let mass = rectangle_mass + disk_mass;
        let rectangle_i = rectangle_mass
            * ((2.0 * self.radius).powi(2) + (2.0 * self.half_height).powi(2))
            / 12.0;
        let disk_i = 0.5 * disk_mass * self.radius * self.radius
            + disk_mass * self.half_height * self.half_height;
        Ok(MassProperties2 {
            mass,
            center: Vec2::ZERO,
            inertia: rectangle_i + disk_i,
        })
    }
    /// Uniform scaling.
    pub fn scaled(self, s: Real) -> Result<Self, GeometryError> {
        Self::new(self.radius * s, self.half_height * s)
    }
}
impl Capsule3 {
    /// Bounding-sphere radius.
    #[must_use]
    pub fn bounding_radius(self) -> Real {
        self.half_height + self.radius
    }
    /// Inclusive containment.
    #[must_use]
    pub fn contains(self, p: Vec3) -> bool {
        let y = p.y.clamp(-self.half_height, self.half_height);
        (p - Vec3 { x: 0.0, y, z: 0.0 }).length_squared() <= self.radius * self.radius
    }
    /// Closest point in/on the capsule.
    #[must_use]
    pub fn closest_point(self, p: Vec3) -> Vec3 {
        if self.contains(p) {
            return p;
        }
        let c = Vec3 {
            x: 0.0,
            y: p.y.clamp(-self.half_height, self.half_height),
            z: 0.0,
        };
        c + (p - c).normalized_or(Vec3::X) * self.radius
    }
    /// Exact local ray hit against the cylindrical body and spherical caps.
    #[must_use]
    pub fn ray_t(self, r: auralite_math::Ray3) -> Option<Real> {
        let a = Vec3 {
            x: 0.0,
            y: -self.half_height,
            z: 0.0,
        };
        let b = Vec3 {
            x: 0.0,
            y: self.half_height,
            z: 0.0,
        };
        let ba = b - a;
        let oa = r.origin - a;
        let baba = ba.dot(ba);
        let bard = ba.dot(r.direction);
        let baoa = ba.dot(oa);
        let rdoa = r.direction.dot(oa);
        let oaoa = oa.dot(oa);
        let qa = baba - bard * bard;
        let qb = baba * rdoa - baoa * bard;
        let qc = baba * oaoa - baoa * baoa - self.radius * self.radius * baba;
        if qa.abs() > ABS_EPSILON {
            let disc = qb * qb - qa * qc;
            if disc >= 0.0 {
                let t = (-qb - disc.sqrt()) / qa;
                let y = baoa + t * bard;
                if t >= 0.0 && y > 0.0 && y < baba {
                    return Some(t);
                }
            }
        }
        let center = if baoa < 0.0 { a } else { b };
        let m = r.origin - center;
        let q = m.dot(r.direction);
        let disc = q * q - (m.length_squared() - self.radius * self.radius);
        if disc < 0.0 {
            None
        } else {
            let t = -q - disc.sqrt();
            (t >= 0.0).then_some(t)
        }
    }
    /// Exact shape ray intersection with outward normal.
    #[must_use]
    pub fn ray_intersection(self, r: auralite_math::Ray3) -> Option<(Real, Vec3)> {
        let t = self.ray_t(r)?;
        let p = r.origin + r.direction * t;
        let cy = p.y.clamp(-self.half_height, self.half_height);
        let center = Vec3 {
            x: 0.0,
            y: cy,
            z: 0.0,
        };
        let normal = (p - center).normalized_or(Vec3::Y);
        Some((t, normal))
    }
    /// Volume of the capsule.
    #[must_use]
    pub fn volume(self) -> Real {
        core::f64::consts::PI as Real
            * self.radius
            * self.radius
            * (2.0 * self.half_height + 4.0 / 3.0 * self.radius)
    }
    /// Volume-density mass and diagonal inertia from cylinder plus sphere decomposition. The sphere's axial placement uses the parallel-axis theorem and is a documented engineering approximation for paired hemispheres.
    pub fn mass_properties(self, density: Real) -> Result<MassProperties3, GeometryError> {
        if !valid_positive(density) {
            return Err(GeometryError::InvalidInput);
        }
        let h = 2.0 * self.half_height;
        let cylinder_mass = core::f64::consts::PI as Real * self.radius * self.radius * h * density;
        let sphere_mass = 4.0 * core::f64::consts::PI as Real * self.radius.powi(3) * density / 3.0;
        let mass = cylinder_mass + sphere_mass;
        let axial = 0.5 * cylinder_mass * self.radius * self.radius
            + 0.4 * sphere_mass * self.radius * self.radius;
        let radial = cylinder_mass * (3.0 * self.radius * self.radius + h * h) / 12.0
            + 0.4 * sphere_mass * self.radius * self.radius
            + sphere_mass * self.half_height * self.half_height;
        Ok(MassProperties3 {
            mass,
            center: Vec3::ZERO,
            inertia_diagonal: Vec3 {
                x: radial,
                y: axial,
                z: radial,
            },
        })
    }
    /// Uniform scaling.
    pub fn scaled(self, s: Real) -> Result<Self, GeometryError> {
        Self::new(self.radius * s, self.half_height * s)
    }
}
impl ConvexPolygon {
    /// Tight local AABB.
    #[must_use]
    pub fn aabb(&self) -> Aabb2 {
        let min = Vec2 {
            x: self
                .vertices
                .iter()
                .map(|v| v.x)
                .fold(Real::INFINITY, Real::min),
            y: self
                .vertices
                .iter()
                .map(|v| v.y)
                .fold(Real::INFINITY, Real::min),
        };
        let max = Vec2 {
            x: self
                .vertices
                .iter()
                .map(|v| v.x)
                .fold(Real::NEG_INFINITY, Real::max),
            y: self
                .vertices
                .iter()
                .map(|v| v.y)
                .fold(Real::NEG_INFINITY, Real::max),
        };
        Aabb2::new(min, max).expect("validated polygon")
    }
    /// Bounding-circle radius about local origin.
    #[must_use]
    pub fn bounding_radius(&self) -> Real {
        self.vertices
            .iter()
            .map(|v| v.length())
            .fold(0.0, Real::max)
    }
    /// Closest point in/on polygon.
    #[must_use]
    pub fn closest_point(&self, p: Vec2) -> Vec2 {
        if self.contains(p) {
            return p;
        }
        (0..self.vertices.len())
            .map(|i| {
                auralite_math::Segment2::new(
                    self.vertices[i],
                    self.vertices[(i + 1) % self.vertices.len()],
                )
                .expect("edge")
                .closest_point(p)
                .0
            })
            .min_by(|a, b| {
                (p - *a)
                    .length_squared()
                    .total_cmp(&(p - *b).length_squared())
            })
            .expect("vertices")
    }
    /// Closest forward ray-edge hit.
    #[must_use]
    pub fn ray_t(&self, r: auralite_math::Ray2) -> Option<Real> {
        (0..self.vertices.len())
            .filter_map(|i| {
                advanced::Edge2::new(
                    self.vertices[i],
                    self.vertices[(i + 1) % self.vertices.len()],
                )
                .ok()?
                .ray_t(r)
            })
            .min_by(Real::total_cmp)
    }
    /// Exact ray intersection and outward normal across polygon edges.
    #[must_use]
    pub fn ray_intersection(&self, r: auralite_math::Ray2) -> Option<(Real, Vec2)> {
        (0..self.vertices.len())
            .filter_map(|i| {
                advanced::Edge2::new(
                    self.vertices[i],
                    self.vertices[(i + 1) % self.vertices.len()],
                )
                .ok()?
                .ray_intersection(r)
            })
            .min_by(|a, b| a.0.total_cmp(&b.0))
    }
    /// Uniform scaling.
    pub fn scaled(&self, s: Real) -> Result<Self, GeometryError> {
        if !valid_positive(s) {
            return Err(GeometryError::InvalidInput);
        }
        Self::new(self.vertices.iter().map(|v| *v * s).collect())
    }
}
/// Transforms a finite 2D AABB and returns its tight axis-aligned bounds.
#[must_use]
pub fn transform_aabb2(a: Aabb2, t: Transform2) -> Aabb2 {
    let c = (a.min + a.max) * 0.5;
    let h = (a.max - a.min) * 0.5;
    let m = t.rotation.matrix();
    let e = Vec2 {
        x: m.x.x.abs() * h.x + m.y.x.abs() * h.y,
        y: m.x.y.abs() * h.x + m.y.y.abs() * h.y,
    };
    let wc = t.transform_point(c);
    Aabb2::new(wc - e, wc + e).expect("finite input")
}
/// Transforms a finite 3D AABB and returns its tight axis-aligned bounds.
#[must_use]
pub fn transform_aabb3(a: Aabb3, t: Transform3) -> Aabb3 {
    let c = (a.min + a.max) * 0.5;
    let h = (a.max - a.min) * 0.5;
    let m = t.rotation.matrix();
    let e = Vec3 {
        x: m.x.x.abs() * h.x + m.y.x.abs() * h.y + m.z.x.abs() * h.z,
        y: m.x.y.abs() * h.x + m.y.y.abs() * h.y + m.z.y.abs() * h.z,
        z: m.x.z.abs() * h.x + m.y.z.abs() * h.y + m.z.z.abs() * h.z,
    };
    let wc = t.transform_point(c);
    Aabb3::new(wc - e, wc + e).expect("finite input")
}
/// Builds a counter-clockwise 2D convex hull using Andrew's monotone chain. Duplicate/collinear interior points are removed.
pub fn convex_hull2(points: &[Vec2]) -> Result<ConvexPolygon, GeometryError> {
    if points.len() < 3 || points.iter().any(|p| !p.is_finite()) {
        return Err(GeometryError::InvalidPolygon);
    }
    let mut p = points.to_vec();
    p.sort_by(|a, b| a.x.total_cmp(&b.x).then(a.y.total_cmp(&b.y)));
    p.dedup_by(|a, b| (*a - *b).length_squared() <= ABS_EPSILON * ABS_EPSILON);
    if p.len() < 3 {
        return Err(GeometryError::InvalidPolygon);
    }
    let mut lower: Vec<Vec2> = Vec::new();
    for &q in &p {
        while lower.len() >= 2
            && (lower[lower.len() - 1] - lower[lower.len() - 2]).cross(q - lower[lower.len() - 1])
                <= ABS_EPSILON
        {
            lower.pop();
        }
        lower.push(q);
    }
    let mut upper: Vec<Vec2> = Vec::new();
    for &q in p.iter().rev() {
        while upper.len() >= 2
            && (upper[upper.len() - 1] - upper[upper.len() - 2]).cross(q - upper[upper.len() - 1])
                <= ABS_EPSILON
        {
            upper.pop();
        }
        upper.push(q);
    }
    lower.pop();
    upper.pop();
    lower.extend(upper);
    ConvexPolygon::new(lower)
}

#[cfg(test)]
mod m2_acceptance_tests {
    use super::*;
    use auralite_math::{Ray2, Ray3};
    #[test]
    fn hull2_discards_interior_and_duplicates() {
        let h = convex_hull2(&[
            Vec2::ZERO,
            Vec2 { x: -1.0, y: -1.0 },
            Vec2 { x: 1.0, y: -1.0 },
            Vec2 { x: 1.0, y: 1.0 },
            Vec2 { x: -1.0, y: 1.0 },
            Vec2 { x: 1.0, y: 1.0 },
        ])
        .unwrap();
        assert_eq!(h.vertices().len(), 4);
        assert!(h.contains(Vec2::ZERO));
    }
    #[test]
    fn all_convex_support_maps_match_sample_references() {
        let circle = Circle2::new(1.7).unwrap();
        let sphere = Sphere3::new(2.1).unwrap();
        let box2 = Box2::new(Vec2 { x: 1.0, y: 3.0 }).unwrap();
        let box3 = Box3::new(Vec3 {
            x: 1.0,
            y: 2.0,
            z: 3.0,
        })
        .unwrap();
        let cap2 = Capsule2::new(0.4, 1.2).unwrap();
        let cap3 = Capsule3::new(0.5, 1.3).unwrap();
        for i in 0..10_000 {
            let a = i as Real * 0.017;
            let b = i as Real * 0.031;
            let d2 = Vec2 {
                x: a.cos(),
                y: a.sin(),
            };
            let d3 = Vec3 {
                x: a.cos() * b.sin(),
                y: a.sin() * b.sin(),
                z: b.cos(),
            };
            let sc = circle.support(d2);
            for j in 0..360 {
                let q = j as Real * core::f64::consts::TAU as Real / 360.0;
                let v = Vec2 {
                    x: q.cos() * circle.radius(),
                    y: q.sin() * circle.radius(),
                };
                assert!(sc.dot(d2) + 1.0e-5 >= v.dot(d2));
            }
            assert!(box2.support(d2).dot(d2) >= box2.closest_point(d2 * 100.0).dot(d2) - 1.0e-5);
            assert!(cap2.support(d2).dot(d2) >= cap2.closest_point(d2 * 100.0).dot(d2) - 1.0e-5);
            assert!((sphere.support(d3).length() - sphere.radius()).abs() < 1.0e-4);
            assert!(box3.support(d3).dot(d3) >= box3.closest_point(d3 * 100.0).dot(d3) - 1.0e-4);
            assert!(cap3.support(d3).dot(d3) >= cap3.closest_point(d3 * 100.0).dot(d3) - 1.0e-4);
        }
    }
    #[test]
    fn primitive_mass_and_query_analytics() {
        let c = Circle2::new(1.0).unwrap().mass_properties(1.0).unwrap();
        assert!((c.mass - core::f64::consts::PI as Real).abs() < 1.0e-5);
        let s = Sphere3::new(1.0).unwrap().mass_properties(1.0).unwrap();
        assert!((s.mass - 4.0 * core::f64::consts::PI as Real / 3.0).abs() < 1.0e-5);
        let b = Box2::new(Vec2 { x: 1.0, y: 2.0 }).unwrap();
        assert_eq!(
            b.ray_interval(Ray2::new(Vec2 { x: -3.0, y: 0.0 }, Vec2::X).unwrap()),
            Some((2.0, 4.0))
        );
        let cap = Capsule3::new(0.5, 1.0).unwrap();
        assert!(
            cap.ray_t(
                Ray3::new(
                    Vec3 {
                        x: -2.0,
                        y: 0.0,
                        z: 0.0
                    },
                    Vec3::X
                )
                .unwrap()
            )
            .is_some()
        );
    }
    #[test]
    fn every_scaling_rule_rejects_invalid() {
        assert!(Circle2::new(1.0).unwrap().scaled(-1.0).is_err());
        assert!(Sphere3::new(1.0).unwrap().scaled(0.0).is_err());
        assert!(Capsule2::new(1.0, 1.0).unwrap().scaled(Real::NAN).is_err());
        assert!(
            Box3::new(Vec3 {
                x: 1.0,
                y: 1.0,
                z: 1.0
            })
            .unwrap()
            .scaled(Vec3 {
                x: 1.0,
                y: -1.0,
                z: 1.0
            })
            .is_err()
        );
        assert!(convex_hull2(&[Vec2::ZERO, Vec2::X, Vec2 { x: 2.0, y: 0.0 }]).is_err());
    }
}
