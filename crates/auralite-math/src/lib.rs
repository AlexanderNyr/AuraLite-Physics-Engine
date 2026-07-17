//! Dimension-safe, finite-checked mathematics for AuraLite.
#![allow(unsafe_code)]

use core::ops::{Add, AddAssign, Div, DivAssign, Mul, MulAssign, Neg, Sub, SubAssign};

/// Simulation scalar. The `f64` feature is mutually exclusive with `f32`.
#[cfg(all(feature = "f64", not(feature = "f32")))]
pub type Real = f64;
/// Simulation scalar (default).
#[cfg(not(all(feature = "f64", not(feature = "f32"))))]
pub type Real = f32;

/// Absolute tolerance at the recommended metre scale.
pub const ABS_EPSILON: Real = 1.0e-6;
/// Relative comparison tolerance.
pub const REL_EPSILON: Real = 1.0e-5;
/// Contact skin/slop in metres.
pub const CONTACT_SLOP: Real = 0.005;

macro_rules! vector {
    ($name:ident, $($field:ident),+) => {
        #[doc = "A finite dimension-specific vector."]
        #[derive(Clone, Copy, Debug, Default, PartialEq)]
        #[repr(C)]
        #[allow(missing_docs)]
        pub struct $name { $(pub $field: Real),+ }
        impl $name {
            /// Constructs a vector, rejecting non-finite values.
            #[allow(clippy::too_many_arguments)]
            pub fn new($($field: Real),+) -> Result<Self, MathError> {
                let v = Self { $($field),+ };
                if v.is_finite() { Ok(v) } else { Err(MathError::NonFinite) }
            }
            /// Returns true when every component is finite.
            #[must_use] pub fn is_finite(self) -> bool { true $(&& self.$field.is_finite())+ }
            /// Dot product.
            #[must_use] pub fn dot(self, rhs: Self) -> Real { 0.0 $(+ self.$field * rhs.$field)+ }
            /// Squared Euclidean length.
            #[must_use] pub fn length_squared(self) -> Real { self.dot(self) }
            /// Euclidean length.
            #[must_use] pub fn length(self) -> Real { self.length_squared().sqrt() }
            /// Normalizes, returning `fallback` for near-zero/non-finite input.
            #[must_use] pub fn normalized_or(self, fallback: Self) -> Self {
                let n2 = self.length_squared();
                if n2 > ABS_EPSILON * ABS_EPSILON && n2.is_finite() { self / n2.sqrt() } else { fallback }
            }
        }
        impl Add for $name { type Output=Self; fn add(self,rhs:Self)->Self { Self{$($field:self.$field+rhs.$field),+} } }
        impl Sub for $name { type Output=Self; fn sub(self,rhs:Self)->Self { Self{$($field:self.$field-rhs.$field),+} } }
        impl AddAssign for $name { fn add_assign(&mut self,rhs:Self){$(self.$field+=rhs.$field;)+} }
        impl SubAssign for $name { fn sub_assign(&mut self,rhs:Self){$(self.$field-=rhs.$field;)+} }
        impl Mul<Real> for $name { type Output=Self; fn mul(self,r:Real)->Self { Self{$($field:self.$field*r),+} } }
        impl MulAssign<Real> for $name { fn mul_assign(&mut self,r:Real){$(self.$field*=r;)+} }
        impl Div<Real> for $name { type Output=Self; fn div(self,r:Real)->Self { Self{$($field:self.$field/r),+} } }
        impl DivAssign<Real> for $name { fn div_assign(&mut self,r:Real){$(self.$field/=r;)+} }
        impl Neg for $name { type Output=Self; fn neg(self)->Self { Self{$($field:-self.$field),+} } }
        impl Mul for $name { type Output=Self; fn mul(self,rhs:Self)->Self { Self{$($field:self.$field*rhs.$field),+} } }
    }
}
vector!(Vec2, x, y);
vector!(Vec3, x, y, z);

impl Vec2 {
    /// Zero vector.
    pub const ZERO: Self = Self { x: 0.0, y: 0.0 };
    /// Unit +X vector.
    pub const X: Self = Self { x: 1.0, y: 0.0 };
    /// Unit +Y vector.
    pub const Y: Self = Self { x: 0.0, y: 1.0 };
    /// 2D scalar cross product.
    #[must_use]
    pub fn cross(self, rhs: Self) -> Real {
        self.x * rhs.y - self.y * rhs.x
    }
}
impl Vec3 {
    /// Zero vector.
    pub const ZERO: Self = Self {
        x: 0.0,
        y: 0.0,
        z: 0.0,
    };
    /// Unit +X vector.
    pub const X: Self = Self {
        x: 1.0,
        y: 0.0,
        z: 0.0,
    };
    /// Unit +Y vector.
    pub const Y: Self = Self {
        x: 0.0,
        y: 1.0,
        z: 0.0,
    };
    /// Unit +Z vector.
    pub const Z: Self = Self {
        x: 0.0,
        y: 0.0,
        z: 1.0,
    };
    /// Cross product.
    #[must_use]
    pub fn cross(self, r: Self) -> Self {
        Self {
            x: self.y * r.z - self.z * r.y,
            y: self.z * r.x - self.x * r.z,
            z: self.x * r.y - self.y * r.x,
        }
    }
}

/// Validated math input error.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum MathError {
    /// A value was NaN or infinite.
    NonFinite,
    /// A dimension was zero or negative.
    NonPositive,
}

/// A native 2D rotation represented by sine/cosine.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct Rot2 {
    /// Cosine of angle.
    pub c: Real,
    /// Sine of angle.
    pub s: Real,
}
impl Rot2 {
    /// Creates a rotation from radians.
    pub fn from_radians(a: Real) -> Result<Self, MathError> {
        if a.is_finite() {
            let (s, c) = a.sin_cos();
            Ok(Self { c, s })
        } else {
            Err(MathError::NonFinite)
        }
    }
    /// Identity rotation.
    #[must_use]
    pub const fn identity() -> Self {
        Self { c: 1.0, s: 0.0 }
    }
    /// Rotates a vector.
    #[must_use]
    pub fn rotate(self, v: Vec2) -> Vec2 {
        Vec2 {
            x: self.c * v.x - self.s * v.y,
            y: self.s * v.x + self.c * v.y,
        }
    }
    /// Inverse.
    #[must_use]
    pub fn inverse(self) -> Self {
        Self {
            c: self.c,
            s: -self.s,
        }
    }
}

/// A normalized quaternion rotation.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct Quat {
    /// Scalar component.
    pub w: Real,
    /// X component.
    pub x: Real,
    /// Y component.
    pub y: Real,
    /// Z component.
    pub z: Real,
}
impl Quat {
    /// Identity rotation.
    #[must_use]
    pub const fn identity() -> Self {
        Self {
            w: 1.0,
            x: 0.0,
            y: 0.0,
            z: 0.0,
        }
    }
    /// Creates an axis-angle quaternion, rejecting invalid values.
    pub fn axis_angle(axis: Vec3, angle: Real) -> Result<Self, MathError> {
        if !angle.is_finite()
            || !axis.is_finite()
            || axis.length_squared() <= ABS_EPSILON * ABS_EPSILON
        {
            return Err(MathError::NonFinite);
        }
        let a = axis.normalized_or(Vec3::X);
        let (s, c) = (angle * 0.5).sin_cos();
        Ok(Self {
            w: c,
            x: a.x * s,
            y: a.y * s,
            z: a.z * s,
        })
    }
    /// Rotates a vector.
    #[must_use]
    pub fn rotate(self, v: Vec3) -> Vec3 {
        let q = Vec3 {
            x: self.x,
            y: self.y,
            z: self.z,
        };
        v + q.cross(v) * (2.0 * self.w) + q.cross(q.cross(v)) * 2.0
    }
    /// Conjugate/inverse for normalized quaternions.
    #[must_use]
    pub fn inverse(self) -> Self {
        Self {
            w: self.w,
            x: -self.x,
            y: -self.y,
            z: -self.z,
        }
    }
    /// Normalizes, returning `fallback` for near-zero/non-finite input.
    #[must_use]
    pub fn normalized_or(self, fallback: Self) -> Self {
        let n2 = self.w * self.w + self.x * self.x + self.y * self.y + self.z * self.z;
        if n2 > ABS_EPSILON * ABS_EPSILON && n2.is_finite() {
            let inv = 1.0 / n2.sqrt();
            Self {
                w: self.w * inv,
                x: self.x * inv,
                y: self.y * inv,
                z: self.z * inv,
            }
        } else {
            fallback
        }
    }
}

impl Mul for Quat {
    type Output = Self;
    fn mul(self, rhs: Self) -> Self {
        Self {
            w: self.w * rhs.w - self.x * rhs.x - self.y * rhs.y - self.z * rhs.z,
            x: self.w * rhs.x + self.x * rhs.w + self.y * rhs.z - self.z * rhs.y,
            y: self.w * rhs.y - self.x * rhs.z + self.y * rhs.w + self.z * rhs.x,
            z: self.w * rhs.z + self.x * rhs.y - self.y * rhs.x + self.z * rhs.w,
        }
    }
}

/// Axis-aligned 2D bounds.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct Aabb2 {
    /// Minimum corner.
    pub min: Vec2,
    /// Maximum corner.
    pub max: Vec2,
}
impl Aabb2 {
    /// Validates ordered finite bounds.
    pub fn new(min: Vec2, max: Vec2) -> Result<Self, MathError> {
        if min.is_finite() && max.is_finite() && min.x <= max.x && min.y <= max.y {
            Ok(Self { min, max })
        } else {
            Err(MathError::NonFinite)
        }
    }
    /// Tests inclusive overlap.
    #[must_use]
    pub fn overlaps(self, r: Self) -> bool {
        self.min.x <= r.max.x
            && self.max.x >= r.min.x
            && self.min.y <= r.max.y
            && self.max.y >= r.min.y
    }
    /// Returns expanded bounds.
    #[must_use]
    pub fn expanded(self, e: Real) -> Self {
        Self {
            min: self.min - Vec2 { x: e, y: e },
            max: self.max + Vec2 { x: e, y: e },
        }
    }
}
/// Axis-aligned 3D bounds.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct Aabb3 {
    /// Minimum corner.
    pub min: Vec3,
    /// Maximum corner.
    pub max: Vec3,
}
impl Aabb3 {
    /// Validates ordered finite bounds.
    pub fn new(min: Vec3, max: Vec3) -> Result<Self, MathError> {
        if min.is_finite() && max.is_finite() && min.x <= max.x && min.y <= max.y && min.z <= max.z
        {
            Ok(Self { min, max })
        } else {
            Err(MathError::NonFinite)
        }
    }
    /// Tests inclusive overlap.
    #[must_use]
    pub fn overlaps(self, r: Self) -> bool {
        self.min.x <= r.max.x
            && self.max.x >= r.min.x
            && self.min.y <= r.max.y
            && self.max.y >= r.min.y
            && self.min.z <= r.max.z
            && self.max.z >= r.min.z
    }
}

/// A column-major 2×2 matrix.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct Mat2 {
    /// First column.
    pub x: Vec2,
    /// Second column.
    pub y: Vec2,
}
impl Mat2 {
    /// Identity matrix.
    pub const IDENTITY: Self = Self {
        x: Vec2 { x: 1.0, y: 0.0 },
        y: Vec2 { x: 0.0, y: 1.0 },
    };
    /// Constructs from finite columns.
    pub fn from_cols(x: Vec2, y: Vec2) -> Result<Self, MathError> {
        if x.is_finite() && y.is_finite() {
            Ok(Self { x, y })
        } else {
            Err(MathError::NonFinite)
        }
    }
    /// Transforms a vector.
    #[must_use]
    pub fn mul_vec(self, v: Vec2) -> Vec2 {
        self.x * v.x + self.y * v.y
    }
    /// Determinant.
    #[must_use]
    pub fn determinant(self) -> Real {
        self.x.cross(self.y)
    }
    /// Inverse, or `None` when singular or ill-conditioned.
    #[must_use]
    pub fn inverse(self) -> Option<Self> {
        let d = self.determinant();
        if !d.is_finite() || d.abs() <= ABS_EPSILON {
            return None;
        }
        Some(Self {
            x: Vec2 {
                x: self.y.y / d,
                y: -self.x.y / d,
            },
            y: Vec2 {
                x: -self.y.x / d,
                y: self.x.x / d,
            },
        })
    }
}

/// A column-major 3×3 matrix.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct Mat3 {
    /// First column.
    pub x: Vec3,
    /// Second column.
    pub y: Vec3,
    /// Third column.
    pub z: Vec3,
}
impl Mat3 {
    /// Identity matrix.
    pub const IDENTITY: Self = Self {
        x: Vec3 {
            x: 1.0,
            y: 0.0,
            z: 0.0,
        },
        y: Vec3 {
            x: 0.0,
            y: 1.0,
            z: 0.0,
        },
        z: Vec3 {
            x: 0.0,
            y: 0.0,
            z: 1.0,
        },
    };
    /// Constructs from finite columns.
    pub fn from_cols(x: Vec3, y: Vec3, z: Vec3) -> Result<Self, MathError> {
        if x.is_finite() && y.is_finite() && z.is_finite() {
            Ok(Self { x, y, z })
        } else {
            Err(MathError::NonFinite)
        }
    }
    /// Transforms a vector.
    #[must_use]
    pub fn mul_vec(self, v: Vec3) -> Vec3 {
        self.x * v.x + self.y * v.y + self.z * v.z
    }
    /// Determinant.
    #[must_use]
    pub fn determinant(self) -> Real {
        self.x.dot(self.y.cross(self.z))
    }
    /// Inverse, or `None` when singular or ill-conditioned.
    #[must_use]
    pub fn inverse(self) -> Option<Self> {
        let c0 = self.y.cross(self.z);
        let c1 = self.z.cross(self.x);
        let c2 = self.x.cross(self.y);
        let d = self.x.dot(c0);
        if !d.is_finite() || d.abs() <= ABS_EPSILON {
            return None;
        }
        Some(Self {
            x: Vec3 {
                x: c0.x,
                y: c1.x,
                z: c2.x,
            } / d,
            y: Vec3 {
                x: c0.y,
                y: c1.y,
                z: c2.y,
            } / d,
            z: Vec3 {
                x: c0.z,
                y: c1.z,
                z: c2.z,
            } / d,
        })
    }
}

impl Rot2 {
    /// Composes rotations, applying `rhs` first.
    #[must_use]
    pub fn compose(self, rhs: Self) -> Self {
        Self {
            c: self.c * rhs.c - self.s * rhs.s,
            s: self.s * rhs.c + self.c * rhs.s,
        }
    }
    /// Converts to a rotation matrix.
    #[must_use]
    pub fn matrix(self) -> Mat2 {
        Mat2 {
            x: Vec2 {
                x: self.c,
                y: self.s,
            },
            y: Vec2 {
                x: -self.s,
                y: self.c,
            },
        }
    }
}
impl Quat {
    /// Composes normalized rotations, applying `rhs` first.
    #[must_use]
    pub fn compose(self, rhs: Self) -> Self {
        Self {
            w: self.w * rhs.w - self.x * rhs.x - self.y * rhs.y - self.z * rhs.z,
            x: self.w * rhs.x + self.x * rhs.w + self.y * rhs.z - self.z * rhs.y,
            y: self.w * rhs.y - self.x * rhs.z + self.y * rhs.w + self.z * rhs.x,
            z: self.w * rhs.z + self.x * rhs.y - self.y * rhs.x + self.z * rhs.w,
        }
    }
    /// Converts to a rotation matrix.
    #[must_use]
    pub fn matrix(self) -> Mat3 {
        Mat3 {
            x: self.rotate(Vec3::X),
            y: self.rotate(Vec3::Y),
            z: self.rotate(Vec3 {
                x: 0.0,
                y: 0.0,
                z: 1.0,
            }),
        }
    }
}

/// A rigid 2D transform.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct Transform2 {
    /// Translation.
    pub translation: Vec2,
    /// Rotation.
    pub rotation: Rot2,
}
impl Transform2 {
    /// Identity transform.
    #[must_use]
    pub const fn identity() -> Self {
        Self {
            translation: Vec2::ZERO,
            rotation: Rot2::identity(),
        }
    }
    /// Validated constructor.
    pub fn new(translation: Vec2, rotation: Rot2) -> Result<Self, MathError> {
        if translation.is_finite() {
            Ok(Self {
                translation,
                rotation,
            })
        } else {
            Err(MathError::NonFinite)
        }
    }
    /// Transforms a point.
    #[must_use]
    pub fn transform_point(self, p: Vec2) -> Vec2 {
        self.rotation.rotate(p) + self.translation
    }
    /// Inverse transform.
    #[must_use]
    pub fn inverse(self) -> Self {
        let r = self.rotation.inverse();
        Self {
            translation: r.rotate(-self.translation),
            rotation: r,
        }
    }
    /// Composes transforms, applying `rhs` first.
    #[must_use]
    pub fn compose(self, rhs: Self) -> Self {
        Self {
            translation: self.transform_point(rhs.translation),
            rotation: self.rotation.compose(rhs.rotation),
        }
    }
}
/// A rigid 3D transform.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct Transform3 {
    /// Translation.
    pub translation: Vec3,
    /// Rotation.
    pub rotation: Quat,
}
impl Transform3 {
    /// Identity transform.
    #[must_use]
    pub const fn identity() -> Self {
        Self {
            translation: Vec3::ZERO,
            rotation: Quat::identity(),
        }
    }
    /// Validated constructor.
    pub fn new(translation: Vec3, rotation: Quat) -> Result<Self, MathError> {
        if translation.is_finite() {
            Ok(Self {
                translation,
                rotation,
            })
        } else {
            Err(MathError::NonFinite)
        }
    }
    /// Transforms a point.
    #[must_use]
    pub fn transform_point(self, p: Vec3) -> Vec3 {
        self.rotation.rotate(p) + self.translation
    }
    /// Inverse transform.
    #[must_use]
    pub fn inverse(self) -> Self {
        let r = self.rotation.inverse();
        Self {
            translation: r.rotate(-self.translation),
            rotation: r,
        }
    }
    /// Composes transforms, applying `rhs` first.
    #[must_use]
    pub fn compose(self, rhs: Self) -> Self {
        Self {
            translation: self.transform_point(rhs.translation),
            rotation: self.rotation.compose(rhs.rotation),
        }
    }
}

/// A finite 2D line segment.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct Segment2 {
    /// First endpoint.
    pub a: Vec2,
    /// Second endpoint.
    pub b: Vec2,
}
impl Segment2 {
    /// Constructs a non-degenerate segment.
    pub fn new(a: Vec2, b: Vec2) -> Result<Self, MathError> {
        if a.is_finite() && b.is_finite() && (b - a).length_squared() > ABS_EPSILON * ABS_EPSILON {
            Ok(Self { a, b })
        } else {
            Err(MathError::NonPositive)
        }
    }
    /// Closest point and clamped segment parameter.
    #[must_use]
    pub fn closest_point(self, p: Vec2) -> (Vec2, Real) {
        let d = self.b - self.a;
        let t = ((p - self.a).dot(d) / d.length_squared()).clamp(0.0, 1.0);
        (self.a + d * t, t)
    }
}
/// A finite 3D line segment.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct Segment3 {
    /// First endpoint.
    pub a: Vec3,
    /// Second endpoint.
    pub b: Vec3,
}
impl Segment3 {
    /// Constructs a non-degenerate segment.
    pub fn new(a: Vec3, b: Vec3) -> Result<Self, MathError> {
        if a.is_finite() && b.is_finite() && (b - a).length_squared() > ABS_EPSILON * ABS_EPSILON {
            Ok(Self { a, b })
        } else {
            Err(MathError::NonPositive)
        }
    }
    /// Closest point and clamped segment parameter.
    #[must_use]
    pub fn closest_point(self, p: Vec3) -> (Vec3, Real) {
        let d = self.b - self.a;
        let t = ((p - self.a).dot(d) / d.length_squared()).clamp(0.0, 1.0);
        (self.a + d * t, t)
    }
}
/// A normalized 3D ray.
///
/// # Example
/// ```
/// use auralite_math::{Ray3, Vec3};
/// let ray = Ray3::new(Vec3 { x: 0.0, y: 5.0, z: 0.0 }, Vec3 { x: 0.0, y: -1.0, z: 0.0 }).unwrap();
/// assert_eq!(ray.point_at(2.0).y, 3.0);
/// ```
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct Ray3 {
    /// Origin.
    pub origin: Vec3,
    /// Unit direction.
    pub direction: Vec3,
}
impl Ray3 {
    /// Constructs from finite origin and nonzero direction.
    pub fn new(origin: Vec3, direction: Vec3) -> Result<Self, MathError> {
        if !origin.is_finite()
            || !direction.is_finite()
            || direction.length_squared() <= ABS_EPSILON * ABS_EPSILON
        {
            Err(MathError::NonFinite)
        } else {
            Ok(Self {
                origin,
                direction: direction.normalized_or(Vec3::X),
            })
        }
    }
    /// Evaluates the ray at parameter `t`.
    #[must_use]
    pub fn at(self, t: Real) -> Vec3 {
        self.origin + self.direction * t
    }
    /// Evaluates the ray at parameter `t`.
    #[must_use]
    pub fn point_at(self, t: Real) -> Vec3 {
        self.at(t)
    }
}
/// Hessian-normal plane satisfying `normal·point = offset`.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct Plane3 {
    /// Unit normal.
    pub normal: Vec3,
    /// Signed origin offset.
    pub offset: Real,
}
impl Plane3 {
    /// Constructs from a point and nonzero normal.
    pub fn from_point_normal(point: Vec3, normal: Vec3) -> Result<Self, MathError> {
        if !point.is_finite()
            || !normal.is_finite()
            || normal.length_squared() <= ABS_EPSILON * ABS_EPSILON
        {
            return Err(MathError::NonFinite);
        }
        let n = normal.normalized_or(Vec3::Y);
        Ok(Self {
            normal: n,
            offset: n.dot(point),
        })
    }
    /// Signed distance, positive along the normal.
    #[must_use]
    pub fn signed_distance(self, p: Vec3) -> Real {
        self.normal.dot(p) - self.offset
    }
    /// Ray intersection parameter, rejecting parallel and behind-origin hits.
    #[must_use]
    pub fn ray_t(self, ray: Ray3) -> Option<Real> {
        let d = self.normal.dot(ray.direction);
        if d.abs() <= ABS_EPSILON {
            return None;
        }
        let t = (self.offset - self.normal.dot(ray.origin)) / d;
        if t >= 0.0 { Some(t) } else { None }
    }
}
/// A non-degenerate 3D triangle.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct Triangle3 {
    /// First vertex.
    pub a: Vec3,
    /// Second vertex.
    pub b: Vec3,
    /// Third vertex.
    pub c: Vec3,
}
impl Triangle3 {
    /// Validates finite vertices and nonzero area.
    pub fn new(a: Vec3, b: Vec3, c: Vec3) -> Result<Self, MathError> {
        if a.is_finite()
            && b.is_finite()
            && c.is_finite()
            && (b - a).cross(c - a).length_squared() > ABS_EPSILON * ABS_EPSILON
        {
            Ok(Self { a, b, c })
        } else {
            Err(MathError::NonPositive)
        }
    }
    /// Unit geometric normal.
    #[must_use]
    pub fn normal(self) -> Vec3 {
        (self.b - self.a)
            .cross(self.c - self.a)
            .normalized_or(Vec3::Y)
    }
    /// Area.
    #[must_use]
    pub fn area(self) -> Real {
        (self.b - self.a).cross(self.c - self.a).length() * 0.5
    }
}

/// A normalized 2D ray.
///
/// # Example
/// ```
/// use auralite_math::{Ray2, Vec2};
/// let ray = Ray2::new(Vec2 { x: 0.0, y: 5.0 }, Vec2 { x: 0.0, y: -1.0 }).unwrap();
/// assert_eq!(ray.point_at(2.0).y, 3.0);
/// ```
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct Ray2 {
    /// Origin.
    pub origin: Vec2,
    /// Unit direction.
    pub direction: Vec2,
}
impl Ray2 {
    /// Constructs from a finite origin and nonzero direction.
    pub fn new(origin: Vec2, direction: Vec2) -> Result<Self, MathError> {
        if !origin.is_finite()
            || !direction.is_finite()
            || direction.length_squared() <= ABS_EPSILON * ABS_EPSILON
        {
            Err(MathError::NonFinite)
        } else {
            Ok(Self {
                origin,
                direction: direction.normalized_or(Vec2::X),
            })
        }
    }
    /// Evaluates the ray's supporting line.
    #[must_use]
    pub fn at(self, t: Real) -> Vec2 {
        self.origin + self.direction * t
    }
    /// Evaluates the ray at parameter `t`.
    #[must_use]
    pub fn point_at(self, t: Real) -> Vec2 {
        self.at(t)
    }
}
/// An infinite 2D line in point-direction form.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct Line2 {
    /// A point on the line.
    pub point: Vec2,
    /// Unit direction.
    pub direction: Vec2,
}
impl Line2 {
    /// Validates and normalizes the direction.
    pub fn new(point: Vec2, direction: Vec2) -> Result<Self, MathError> {
        let r = Ray2::new(point, direction)?;
        Ok(Self {
            point: r.origin,
            direction: r.direction,
        })
    }
    /// Closest point on the infinite line.
    #[must_use]
    pub fn closest_point(self, p: Vec2) -> Vec2 {
        self.point + self.direction * (p - self.point).dot(self.direction)
    }
    /// Intersection of two nonparallel lines.
    #[must_use]
    pub fn intersection(self, rhs: Self) -> Option<Vec2> {
        let d = self.direction.cross(rhs.direction);
        if d.abs() <= ABS_EPSILON {
            return None;
        }
        Some(self.point + self.direction * ((rhs.point - self.point).cross(rhs.direction) / d))
    }
}
/// A 2D half-space boundary satisfying `normal·point = offset`.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct Plane2 {
    /// Unit outward normal.
    pub normal: Vec2,
    /// Offset from origin.
    pub offset: Real,
}
impl Plane2 {
    /// Constructs from point and nonzero normal.
    pub fn from_point_normal(point: Vec2, normal: Vec2) -> Result<Self, MathError> {
        if !point.is_finite()
            || !normal.is_finite()
            || normal.length_squared() <= ABS_EPSILON * ABS_EPSILON
        {
            return Err(MathError::NonFinite);
        }
        let n = normal.normalized_or(Vec2::X);
        Ok(Self {
            normal: n,
            offset: n.dot(point),
        })
    }
    /// Signed distance.
    #[must_use]
    pub fn signed_distance(self, p: Vec2) -> Real {
        self.normal.dot(p) - self.offset
    }
    /// Closest boundary point.
    #[must_use]
    pub fn closest_point(self, p: Vec2) -> Vec2 {
        p - self.normal * self.signed_distance(p)
    }
    /// Forward ray hit parameter.
    #[must_use]
    pub fn ray_t(self, ray: Ray2) -> Option<Real> {
        let d = self.normal.dot(ray.direction);
        if d.abs() <= ABS_EPSILON {
            return None;
        }
        let t = (self.offset - self.normal.dot(ray.origin)) / d;
        (t >= 0.0).then_some(t)
    }
}
/// A non-degenerate counter-clockwise 2D triangle.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct Triangle2 {
    /// First vertex.
    pub a: Vec2,
    /// Second vertex.
    pub b: Vec2,
    /// Third vertex.
    pub c: Vec2,
}
impl Triangle2 {
    /// Rejects non-finite, clockwise, collinear, and tiny triangles.
    pub fn new(a: Vec2, b: Vec2, c: Vec2) -> Result<Self, MathError> {
        if a.is_finite() && b.is_finite() && c.is_finite() && (b - a).cross(c - a) > ABS_EPSILON {
            Ok(Self { a, b, c })
        } else {
            Err(MathError::NonPositive)
        }
    }
    /// Area.
    #[must_use]
    pub fn area(self) -> Real {
        (self.b - self.a).cross(self.c - self.a) * 0.5
    }
    /// Inclusive containment using robust orientation signs.
    #[must_use]
    pub fn contains(self, p: Vec2) -> bool {
        orientation2(self.a, self.b, p) >= -ABS_EPSILON
            && orientation2(self.b, self.c, p) >= -ABS_EPSILON
            && orientation2(self.c, self.a, p) >= -ABS_EPSILON
    }
    /// Closest point on or in the triangle.
    #[must_use]
    pub fn closest_point(self, p: Vec2) -> Vec2 {
        if self.contains(p) {
            return p;
        }
        let ab = Segment2 {
            a: self.a,
            b: self.b,
        }
        .closest_point(p)
        .0;
        let bc = Segment2 {
            a: self.b,
            b: self.c,
        }
        .closest_point(p)
        .0;
        let ca = Segment2 {
            a: self.c,
            b: self.a,
        }
        .closest_point(p)
        .0;
        let dab = (p - ab).length_squared();
        let dbc = (p - bc).length_squared();
        let dca = (p - ca).length_squared();
        if dab <= dbc && dab <= dca {
            ab
        } else if dbc <= dca {
            bc
        } else {
            ca
        }
    }
}
/// Signed twice-area orientation predicate with scale-aware collinearity classification.
#[must_use]
pub fn orientation2(a: Vec2, b: Vec2, c: Vec2) -> Real {
    (b - a).cross(c - a)
}
/// Returns whether an orientation is numerically collinear using absolute and relative tolerance.
#[must_use]
pub fn collinear2(a: Vec2, b: Vec2, c: Vec2) -> bool {
    let ab = b - a;
    let ac = c - a;
    orientation2(a, b, c).abs() <= ABS_EPSILON + REL_EPSILON * ab.length() * ac.length()
}
/// Returns whether three 3D points are numerically collinear.
#[must_use]
pub fn collinear3(a: Vec3, b: Vec3, c: Vec3) -> bool {
    let ab = b - a;
    let ac = c - a;
    ab.cross(ac).length() <= ABS_EPSILON + REL_EPSILON * ab.length() * ac.length()
}
/// Returns whether four points are numerically coplanar.
#[must_use]
pub fn coplanar3(a: Vec3, b: Vec3, c: Vec3, d: Vec3) -> bool {
    let ab = b - a;
    let ac = c - a;
    let ad = d - a;
    ab.dot(ac.cross(ad)).abs()
        <= ABS_EPSILON + REL_EPSILON * ab.length() * ac.length() * ad.length()
}

/// A 2D oriented box.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct Obb2 {
    /// Center.
    pub center: Vec2,
    /// Positive half extents.
    pub half_extents: Vec2,
    /// Orientation.
    pub rotation: Rot2,
}
impl Obb2 {
    /// Validated constructor.
    pub fn new(center: Vec2, half_extents: Vec2, rotation: Rot2) -> Result<Self, MathError> {
        if center.is_finite()
            && half_extents.is_finite()
            && half_extents.x > 0.0
            && half_extents.y > 0.0
        {
            Ok(Self {
                center,
                half_extents,
                rotation,
            })
        } else {
            Err(MathError::NonPositive)
        }
    }
    /// Inclusive containment.
    #[must_use]
    pub fn contains(self, p: Vec2) -> bool {
        let q = self.rotation.inverse().rotate(p - self.center);
        q.x.abs() <= self.half_extents.x && q.y.abs() <= self.half_extents.y
    }
    /// Closest point.
    #[must_use]
    pub fn closest_point(self, p: Vec2) -> Vec2 {
        let q = self.rotation.inverse().rotate(p - self.center);
        self.center
            + self.rotation.rotate(Vec2 {
                x: q.x.clamp(-self.half_extents.x, self.half_extents.x),
                y: q.y.clamp(-self.half_extents.y, self.half_extents.y),
            })
    }
    /// Tight world AABB.
    #[must_use]
    pub fn aabb(self) -> Aabb2 {
        let m = self.rotation.matrix();
        let e = Vec2 {
            x: m.x.x.abs() * self.half_extents.x + m.y.x.abs() * self.half_extents.y,
            y: m.x.y.abs() * self.half_extents.x + m.y.y.abs() * self.half_extents.y,
        };
        Aabb2::new(self.center - e, self.center + e).expect("validated OBB")
    }
}
/// A 3D oriented box.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct Obb3 {
    /// Center.
    pub center: Vec3,
    /// Positive half extents.
    pub half_extents: Vec3,
    /// Orientation.
    pub rotation: Quat,
}
impl Obb3 {
    /// Validated constructor.
    pub fn new(center: Vec3, half_extents: Vec3, rotation: Quat) -> Result<Self, MathError> {
        if center.is_finite()
            && half_extents.is_finite()
            && half_extents.x > 0.0
            && half_extents.y > 0.0
            && half_extents.z > 0.0
        {
            Ok(Self {
                center,
                half_extents,
                rotation,
            })
        } else {
            Err(MathError::NonPositive)
        }
    }
    /// Inclusive containment.
    #[must_use]
    pub fn contains(self, p: Vec3) -> bool {
        let q = self.rotation.inverse().rotate(p - self.center);
        q.x.abs() <= self.half_extents.x
            && q.y.abs() <= self.half_extents.y
            && q.z.abs() <= self.half_extents.z
    }
    /// Closest point.
    #[must_use]
    pub fn closest_point(self, p: Vec3) -> Vec3 {
        let q = self.rotation.inverse().rotate(p - self.center);
        self.center
            + self.rotation.rotate(Vec3 {
                x: q.x.clamp(-self.half_extents.x, self.half_extents.x),
                y: q.y.clamp(-self.half_extents.y, self.half_extents.y),
                z: q.z.clamp(-self.half_extents.z, self.half_extents.z),
            })
    }
    /// Tight world AABB.
    #[must_use]
    pub fn aabb(self) -> Aabb3 {
        let m = self.rotation.matrix();
        let h = self.half_extents;
        let e = Vec3 {
            x: m.x.x.abs() * h.x + m.y.x.abs() * h.y + m.z.x.abs() * h.z,
            y: m.x.y.abs() * h.x + m.y.y.abs() * h.y + m.z.y.abs() * h.z,
            z: m.x.z.abs() * h.x + m.y.z.abs() * h.y + m.z.z.abs() * h.z,
        };
        Aabb3::new(self.center - e, self.center + e).expect("validated OBB")
    }
}

impl Triangle3 {
    /// Inclusive closest point using the Voronoi-region algorithm.
    #[must_use]
    pub fn closest_point(self, p: Vec3) -> Vec3 {
        let ab = self.b - self.a;
        let ac = self.c - self.a;
        let ap = p - self.a;
        let d1 = ab.dot(ap);
        let d2 = ac.dot(ap);
        if d1 <= 0.0 && d2 <= 0.0 {
            return self.a;
        }
        let bp = p - self.b;
        let d3 = ab.dot(bp);
        let d4 = ac.dot(bp);
        if d3 >= 0.0 && d4 <= d3 {
            return self.b;
        }
        let vc = d1 * d4 - d3 * d2;
        if vc <= 0.0 && d1 >= 0.0 && d3 <= 0.0 {
            return self.a + ab * (d1 / (d1 - d3));
        }
        let cp = p - self.c;
        let d5 = ab.dot(cp);
        let d6 = ac.dot(cp);
        if d6 >= 0.0 && d5 <= d6 {
            return self.c;
        }
        let vb = d5 * d2 - d1 * d6;
        if vb <= 0.0 && d2 >= 0.0 && d6 <= 0.0 {
            return self.a + ac * (d2 / (d2 - d6));
        }
        let va = d3 * d6 - d5 * d4;
        if va <= 0.0 && (d4 - d3) >= 0.0 && (d5 - d6) >= 0.0 {
            return self.b + (self.c - self.b) * ((d4 - d3) / ((d4 - d3) + (d5 - d6)));
        }
        let denom = 1.0 / (va + vb + vc);
        self.a + ab * (vb * denom) + ac * (vc * denom)
    }
    /// Möller–Trumbore forward ray intersection parameter and barycentrics.
    #[must_use]
    pub fn ray_intersection(self, ray: Ray3) -> Option<(Real, Vec3)> {
        let e1 = self.b - self.a;
        let e2 = self.c - self.a;
        let h = ray.direction.cross(e2);
        let det = e1.dot(h);
        if det.abs() <= ABS_EPSILON {
            return None;
        }
        let inv = 1.0 / det;
        let s = ray.origin - self.a;
        let u = inv * s.dot(h);
        if !(0.0..=1.0).contains(&u) {
            return None;
        }
        let q = s.cross(e1);
        let v = inv * ray.direction.dot(q);
        if v < 0.0 || u + v > 1.0 {
            return None;
        }
        let t = inv * e2.dot(q);
        if t < 0.0 {
            return None;
        }
        Some((
            t,
            Vec3 {
                x: 1.0 - u - v,
                y: u,
                z: v,
            },
        ))
    }
}

/// Analytic mass properties for a solid 2D disk centered at the origin.
#[must_use]
pub fn disk_mass_properties(radius: Real, density: Real) -> Option<(Real, Real)> {
    if radius > 0.0 && density > 0.0 && radius.is_finite() && density.is_finite() {
        let mass = core::f64::consts::PI as Real * radius * radius * density;
        Some((mass, mass * radius * radius * 0.5))
    } else {
        None
    }
}
/// Analytic mass and diagonal inertia for a solid sphere centered at the origin.
#[must_use]
pub fn sphere_mass_properties(radius: Real, density: Real) -> Option<(Real, Vec3)> {
    if radius > 0.0 && density > 0.0 && radius.is_finite() && density.is_finite() {
        let mass =
            (4.0 / 3.0) * (core::f64::consts::PI as Real) * radius * radius * radius * density;
        let i = 0.4 * mass * radius * radius;
        Some((mass, Vec3 { x: i, y: i, z: i }))
    } else {
        None
    }
}
/// Analytic mass properties for a solid axis-aligned rectangle.
#[must_use]
pub fn rectangle_mass_properties(half: Vec2, density: Real) -> Option<(Real, Real)> {
    if half.x > 0.0 && half.y > 0.0 && half.is_finite() && density > 0.0 && density.is_finite() {
        let w = 2.0 * half.x;
        let h = 2.0 * half.y;
        let m = w * h * density;
        Some((m, m * (w * w + h * h) / 12.0))
    } else {
        None
    }
}
pub mod simd;

/// Analytic mass and diagonal inertia for a solid axis-aligned cuboid.
#[must_use]
pub fn cuboid_mass_properties(half: Vec3, density: Real) -> Option<(Real, Vec3)> {
    if half.x > 0.0
        && half.y > 0.0
        && half.z > 0.0
        && half.is_finite()
        && density > 0.0
        && density.is_finite()
    {
        let x = 2.0 * half.x;
        let y = 2.0 * half.y;
        let z = 2.0 * half.z;
        let m = x * y * z * density;
        Some((
            m,
            Vec3 {
                x: m * (y * y + z * z) / 12.0,
                y: m * (x * x + z * z) / 12.0,
                z: m * (x * x + y * y) / 12.0,
            },
        ))
    } else {
        None
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    #[test]
    fn dimensions_and_rotation() {
        let r = Rot2::from_radians(core::f32::consts::FRAC_PI_2 as Real).unwrap();
        let v = r.inverse().rotate(r.rotate(Vec2::X));
        assert!((v.x - 1.0).abs() < REL_EPSILON);
    }
    #[test]
    fn normalization_fallback() {
        assert_eq!(Vec3::ZERO.normalized_or(Vec3::Y), Vec3::Y);
    }
    #[test]
    fn cross_is_orthogonal() {
        let a = Vec3::new(2.0, 3.0, 4.0).unwrap();
        let b = Vec3::new(-1.0, 2.0, 1.0).unwrap();
        assert!(a.cross(b).dot(a).abs() < REL_EPSILON);
    }
    #[test]
    fn rejects_nan() {
        assert_eq!(Vec2::new(Real::NAN, 0.0), Err(MathError::NonFinite));
    }
    #[test]
    fn transforms_round_trip_and_compose() {
        let a =
            Transform2::new(Vec2 { x: 3.0, y: -2.0 }, Rot2::from_radians(0.7).unwrap()).unwrap();
        let b =
            Transform2::new(Vec2 { x: -1.0, y: 4.0 }, Rot2::from_radians(-0.2).unwrap()).unwrap();
        let p = Vec2 { x: 2.0, y: 5.0 };
        let back = a.inverse().transform_point(a.transform_point(p));
        assert!((back - p).length() < REL_EPSILON * 10.0);
        assert!(
            (a.compose(b).transform_point(p) - a.transform_point(b.transform_point(p))).length()
                < REL_EPSILON * 10.0
        );
    }
    #[test]
    fn matrices_inverse_round_trip() {
        let m = Mat3::from_cols(
            Vec3 {
                x: 2.0,
                y: 0.0,
                z: 1.0,
            },
            Vec3 {
                x: 1.0,
                y: 3.0,
                z: 0.0,
            },
            Vec3 {
                x: 0.0,
                y: 2.0,
                z: 4.0,
            },
        )
        .unwrap();
        let v = Vec3 {
            x: 2.0,
            y: -3.0,
            z: 5.0,
        };
        assert!((m.inverse().unwrap().mul_vec(m.mul_vec(v)) - v).length() < REL_EPSILON * 20.0);
        assert!(
            Mat2::from_cols(Vec2::X, Vec2::X)
                .unwrap()
                .inverse()
                .is_none()
        );
    }
    #[test]
    fn primitives_handle_boundaries() {
        let s = Segment3::new(
            Vec3::ZERO,
            Vec3 {
                x: 2.0,
                y: 0.0,
                z: 0.0,
            },
        )
        .unwrap();
        assert_eq!(
            s.closest_point(Vec3 {
                x: 3.0,
                y: 1.0,
                z: 0.0
            })
            .1,
            1.0
        );
        let plane = Plane3::from_point_normal(Vec3::ZERO, Vec3::Y).unwrap();
        let ray = Ray3::new(
            Vec3 {
                x: 0.0,
                y: 2.0,
                z: 0.0,
            },
            Vec3 {
                x: 0.0,
                y: -2.0,
                z: 0.0,
            },
        )
        .unwrap();
        assert!((plane.ray_t(ray).unwrap() - 2.0).abs() < REL_EPSILON);
        assert!(
            Triangle3::new(
                Vec3::ZERO,
                Vec3::X,
                Vec3 {
                    x: 2.0,
                    y: 0.0,
                    z: 0.0
                }
            )
            .is_err()
        );
    }
    #[test]
    fn seeded_transform_properties_ten_thousand_cases() {
        let seed = 0xA11C_E5EED_u64;
        let mut state = seed;
        let mut next = || {
            state ^= state >> 12;
            state ^= state << 25;
            state ^= state >> 27;
            let u = state.wrapping_mul(0x2545_F491_4F6C_DD1D);
            ((u >> 40) as Real / (1u64 << 24) as Real) * 20.0 - 10.0
        };
        for _ in 0..10_000 {
            let angle = next();
            let p = Vec2 {
                x: next(),
                y: next(),
            };
            let tr = Transform2::new(
                Vec2 {
                    x: next(),
                    y: next(),
                },
                Rot2::from_radians(angle).unwrap(),
            )
            .unwrap();
            let q = tr.inverse().transform_point(tr.transform_point(p));
            assert!(
                (q - p).length() <= 2.0e-4 * (1.0 + p.length()),
                "seed={seed:#x}"
            );
        }
    }
    #[test]
    fn predicates_and_extreme_scales_are_bounded() {
        assert!(collinear2(
            Vec2::ZERO,
            Vec2 { x: 1.0e6, y: 1.0e6 },
            Vec2 {
                x: 2.0e6,
                y: 2.0e6 + 0.01
            }
        ));
        assert!(coplanar3(
            Vec3::ZERO,
            Vec3::X,
            Vec3::Y,
            Vec3 {
                x: 0.2,
                y: 0.3,
                z: 1.0e-7
            }
        ));
        for scale in [1.0e-3, 1.0, 1.0e3] {
            let h = Vec3 {
                x: scale,
                y: 2.0 * scale,
                z: 3.0 * scale,
            };
            let (m, i) = cuboid_mass_properties(h, 2.0).unwrap();
            assert!(m.is_finite() && i.is_finite() && m > 0.0);
        }
        assert!(sphere_mass_properties(Real::INFINITY, 1.0).is_none());
    }
    #[test]
    fn obb_and_triangle_queries() {
        let obb = Obb2::new(
            Vec2 { x: 2.0, y: 3.0 },
            Vec2 { x: 1.0, y: 2.0 },
            Rot2::from_radians(0.5).unwrap(),
        )
        .unwrap();
        assert!(obb.contains(obb.center));
        assert!(
            obb.aabb()
                .overlaps(Aabb2::new(obb.center, obb.center).unwrap())
        );
        let tri = Triangle3::new(Vec3::ZERO, Vec3::X, Vec3::Y).unwrap();
        let p = tri.closest_point(Vec3 {
            x: 0.2,
            y: 0.2,
            z: 4.0,
        });
        assert!(
            (p - Vec3 {
                x: 0.2,
                y: 0.2,
                z: 0.0
            })
            .length()
                < REL_EPSILON
        );
        let ray = Ray3::new(
            Vec3 {
                x: 0.2,
                y: 0.2,
                z: 1.0,
            },
            Vec3 {
                x: 0.0,
                y: 0.0,
                z: -1.0,
            },
        )
        .unwrap();
        assert!((tri.ray_intersection(ray).unwrap().0 - 1.0).abs() < REL_EPSILON);
    }
    #[test]
    fn analytic_mass_references() {
        let (m, i) = disk_mass_properties(2.0, 3.0).unwrap();
        assert!((m - 12.0 * core::f64::consts::PI as Real).abs() < REL_EPSILON * m);
        assert!((i - 2.0 * m).abs() < REL_EPSILON * m);
        let (sm, si) = sphere_mass_properties(1.0, 1.0).unwrap();
        assert!((si.x - 0.4 * sm).abs() < REL_EPSILON);
    }
}
