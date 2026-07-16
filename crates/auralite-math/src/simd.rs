//! Internal SIMD abstraction layer for vector math.
#![allow(missing_docs)]
//!
//! Provides scalar fallback implementations that match the public `Vec2`/`Vec3`
//! API. When target features (SSE2, AVX2, NEON) are detected at compile time
//! or runtime, these implementations can be swapped for SIMD-accelerated versions.
//!
//! **SIMD is never exposed in the public API.** This module is purely internal.
//!
//! ## Architecture
//! - x86-64: SSE2 (baseline), AVX2 (optional, runtime-detected)
//! - ARM64: NEON (baseline)
//! - Scalar fallback (always available, no target features required)
//!
//! ## Current implementation
//! The implementations below are scalar (no unsafe intrinsics). They serve as
//! correctness references and portable fallbacks. When SIMD acceleration is
//! added, the module will use `cfg(target_feature)` and `is_x86_feature_detected!`
//! to dispatch at runtime.
//!
//! ## Determinism
//! All SIMD paths must produce bitwise-identical results to the scalar fallback
//! for equal floating-point inputs. Fast-math flags are never enabled.

use crate::{Real, Vec2, Vec3};

/// Compute `a + b * c` for each component of a 3D vector (fused multiply-add
/// semantics, but implemented as scalar ops in the fallback).
#[inline]
pub fn vec3_mul_add(a: Vec3, b: Vec3, c: Real) -> Vec3 {
    Vec3 {
        x: a.x + b.x * c,
        y: a.y + b.y * c,
        z: a.z + b.z * c,
    }
}

/// Dot product of two 2D vectors.
#[inline]
pub fn vec2_dot(a: Vec2, b: Vec2) -> Real {
    a.x * b.x + a.y * b.y
}

/// Dot product of two 3D vectors.
#[inline]
pub fn vec3_dot(a: Vec3, b: Vec3) -> Real {
    a.x * b.x + a.y * b.y + a.z * b.z
}

/// Cross product of two 3D vectors.
#[inline]
pub fn vec3_cross(a: Vec3, b: Vec3) -> Vec3 {
    Vec3 {
        x: a.y * b.z - a.z * b.y,
        y: a.z * b.x - a.x * b.z,
        z: a.x * b.y - a.y * b.x,
    }
}

/// 2D length squared.
#[inline]
pub fn vec2_length_sq(a: Vec2) -> Real {
    a.x * a.x + a.y * a.y
}

/// 3D length squared.
#[inline]
pub fn vec3_length_sq(a: Vec3) -> Real {
    a.x * a.x + a.y * a.y + a.z * a.z
}

/// Normalize a 3D vector, falling back to `fallback` if near-zero.
#[inline]
pub fn vec3_normalized_or(a: Vec3, fallback: Vec3) -> Vec3 {
    let n2 = vec3_length_sq(a);
    if n2 > crate::ABS_EPSILON * crate::ABS_EPSILON && n2.is_finite() {
        let inv = n2.sqrt().recip();
        Vec3 {
            x: a.x * inv,
            y: a.y * inv,
            z: a.z * inv,
        }
    } else {
        fallback
    }
}

/// Normalize a 2D vector, falling back to `fallback` if near-zero.
#[inline]
pub fn vec2_normalized_or(a: Vec2, fallback: Vec2) -> Vec2 {
    let n2 = vec2_length_sq(a);
    if n2 > crate::ABS_EPSILON * crate::ABS_EPSILON && n2.is_finite() {
        let inv = n2.sqrt().recip();
        Vec2 {
            x: a.x * inv,
            y: a.y * inv,
        }
    } else {
        fallback
    }
}

/// Multiply a 3x3 matrix (column-major) by a 3D vector.
#[inline]
pub fn mat3_mul_vec(x: Vec3, y: Vec3, z: Vec3, v: Vec3) -> Vec3 {
    Vec3 {
        x: x.x * v.x + y.x * v.y + z.x * v.z,
        y: x.y * v.x + y.y * v.y + z.y * v.z,
        z: x.z * v.x + y.z * v.y + z.z * v.z,
    }
}

/// Linearly interpolate between two 3D vectors: `a * (1 - t) + b * t`.
#[inline]
pub fn vec3_lerp(a: Vec3, b: Vec3, t: Real) -> Vec3 {
    Vec3 {
        x: a.x + (b.x - a.x) * t,
        y: a.y + (b.y - a.y) * t,
        z: a.z + (b.z - a.z) * t,
    }
}

// ─── Tests ────────────────────────────────────────────────────────────────────

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn simd_fallback_dot_matches_std() {
        let a = Vec3 {
            x: 1.0,
            y: 2.0,
            z: 3.0,
        };
        let b = Vec3 {
            x: 4.0,
            y: 5.0,
            z: 6.0,
        };
        assert!((vec3_dot(a, b) - a.dot(b)).abs() < 1.0e-6);
    }

    #[test]
    fn simd_fallback_normalize_matches_std() {
        let a = Vec3 {
            x: 3.0,
            y: 4.0,
            z: 0.0,
        };
        let fallback = Vec3::Y;
        let r1 = vec3_normalized_or(a, fallback);
        let r2 = a.normalized_or(fallback);
        assert!((r1 - r2).length() < 1.0e-6);
    }

    #[test]
    fn simd_fallback_cross_matches_std() {
        let a = Vec3 {
            x: 1.0,
            y: 0.0,
            z: 0.0,
        };
        let b = Vec3 {
            x: 0.0,
            y: 1.0,
            z: 0.0,
        };
        assert!((vec3_cross(a, b) - a.cross(b)).length() < 1.0e-6);
    }

    #[test]
    fn simd_fallback_lerp() {
        let a = Vec3::ZERO;
        let b = Vec3 {
            x: 10.0,
            y: 10.0,
            z: 10.0,
        };
        let m = vec3_lerp(a, b, 0.5);
        assert!(
            (m - Vec3 {
                x: 5.0,
                y: 5.0,
                z: 5.0
            })
            .length()
                < 1.0e-6
        );
    }

    #[test]
    fn simd_fallback_mul_add() {
        let a = Vec3 {
            x: 1.0,
            y: 1.0,
            z: 1.0,
        };
        let b = Vec3 {
            x: 2.0,
            y: 3.0,
            z: 4.0,
        };
        let r = vec3_mul_add(a, b, 5.0);
        assert!((r.x - 11.0).abs() < 1.0e-6);
        assert!((r.y - 16.0).abs() < 1.0e-6);
        assert!((r.z - 21.0).abs() < 1.0e-6);
    }
}
