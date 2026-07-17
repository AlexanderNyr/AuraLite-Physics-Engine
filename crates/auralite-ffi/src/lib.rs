//! C ABI for AuraLite Physics Engine.
//! Generation-safe opaque tokens, thread-local last-error, panic containment.
#![allow(unsafe_code)]
#![allow(missing_docs, clippy::missing_safety_doc)]

use auralite_dynamics::{World2, World3};
use std::cell::RefCell;
use std::panic::{AssertUnwindSafe, catch_unwind};
use std::sync::{Mutex, OnceLock};

thread_local! { static LAST_ERROR: RefCell<Vec<u8>> = const { RefCell::new(Vec::new()) }; }

fn set_error(msg: &str) {
    LAST_ERROR.with(|cell| *cell.borrow_mut() = msg.as_bytes().to_vec());
}

/// Returns pointer to null-terminated UTF-8 error string. Empty/null = no error.
/// Valid until next FFI call on this thread.
#[unsafe(no_mangle)]
pub extern "C" fn auralite_last_error() -> *const u8 {
    LAST_ERROR.with(|cell| {
        let bytes = cell.borrow();
        if bytes.is_empty() {
            std::ptr::null()
        } else {
            bytes.as_ptr()
        }
    })
}

struct WorldSlot {
    world2: Option<World2>,
    world3: Option<World3>,
    generation: u32,
}
static REGISTRY: OnceLock<Mutex<Vec<WorldSlot>>> = OnceLock::new();
fn registry() -> &'static Mutex<Vec<WorldSlot>> {
    REGISTRY.get_or_init(|| Mutex::new(Vec::new()))
}

fn boundary<F: FnOnce() -> Result<i32, String>>(f: F) -> i32 {
    match catch_unwind(AssertUnwindSafe(f)) {
        Ok(Ok(v)) => v,
        Ok(Err(e)) => {
            set_error(&e);
            -1
        }
        Err(_) => {
            set_error("panic contained");
            -2
        }
    }
}

#[unsafe(no_mangle)]
pub extern "C" fn auralite_api_version() -> u32 {
    (1u32 << 16) | 1
}
#[unsafe(no_mangle)]
pub extern "C" fn auralite_abi_version() -> u32 {
    1
}

/// Creates a 2D world.
#[unsafe(no_mangle)]
pub unsafe extern "C" fn auralite_world2_create(out: *mut u64) -> i32 {
    boundary(|| {
        if out.is_null() {
            return Err("null output pointer".into());
        }
        let mut reg = registry().lock().map_err(|_| "registry poisoned")?;
        let slot_idx = if let Some(i) = reg
            .iter()
            .position(|s| s.world2.is_none() && s.world3.is_none())
        {
            reg[i].world2 = Some(World2::default());
            reg[i].generation = reg[i].generation.wrapping_add(1);
            i
        } else {
            reg.push(WorldSlot {
                world2: Some(World2::default()),
                world3: None,
                generation: 0,
            });
            reg.len() - 1
        };
        unsafe {
            out.write((((slot_idx as u64) + 1) << 32) | (reg[slot_idx].generation as u64));
        }
        Ok(0)
    })
}

/// Creates a 3D world.
#[unsafe(no_mangle)]
pub unsafe extern "C" fn auralite_world3_create(out: *mut u64) -> i32 {
    boundary(|| {
        if out.is_null() {
            return Err("null output pointer".into());
        }
        let mut reg = registry().lock().map_err(|_| "registry poisoned")?;
        let slot_idx = if let Some(i) = reg
            .iter()
            .position(|s| s.world2.is_none() && s.world3.is_none())
        {
            reg[i].world3 = Some(World3::default());
            reg[i].generation = reg[i].generation.wrapping_add(1);
            i
        } else {
            reg.push(WorldSlot {
                world2: None,
                world3: Some(World3::default()),
                generation: 0,
            });
            reg.len() - 1
        };
        unsafe {
            out.write((((slot_idx as u64) + 1) << 32) | (reg[slot_idx].generation as u64));
        }
        Ok(0)
    })
}

fn with_world2<F: FnOnce(&mut World2) -> Result<i32, String>>(token: u64, f: F) -> i32 {
    boundary(|| {
        let idx = ((token >> 32) as usize).wrapping_sub(1);
        let mut reg = registry().lock().map_err(|_| "registry poisoned")?;
        let slot = reg.get_mut(idx).ok_or("invalid token")?;
        if slot.generation != (token as u32) {
            return Err("stale token".into());
        }
        let w = slot.world2.as_mut().ok_or("not a world2")?;
        f(w)
    })
}

fn with_world3<F: FnOnce(&mut World3) -> Result<i32, String>>(token: u64, f: F) -> i32 {
    boundary(|| {
        let idx = ((token >> 32) as usize).wrapping_sub(1);
        let mut reg = registry().lock().map_err(|_| "registry poisoned")?;
        let slot = reg.get_mut(idx).ok_or("invalid token")?;
        if slot.generation != (token as u32) {
            return Err("stale token".into());
        }
        let w = slot.world3.as_mut().ok_or("not a world3")?;
        f(w)
    })
}

#[unsafe(no_mangle)]
pub extern "C" fn auralite_world2_step(token: u64, dt: f32) -> i32 {
    with_world2(token, |w| {
        w.step(dt).map(|_| 0).map_err(|e| format!("{:?}", e))
    })
}
#[unsafe(no_mangle)]
pub extern "C" fn auralite_world3_step(token: u64, dt: f32) -> i32 {
    with_world3(token, |w| {
        w.step(dt).map(|_| 0).map_err(|e| format!("{:?}", e))
    })
}

fn auralite_world_destroy(token: u64) -> i32 {
    boundary(|| {
        let idx = ((token >> 32) as usize).wrapping_sub(1);
        let mut reg = registry().lock().map_err(|_| "registry poisoned")?;
        let slot = reg.get_mut(idx).ok_or("invalid token")?;
        if slot.generation != (token as u32) {
            return Err("stale token".into());
        }
        slot.world2 = None;
        slot.world3 = None;
        slot.generation = slot.generation.wrapping_add(1);
        Ok(0)
    })
}

#[unsafe(no_mangle)]
pub extern "C" fn auralite_world2_destroy(token: u64) -> i32 {
    auralite_world_destroy(token)
}

#[unsafe(no_mangle)]
pub extern "C" fn auralite_world3_destroy(token: u64) -> i32 {
    auralite_world_destroy(token)
}

#[unsafe(no_mangle)]
pub extern "C" fn auralite_world_count() -> u32 {
    registry()
        .lock()
        .map(|r| {
            r.iter()
                .filter(|s| s.world2.is_some() || s.world3.is_some())
                .count() as u32
        })
        .unwrap_or(0)
}

/// Canonical C header for drift checking.
pub const CANONICAL_HEADER: &str = r##"#ifndef AURALITE_H
#define AURALITE_H
#include <stdint.h>
#ifdef __cplusplus
extern "C" {
#endif
typedef void (*AuraliteLogCallback)(uint32_t level, const char* msg);
typedef void (*AuraliteDebugDrawLineCallback)(float x1, float y1, float z1, float x2, float y2, float z2, uint32_t color_rgb);

uint32_t auralite_api_version(void);
uint32_t auralite_abi_version(void);
const char* auralite_last_error(void);
int32_t auralite_set_log_callback(AuraliteLogCallback cb);
int32_t auralite_set_debug_draw_line_callback(AuraliteDebugDrawLineCallback cb);
int32_t auralite_world2_create(uint64_t* out);
int32_t auralite_world3_create(uint64_t* out);
int32_t auralite_world2_step(uint64_t token, float dt);
int32_t auralite_world3_step(uint64_t token, float dt);
int32_t auralite_world2_destroy(uint64_t token);
int32_t auralite_world3_destroy(uint64_t token);
uint32_t auralite_world_count(void);
int32_t auralite_world2_add_body(uint64_t token, uint8_t kind, float px, float py, float vx, float vy, float mass, uint64_t* out_body_id);
int32_t auralite_world3_add_body(uint64_t token, uint8_t kind, float px, float py, float pz, float vx, float vy, float vz, float mass, uint64_t* out_body_id);
int32_t auralite_world2_body_query(uint64_t token, uint64_t body_id, float* out_px, float* out_py, float* out_vx, float* out_vy, uint8_t* out_sleeping);
int32_t auralite_world3_body_query(uint64_t token, uint64_t body_id, float* out_px, float* out_py, float* out_pz, float* out_vx, float* out_vy, float* out_vz, uint8_t* out_sleeping);
int32_t auralite_world2_body_apply_impulse(uint64_t token, uint64_t body_id, float ix, float iy);
int32_t auralite_world3_body_apply_impulse(uint64_t token, uint64_t body_id, float ix, float iy, float iz);
int32_t auralite_world3_batch_query_positions(uint64_t token, const uint64_t* body_ids, uint32_t count, float* out_positions);
#ifdef __cplusplus
}
#endif
#endif /* AURALITE_H */
"##;

pub type AuraliteLogCallback = extern "C" fn(level: u32, msg: *const u8);
pub type AuraliteDebugDrawLineCallback =
    extern "C" fn(x1: f32, y1: f32, z1: f32, x2: f32, y2: f32, z2: f32, color_rgb: u32);

static LOG_CALLBACK: OnceLock<Mutex<Option<AuraliteLogCallback>>> = OnceLock::new();
static DRAW_LINE_CALLBACK: OnceLock<Mutex<Option<AuraliteDebugDrawLineCallback>>> = OnceLock::new();

#[unsafe(no_mangle)]
pub extern "C" fn auralite_set_log_callback(cb: Option<AuraliteLogCallback>) -> i32 {
    boundary(|| {
        let mut l = LOG_CALLBACK
            .get_or_init(|| Mutex::new(None))
            .lock()
            .map_err(|_| "poisoned")?;
        *l = cb;
        Ok(0)
    })
}

#[unsafe(no_mangle)]
pub extern "C" fn auralite_set_debug_draw_line_callback(
    cb: Option<AuraliteDebugDrawLineCallback>,
) -> i32 {
    boundary(|| {
        let mut l = DRAW_LINE_CALLBACK
            .get_or_init(|| Mutex::new(None))
            .lock()
            .map_err(|_| "poisoned")?;
        *l = cb;
        Ok(0)
    })
}

#[unsafe(no_mangle)]
pub unsafe extern "C" fn auralite_world2_add_body(
    token: u64,
    kind: u8,
    px: f32,
    py: f32,
    vx: f32,
    vy: f32,
    mass: f32,
    out_body_id: *mut u64,
) -> i32 {
    with_world2(token, |w| {
        if out_body_id.is_null() {
            return Err("null output pointer".into());
        }
        let bkind = match kind {
            0 => auralite_dynamics::BodyType::Static,
            1 => auralite_dynamics::BodyType::Kinematic,
            _ => auralite_dynamics::BodyType::Dynamic,
        };
        let mut b = auralite_dynamics::BodyBuilder2::dynamic()
            .position(auralite_math::Vec2 {
                x: px as auralite_math::Real,
                y: py as auralite_math::Real,
            })
            .velocity(auralite_math::Vec2 {
                x: vx as auralite_math::Real,
                y: vy as auralite_math::Real,
            })
            .mass(mass as auralite_math::Real);
        b.kind = bkind;
        let h = w.add_body(b).map_err(|_| "add body failed")?;
        unsafe {
            *out_body_id = (((h.index() as u64) + 1) << 32) | (h.generation() as u64);
        }
        Ok(0)
    })
}

#[unsafe(no_mangle)]
pub unsafe extern "C" fn auralite_world3_add_body(
    token: u64,
    kind: u8,
    px: f32,
    py: f32,
    pz: f32,
    vx: f32,
    vy: f32,
    vz: f32,
    mass: f32,
    out_body_id: *mut u64,
) -> i32 {
    with_world3(token, |w| {
        if out_body_id.is_null() {
            return Err("null output pointer".into());
        }
        let bkind = match kind {
            0 => auralite_dynamics::BodyType::Static,
            1 => auralite_dynamics::BodyType::Kinematic,
            _ => auralite_dynamics::BodyType::Dynamic,
        };
        let mut b = auralite_dynamics::BodyBuilder3::dynamic()
            .position(auralite_math::Vec3 {
                x: px as auralite_math::Real,
                y: py as auralite_math::Real,
                z: pz as auralite_math::Real,
            })
            .velocity(auralite_math::Vec3 {
                x: vx as auralite_math::Real,
                y: vy as auralite_math::Real,
                z: vz as auralite_math::Real,
            })
            .mass(mass as auralite_math::Real);
        b.kind = bkind;
        let h = w.add_body(b).map_err(|_| "add body failed")?;
        unsafe {
            *out_body_id = (((h.index() as u64) + 1) << 32) | (h.generation() as u64);
        }
        Ok(0)
    })
}

#[allow(clippy::unnecessary_cast)]
#[unsafe(no_mangle)]
pub unsafe extern "C" fn auralite_world2_body_query(
    token: u64,
    body_id: u64,
    out_px: *mut f32,
    out_py: *mut f32,
    out_vx: *mut f32,
    out_vy: *mut f32,
    out_sleeping: *mut u8,
) -> i32 {
    with_world2(token, |w| {
        let h = auralite_dynamics::BodyHandle2::new(
            ((body_id >> 32) as u32).wrapping_sub(1),
            body_id as u32,
        );
        let b = w.body(h).map_err(|_| "invalid body handle")?;
        unsafe {
            if !out_px.is_null() {
                *out_px = b.position.x as f32;
            }
            if !out_py.is_null() {
                *out_py = b.position.y as f32;
            }
            if !out_vx.is_null() {
                *out_vx = b.velocity.x as f32;
            }
            if !out_vy.is_null() {
                *out_vy = b.velocity.y as f32;
            }
            if !out_sleeping.is_null() {
                *out_sleeping = u8::from(b.sleeping);
            }
        }
        Ok(0)
    })
}

#[allow(clippy::unnecessary_cast)]
#[unsafe(no_mangle)]
pub unsafe extern "C" fn auralite_world3_body_query(
    token: u64,
    body_id: u64,
    out_px: *mut f32,
    out_py: *mut f32,
    out_pz: *mut f32,
    out_vx: *mut f32,
    out_vy: *mut f32,
    out_vz: *mut f32,
    out_sleeping: *mut u8,
) -> i32 {
    with_world3(token, |w| {
        let h = auralite_dynamics::BodyHandle3::new(
            ((body_id >> 32) as u32).wrapping_sub(1),
            body_id as u32,
        );
        let b = w.body(h).map_err(|_| "invalid body handle")?;
        unsafe {
            if !out_px.is_null() {
                *out_px = b.position.x as f32;
            }
            if !out_py.is_null() {
                *out_py = b.position.y as f32;
            }
            if !out_pz.is_null() {
                *out_pz = b.position.z as f32;
            }
            if !out_vx.is_null() {
                *out_vx = b.velocity.x as f32;
            }
            if !out_vy.is_null() {
                *out_vy = b.velocity.y as f32;
            }
            if !out_vz.is_null() {
                *out_vz = b.velocity.z as f32;
            }
            if !out_sleeping.is_null() {
                *out_sleeping = u8::from(b.sleeping);
            }
        }
        Ok(0)
    })
}

#[unsafe(no_mangle)]
pub extern "C" fn auralite_world2_body_apply_impulse(
    token: u64,
    body_id: u64,
    ix: f32,
    iy: f32,
) -> i32 {
    with_world2(token, |w| {
        let h = auralite_dynamics::BodyHandle2::new(
            ((body_id >> 32) as u32).wrapping_sub(1),
            body_id as u32,
        );
        w.apply_impulse(
            h,
            auralite_math::Vec2 {
                x: ix as auralite_math::Real,
                y: iy as auralite_math::Real,
            },
        )
        .map(|_| 0)
        .map_err(|_| "invalid body handle".to_string())
    })
}

#[unsafe(no_mangle)]
pub extern "C" fn auralite_world3_body_apply_impulse(
    token: u64,
    body_id: u64,
    ix: f32,
    iy: f32,
    iz: f32,
) -> i32 {
    with_world3(token, |w| {
        let h = auralite_dynamics::BodyHandle3::new(
            ((body_id >> 32) as u32).wrapping_sub(1),
            body_id as u32,
        );
        w.apply_impulse(
            h,
            auralite_math::Vec3 {
                x: ix as auralite_math::Real,
                y: iy as auralite_math::Real,
                z: iz as auralite_math::Real,
            },
        )
        .map(|_| 0)
        .map_err(|_| "invalid body handle".to_string())
    })
}

#[allow(clippy::unnecessary_cast)]
#[unsafe(no_mangle)]
pub unsafe extern "C" fn auralite_world3_batch_query_positions(
    token: u64,
    body_ids: *const u64,
    count: u32,
    out_positions: *mut f32,
) -> i32 {
    with_world3(token, |w| {
        if body_ids.is_null() || out_positions.is_null() {
            return Err("null pointer".into());
        }
        unsafe {
            let ids = std::slice::from_raw_parts(body_ids, count as usize);
            let out = std::slice::from_raw_parts_mut(out_positions, (count as usize) * 3);
            for (i, &bid) in ids.iter().enumerate() {
                let h = auralite_dynamics::BodyHandle3::new(
                    ((bid >> 32) as u32).wrapping_sub(1),
                    bid as u32,
                );
                if let Ok(b) = w.body(h) {
                    out[i * 3] = b.position.x as f32;
                    out[i * 3 + 1] = b.position.y as f32;
                    out[i * 3 + 2] = b.position.z as f32;
                } else {
                    out[i * 3] = f32::NAN;
                    out[i * 3 + 1] = f32::NAN;
                    out[i * 3 + 2] = f32::NAN;
                }
            }
        }
        Ok(0)
    })
}

#[unsafe(no_mangle)]
pub extern "C" fn auralite_header_string() -> *const u8 {
    CANONICAL_HEADER.as_ptr()
}

#[unsafe(no_mangle)]
pub unsafe extern "C" fn auralite_verify_header(header: *const u8, len: u32) -> i32 {
    boundary(|| {
        if header.is_null() {
            return Err("null header".into());
        }
        // SAFETY: caller guarantees valid pointer per `unsafe` contract
        let slice = unsafe { std::slice::from_raw_parts(header, len as usize) };
        let given = std::str::from_utf8(slice).map_err(|_| "not utf-8")?;
        if given.trim() == CANONICAL_HEADER.trim() {
            Ok(0)
        } else {
            Err("header mismatch".into())
        }
    })
}

// ─── Tests ───────────────────────────────────────────────────────────────────

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn lifecycle() {
        let mut token: u64 = 0;
        assert_eq!(unsafe { auralite_world2_create(&raw mut token) }, 0);
        assert_ne!(token, 0);
        assert_eq!(auralite_world2_step(token, 1.0 / 60.0), 0);
        assert_eq!(auralite_world2_destroy(token), 0);
        assert_eq!(auralite_world2_destroy(token), -1, "double destroy fails");
    }

    #[test]
    fn stale_token_rejected() {
        let mut t1: u64 = 0;
        let mut t2: u64 = 0;
        assert_eq!(unsafe { auralite_world2_create(&raw mut t1) }, 0);
        assert_eq!(unsafe { auralite_world2_create(&raw mut t2) }, 0);
        assert_eq!(auralite_world2_destroy(t1), 0);
        assert_eq!(
            auralite_world2_step(t1, 0.016),
            -1,
            "stale token should fail"
        );
    }

    #[test]
    fn version_queries() {
        assert_eq!(auralite_api_version() >> 16, 1);
        assert_eq!(auralite_abi_version(), 1);
    }

    #[test]
    fn header_self_verify() {
        let h = CANONICAL_HEADER.as_bytes();
        assert_eq!(
            unsafe { auralite_verify_header(h.as_ptr(), h.len() as u32) },
            0
        );
    }

    #[test]
    fn world_count_tracking() {
        let before = auralite_world_count();
        let mut t: u64 = 0;
        assert_eq!(unsafe { auralite_world2_create(&raw mut t) }, 0);
        assert_eq!(auralite_world_count(), before + 1);
        auralite_world2_destroy(t);
        assert_eq!(auralite_world_count(), before);
    }

    #[test]
    fn ffi_world2_add_and_query_body() {
        let mut token: u64 = 0;
        assert_eq!(unsafe { auralite_world2_create(&raw mut token) }, 0);
        let mut body_id: u64 = 0;
        assert_eq!(
            unsafe {
                auralite_world2_add_body(token, 2, 10.0, 20.0, 1.0, 2.0, 5.0, &raw mut body_id)
            },
            0
        );
        assert_ne!(body_id, 0);

        let mut px = 0.0f32;
        let mut py = 0.0f32;
        let mut vx = 0.0f32;
        let mut vy = 0.0f32;
        let mut sleep = 0u8;
        assert_eq!(
            unsafe {
                auralite_world2_body_query(
                    token,
                    body_id,
                    &raw mut px,
                    &raw mut py,
                    &raw mut vx,
                    &raw mut vy,
                    &raw mut sleep,
                )
            },
            0
        );
        assert_eq!(px, 10.0);
        assert_eq!(py, 20.0);
        assert_eq!(vx, 1.0);
        assert_eq!(vy, 2.0);

        assert_eq!(
            auralite_world2_body_apply_impulse(token, body_id, 5.0, 10.0),
            0
        );
        assert_eq!(auralite_world2_destroy(token), 0);
    }

    #[test]
    fn ffi_world3_add_and_batch_query_bodies() {
        let mut token: u64 = 0;
        assert_eq!(unsafe { auralite_world3_create(&raw mut token) }, 0);
        let mut b1: u64 = 0;
        let mut b2: u64 = 0;
        assert_eq!(
            unsafe {
                auralite_world3_add_body(token, 2, 1.0, 2.0, 3.0, 0.0, 0.0, 0.0, 1.0, &raw mut b1)
            },
            0
        );
        assert_eq!(
            unsafe {
                auralite_world3_add_body(token, 2, 4.0, 5.0, 6.0, 0.0, 0.0, 0.0, 1.0, &raw mut b2)
            },
            0
        );

        let ids = [b1, b2];
        let mut out_positions = [0.0f32; 6];
        assert_eq!(
            unsafe {
                auralite_world3_batch_query_positions(
                    token,
                    ids.as_ptr(),
                    2,
                    out_positions.as_mut_ptr(),
                )
            },
            0
        );
        assert_eq!(out_positions[0], 1.0);
        assert_eq!(out_positions[1], 2.0);
        assert_eq!(out_positions[2], 3.0);
        assert_eq!(out_positions[3], 4.0);
        assert_eq!(out_positions[4], 5.0);
        assert_eq!(out_positions[5], 6.0);
        assert_eq!(auralite_world3_destroy(token), 0);
    }
}
