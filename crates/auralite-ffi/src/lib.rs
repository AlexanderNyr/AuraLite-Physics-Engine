//! C ABI for AuraLite Physics Engine.
//! Generation-safe opaque tokens, thread-local last-error, panic containment.
#![allow(unsafe_code)]
#![allow(missing_docs, clippy::missing_safety_doc)]

use auralite_dynamics::{World2, World3};
use auralite_math::{Vec2, Vec3};
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
        if out.is_null() { return Err("null output pointer".into()); }
        let mut reg = registry().lock().map_err(|_| "registry poisoned")?;
        let slot_idx = if let Some(i) = reg.iter().position(|s| s.world2.is_none() && s.world3.is_none()) {
            reg[i].world2 = Some(World2::default());
            reg[i].generation = reg[i].generation.wrapping_add(1);
            i
        } else {
            reg.push(WorldSlot { world2: Some(World2::default()), world3: None, generation: 0 });
            reg.len() - 1
        };
        unsafe { out.write((((slot_idx as u64) + 1) << 32) | (reg[slot_idx].generation as u64)); }
        Ok(0)
    })
}

/// Creates a 3D world.
#[unsafe(no_mangle)]
pub unsafe extern "C" fn auralite_world3_create(out: *mut u64) -> i32 {
    boundary(|| {
        if out.is_null() { return Err("null output pointer".into()); }
        let mut reg = registry().lock().map_err(|_| "registry poisoned")?;
        let slot_idx = if let Some(i) = reg.iter().position(|s| s.world2.is_none() && s.world3.is_none()) {
            reg[i].world3 = Some(World3::default());
            reg[i].generation = reg[i].generation.wrapping_add(1);
            i
        } else {
            reg.push(WorldSlot { world2: None, world3: Some(World3::default()), generation: 0 });
            reg.len() - 1
        };
        unsafe { out.write((((slot_idx as u64) + 1) << 32) | (reg[slot_idx].generation as u64)); }
        Ok(0)
    })
}

fn with_world2<F: FnOnce(&mut World2) -> Result<i32, String>>(token: u64, f: F) -> i32 {
    boundary(|| {
        let idx = ((token >> 32) as usize).wrapping_sub(1);
        let mut reg = registry().lock().map_err(|_| "registry poisoned")?;
        let slot = reg.get_mut(idx).ok_or("invalid token")?;
        if slot.generation != (token as u32) { return Err("stale token".into()); }
        let w = slot.world2.as_mut().ok_or("not a world2")?;
        f(w)
    })
}

fn with_world3<F: FnOnce(&mut World3) -> Result<i32, String>>(token: u64, f: F) -> i32 {
    boundary(|| {
        let idx = ((token >> 32) as usize).wrapping_sub(1);
        let mut reg = registry().lock().map_err(|_| "registry poisoned")?;
        let slot = reg.get_mut(idx).ok_or("invalid token")?;
        if slot.generation != (token as u32) { return Err("stale token".into()); }
        let w = slot.world3.as_mut().ok_or("not a world3")?;
        f(w)
    })
}

#[unsafe(no_mangle)]
pub extern "C" fn auralite_world2_step(token: u64, dt: f32) -> i32 {
    with_world2(token, |w| { w.step(dt).map(|_| 0).map_err(|e| format!("{:?}", e)) })
}
#[unsafe(no_mangle)]
pub extern "C" fn auralite_world3_step(token: u64, dt: f32) -> i32 {
    with_world3(token, |w| { w.step(dt).map(|_| 0).map_err(|e| format!("{:?}", e)) })
}

fn auralite_world_destroy(token: u64) -> i32 {
    boundary(|| {
        let idx = ((token >> 32) as usize).wrapping_sub(1);
        let mut reg = registry().lock().map_err(|_| "registry poisoned")?;
        let slot = reg.get_mut(idx).ok_or("invalid token")?;
        if slot.generation != (token as u32) { return Err("stale token".into()); }
        slot.world2 = None; slot.world3 = None;
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
        .map(|r| r.iter().filter(|s| s.world2.is_some() || s.world3.is_some()).count() as u32)
        .unwrap_or(0)
}

/// Canonical C header for drift checking.
pub const CANONICAL_HEADER: &str = r##"#ifndef AURALITE_H
#define AURALITE_H
#include <stdint.h>
#ifdef __cplusplus
extern "C" {
#endif
uint32_t auralite_api_version(void);
uint32_t auralite_abi_version(void);
const char* auralite_last_error(void);
int32_t auralite_world2_create(uint64_t* out);
int32_t auralite_world2_step(uint64_t token, float dt);
int32_t auralite_world2_destroy(uint64_t token);
uint32_t auralite_world_count(void);
#ifdef __cplusplus
}
#endif
#endif /* AURALITE_H */
"##;

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
}
