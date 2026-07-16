//! Minimal panic-contained C ABI for the verified 2D vertical slice.
#![allow(unsafe_code)]
use auralite_dynamics::World2;
use std::cell::RefCell;
use std::panic::{AssertUnwindSafe, catch_unwind};
use std::sync::{Mutex, OnceLock};
thread_local! {static LAST:RefCell<String>=const{RefCell::new(String::new())};}
static WORLDS: OnceLock<Mutex<Vec<Option<World2>>>> = OnceLock::new();
fn worlds() -> &'static Mutex<Vec<Option<World2>>> {
    WORLDS.get_or_init(|| Mutex::new(Vec::new()))
}
fn boundary(f: impl FnOnce() -> Result<i32, String>) -> i32 {
    match catch_unwind(AssertUnwindSafe(f)) {
        Ok(Ok(v)) => v,
        Ok(Err(e)) => {
            LAST.with(|x| *x.borrow_mut() = e);
            -1
        }
        Err(_) => {
            LAST.with(|x| *x.borrow_mut() = "panic contained".into());
            -2
        }
    }
}
/// Returns `(major << 16) | minor`.
#[unsafe(no_mangle)]
pub extern "C" fn auralite_api_version() -> u32 {
    1 << 16
}
/// Creates a world and writes a nonzero opaque token. Returns zero on success.
///
/// # Safety
/// `out` must be non-null, aligned, and valid for one `u64` write.
#[unsafe(no_mangle)]
pub unsafe extern "C" fn auralite_world2_create(out: *mut u64) -> i32 {
    boundary(|| {
        if out.is_null() {
            return Err("null output".into());
        }
        let mut w = worlds().lock().map_err(|_| "registry poisoned")?;
        let idx = if let Some(i) = w.iter().position(Option::is_none) {
            w[i] = Some(World2::default());
            i
        } else {
            w.push(Some(World2::default()));
            w.len() - 1
        }; /* SAFETY: validated non-null; caller contract requires valid aligned writable pointer. */
        unsafe {
            out.write((idx as u64) + 1);
        }
        Ok(0)
    })
}
/// Steps a world. Returns zero on success.
#[unsafe(no_mangle)]
pub extern "C" fn auralite_world2_step(token: u64, dt: f32) -> i32 {
    boundary(|| {
        let idx = usize::try_from(token.checked_sub(1).ok_or("invalid token")?)
            .map_err(|_| "token range")?;
        let mut w = worlds().lock().map_err(|_| "registry poisoned")?;
        w.get_mut(idx)
            .and_then(Option::as_mut)
            .ok_or("stale token")?
            .step(dt)
            .map_err(|_| "invalid step")?;
        Ok(0)
    })
}
/// Destroys a world token. Returns zero on success.
#[unsafe(no_mangle)]
pub extern "C" fn auralite_world2_destroy(token: u64) -> i32 {
    boundary(|| {
        let idx = usize::try_from(token.checked_sub(1).ok_or("invalid token")?)
            .map_err(|_| "token range")?;
        let mut w = worlds().lock().map_err(|_| "registry poisoned")?;
        let slot = w.get_mut(idx).ok_or("stale token")?;
        if slot.take().is_none() {
            return Err("stale token".into());
        }
        Ok(0)
    })
}
#[cfg(test)]
mod tests {
    use super::*;
    #[test]
    fn lifecycle_and_errors() {
        let mut h = 0; /* SAFETY: valid local output pointer. */
        assert_eq!(unsafe { auralite_world2_create(&raw mut h) }, 0);
        assert_eq!(auralite_world2_step(h, 1.0 / 60.0), 0);
        assert_eq!(auralite_world2_destroy(h), 0);
        assert_eq!(auralite_world2_destroy(h), -1);
    }
}
