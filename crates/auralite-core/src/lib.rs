//! Stable IDs, generational storage, deterministic RNG, hashing,
//! fixed-step accumulator, and job scheduler abstraction.
#![allow(unsafe_code)]

use std::collections::HashMap;

/// Stable, monotonically assigned identity within a world.
#[derive(Clone, Copy, Debug, PartialEq, Eq, PartialOrd, Ord, Hash)]
pub struct StableId(pub u64);

/// Type-safe generational handle.
#[derive(Debug, PartialEq, Eq, Hash)]
pub struct Handle<T> {
    index: u32,
    generation: u32,
    marker: core::marker::PhantomData<fn() -> T>,
}
impl<T> Copy for Handle<T> {}
impl<T> Default for Handle<T> {
    fn default() -> Self {
        Self {
            index: u32::MAX,
            generation: 0,
            marker: core::marker::PhantomData,
        }
    }
}
impl<T> Clone for Handle<T> {
    fn clone(&self) -> Self {
        *self
    }
}
impl<T> Handle<T> {
    /// Creates a new handle.
    pub const fn new(index: u32, generation: u32) -> Self {
        Self { index, generation, marker: core::marker::PhantomData }
    }
    /// Slot index.
    #[must_use]
    pub const fn index(self) -> u32 {
        self.index
    }
    /// Generation counter.
    #[must_use]
    pub const fn generation(self) -> u32 {
        self.generation
    }
    /// Packs as generation:index.
    #[must_use]
    pub const fn packed(self) -> u64 {
        ((self.generation as u64) << 32) | (self.index as u64)
    }
}
struct Slot<T> {
    generation: u32,
    value: Option<T>,
}
/// Generational slot pool.
pub struct Pool<T> {
    slots: Vec<Slot<T>>,
    free: Vec<u32>,
}
impl<T> Default for Pool<T> {
    fn default() -> Self {
        Self {
            slots: Vec::new(),
            free: Vec::new(),
        }
    }
}
impl<T> Pool<T> {
    /// Inserts a value.
    pub fn insert(&mut self, value: T) -> Handle<T> {
        if let Some(index) = self.free.pop() {
            let s = &mut self.slots[index as usize];
            s.value = Some(value);
            Handle {
                index,
                generation: s.generation,
                marker: core::marker::PhantomData,
            }
        } else {
            let index = self.slots.len() as u32;
            self.slots.push(Slot {
                generation: 0,
                value: Some(value),
            });
            Handle {
                index,
                generation: 0,
                marker: core::marker::PhantomData,
            }
        }
    }
    /// Gets a value.
    #[must_use]
    pub fn get(&self, h: Handle<T>) -> Option<&T> {
        self.slots
            .get(h.index as usize)
            .filter(|s| s.generation == h.generation)?
            .value
            .as_ref()
    }
    /// Gets a mutable value.
    pub fn get_mut(&mut self, h: Handle<T>) -> Option<&mut T> {
        self.slots
            .get_mut(h.index as usize)
            .filter(|s| s.generation == h.generation)?
            .value
            .as_mut()
    }
    /// Removes a value.
    pub fn remove(&mut self, h: Handle<T>) -> Option<T> {
        let s = self.slots.get_mut(h.index as usize)?;
        if s.generation != h.generation {
            return None;
        }
        let v = s.value.take()?;
        s.generation = s.generation.wrapping_add(1);
        self.free.push(h.index);
        Some(v)
    }
    /// Iterates values.
    pub fn iter(&self) -> impl Iterator<Item = (Handle<T>, &T)> {
        self.slots.iter().enumerate().filter_map(|(i, s)| {
            s.value.as_ref().map(|v| {
                (
                    Handle {
                        index: i as u32,
                        generation: s.generation,
                        marker: core::marker::PhantomData,
                    },
                    v,
                )
            })
        })
    }
    /// Mutable iterator.
    pub fn iter_mut(&mut self) -> impl Iterator<Item = (Handle<T>, &mut T)> {
        self.slots.iter_mut().enumerate().filter_map(|(i, s)| {
            s.value.as_mut().map(|v| {
                (
                    Handle {
                        index: i as u32,
                        generation: s.generation,
                        marker: core::marker::PhantomData,
                    },
                    v,
                )
            })
        })
    }
    /// Count.
    #[must_use] pub fn len(&self) -> usize { self.slots.len() - self.free.len() }
    /// Empty check.
    #[must_use] pub fn is_empty(&self) -> bool { self.len() == 0 }
}
/// Fixed RNG.
#[derive(Clone, Copy, Debug)]
pub struct Rng { state: u64 }
impl Rng {
    /// New RNG.
    #[must_use] pub fn new(seed: u64) -> Self { Self { state: if seed == 0 { 0x9E37_79B9_7F4A_7C15 } else { seed } } }
    /// Next word.
    pub fn next_u64(&mut self) -> u64 {
        let mut x = self.state;
        x ^= x >> 12; x ^= x << 25; x ^= x >> 27;
        self.state = x; x.wrapping_mul(0x2545_F491_4F6C_DD1D)
    }
    /// Current state.
    #[must_use] pub const fn state(self) -> u64 { self.state }
}
/// FNV hash.
#[must_use] pub fn hash_bytes(bytes: &[u8]) -> u64 {
    let mut h = 0xcbf2_9ce4_8422_2325u64;
    for b in bytes { h ^= u64::from(*b); h = h.wrapping_mul(0x100_0000_01b3); }
    h
}

/// Step configuration.
#[derive(Clone, Copy, Debug, PartialEq)] pub struct StepConfig { 
    /// Fixed dt.
    pub dt: f64, 
    /// Substeps.
    pub substeps: u16, 
    /// Max frame.
    pub max_frame_time: f64 
}
impl StepConfig {
    /// New config.
    pub fn new(dt: f64, substeps: u16, max_frame_time: f64) -> Result<Self, ConfigError> {
        if dt.is_finite() && dt > 0.0 && substeps > 0 && max_frame_time.is_finite() && max_frame_time >= dt { Ok(Self { dt, substeps, max_frame_time }) } else { Err(ConfigError) }
    }
    /// Substep dt.
    #[must_use] pub fn substep_dt(self) -> f64 { self.dt / f64::from(self.substeps) }
}
/// Config error.
#[derive(Clone, Copy, Debug, PartialEq, Eq)] pub struct ConfigError;
/// Accumulator.
#[derive(Clone, Copy, Debug, Default, PartialEq)] pub struct FixedAccumulator { remainder: f64 }
impl FixedAccumulator {
    /// Push frame.
    pub fn push_frame(&mut self, frame_time: f64, config: StepConfig) -> Result<u32, ConfigError> {
        if !frame_time.is_finite() || frame_time < 0.0 { return Err(ConfigError); }
        self.remainder += frame_time.min(config.max_frame_time);
        let steps = (self.remainder / config.dt).floor() as u32;
        self.remainder -= f64::from(steps) * config.dt;
        Ok(steps)
    }
    /// Alpha.
    #[must_use] pub fn alpha(self, config: StepConfig) -> f64 { self.remainder / config.dt }
}

/// Job.
#[derive(Clone, Debug)] pub struct Job { 
    /// Job ID.
    pub id: u32, 
    /// Work fn.
    pub work: fn(u32, u32, &mut [u8]) 
}
/// Scheduler.
pub trait Scheduler { 
    /// Run batch.
    fn run_batch(&mut self, jobs: &mut [Job], user_data: &mut [u8]); 
}
/// Single thread scheduler.
#[derive(Default)] pub struct SingleThreadScheduler;
impl Scheduler for SingleThreadScheduler {
    fn run_batch(&mut self, jobs: &mut [Job], user_data: &mut [u8]) {
        let total = jobs.len() as u32; for job in jobs { (job.work)(job.id, total, user_data); }
    }
}
/// Multi thread scheduler.
#[derive(Default)] pub struct ThreadPoolScheduler;
impl Scheduler for ThreadPoolScheduler {
    fn run_batch(&mut self, jobs: &mut [Job], user_data: &mut [u8]) {
        let total = jobs.len() as u32; let ptr = user_data.as_mut_ptr() as usize; let len = user_data.len();
        std::thread::scope(|s| {
            for job in jobs {
                let f = job.work; let id = job.id;
                s.spawn(move || { let slice = unsafe { std::slice::from_raw_parts_mut(ptr as *mut u8, len) }; (f)(id, total, slice); });
            }
        });
    }
}
/// Noop scheduler.
pub struct NoopScheduler;
impl Scheduler for NoopScheduler { fn run_batch(&mut self, _j: &mut [Job], _d: &mut [u8]) {} }

/// Spatial hash.
#[derive(Clone, Debug)]
pub struct SpatialHash { 
    /// Cell size.
    pub cell_size: auralite_math::Real, 
    cells: HashMap<(i32, i32, i32), Vec<usize>> 
}
impl SpatialHash {
    /// New hash.
    pub fn new(cell_size: auralite_math::Real) -> Self { Self { cell_size: cell_size.max(0.1), cells: HashMap::new() } }
    /// Clear.
    pub fn clear(&mut self) { self.cells.clear(); }
    /// Insert.
    pub fn insert(&mut self, pos: auralite_math::Vec3, index: usize) {
        let k = ((pos.x / self.cell_size).floor() as i32, (pos.y / self.cell_size).floor() as i32, (pos.z / self.cell_size).floor() as i32);
        self.cells.entry(k).or_default().push(index);
    }
    /// Query.
    pub fn query(&self, pos: auralite_math::Vec3, radius: auralite_math::Real) -> Vec<usize> {
        let mut r = Vec::new();
        let min = (( (pos.x - radius) / self.cell_size).floor() as i32, ( (pos.y - radius) / self.cell_size).floor() as i32, ( (pos.z - radius) / self.cell_size).floor() as i32);
        let max = (( (pos.x + radius) / self.cell_size).floor() as i32, ( (pos.y + radius) / self.cell_size).floor() as i32, ( (pos.z + radius) / self.cell_size).floor() as i32);
        for x in min.0..=max.0 { for y in min.1..=max.1 { for z in min.2..=max.2 { if let Some(c) = self.cells.get(&(x, y, z)) { r.extend_from_slice(c); } } } }
        r
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    #[test] fn stale_handle_fails() { let mut p = Pool::default(); let h = p.insert(3); p.remove(h); assert!(p.get(h).is_none()); }
    #[test] fn rng_replays() { let mut a = Rng::new(7); let mut b = Rng::new(7); for _ in 0..100 { assert_eq!(a.next_u64(), b.next_u64()); } }
    #[test] fn hash_replays() { assert_eq!(hash_bytes(b"abc"), hash_bytes(b"abc")); }
}
