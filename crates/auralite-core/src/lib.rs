//! Stable IDs, generational storage, deterministic RNG, hashing,
//! fixed-step accumulator, and job scheduler abstraction.
#![forbid(unsafe_code)]

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
impl<T> Clone for Handle<T> {
    fn clone(&self) -> Self {
        *self
    }
}
impl<T> Handle<T> {
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
/// Dense-enough generational slot pool with stale-handle rejection.
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
            assert!(
                self.slots.len() < u32::MAX as usize,
                "AuraLite pool exhausted u32 handle space"
            );
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
    /// Gets a value only if generation matches.
    #[must_use]
    pub fn get(&self, h: Handle<T>) -> Option<&T> {
        self.slots
            .get(h.index as usize)
            .filter(|s| s.generation == h.generation)?
            .value
            .as_ref()
    }
    /// Gets a mutable value only if generation matches.
    pub fn get_mut(&mut self, h: Handle<T>) -> Option<&mut T> {
        self.slots
            .get_mut(h.index as usize)
            .filter(|s| s.generation == h.generation)?
            .value
            .as_mut()
    }
    /// Removes a value and increments its generation.
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
    /// Iterates occupied slots in index order.
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
    /// Mutable occupied iteration in index order.
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
    /// Number of live values.
    #[must_use]
    pub fn len(&self) -> usize {
        self.slots.len() - self.free.len()
    }
    /// Whether no values are live.
    #[must_use]
    pub fn is_empty(&self) -> bool {
        self.len() == 0
    }
}
/// Fixed algorithm deterministic random generator (xorshift64*).
#[derive(Clone, Copy, Debug)]
pub struct Rng {
    state: u64,
}
impl Rng {
    /// Creates from a nonzero-normalized seed.
    #[must_use]
    pub fn new(seed: u64) -> Self {
        Self {
            state: if seed == 0 {
                0x9E37_79B9_7F4A_7C15
            } else {
                seed
            },
        }
    }
    /// Next random word.
    pub fn next_u64(&mut self) -> u64 {
        let mut x = self.state;
        x ^= x >> 12;
        x ^= x << 25;
        x ^= x >> 27;
        self.state = x;
        x.wrapping_mul(0x2545_F491_4F6C_DD1D)
    }
    /// Current serializable state.
    #[must_use]
    pub const fn state(self) -> u64 {
        self.state
    }
}
/// Stable FNV-1a byte hash used for replay diagnostics.
#[must_use]
pub fn hash_bytes(bytes: &[u8]) -> u64 {
    let mut h = 0xcbf2_9ce4_8422_2325u64;
    for b in bytes {
        h ^= u64::from(*b);
        h = h.wrapping_mul(0x100_0000_01b3);
    }
    h
}

/// Validated fixed-step and substep settings shared by worlds.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct StepConfig {
    /// Fixed step duration in seconds.
    pub dt: f64,
    /// Number of solver substeps, at least one.
    pub substeps: u16,
    /// Maximum accumulated frame time, protecting against a spiral of death.
    pub max_frame_time: f64,
}
impl StepConfig {
    /// Constructs finite positive settings with `dt <= max_frame_time`.
    pub fn new(dt: f64, substeps: u16, max_frame_time: f64) -> Result<Self, ConfigError> {
        if dt.is_finite()
            && dt > 0.0
            && substeps > 0
            && max_frame_time.is_finite()
            && max_frame_time >= dt
        {
            Ok(Self {
                dt,
                substeps,
                max_frame_time,
            })
        } else {
            Err(ConfigError)
        }
    }
    /// Duration of one substep.
    #[must_use]
    pub fn substep_dt(self) -> f64 {
        self.dt / f64::from(self.substeps)
    }
}
/// Invalid engine configuration.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct ConfigError;
/// Fixed-step accumulator which clamps unusually long frames.
#[derive(Clone, Copy, Debug, Default, PartialEq)]
pub struct FixedAccumulator {
    remainder: f64,
}
impl FixedAccumulator {
    /// Adds finite nonnegative frame time and returns the bounded number of fixed steps.
    pub fn push_frame(&mut self, frame_time: f64, config: StepConfig) -> Result<u32, ConfigError> {
        if !frame_time.is_finite() || frame_time < 0.0 {
            return Err(ConfigError);
        }
        self.remainder += frame_time.min(config.max_frame_time);
        let steps = (self.remainder / config.dt).floor() as u32;
        self.remainder -= f64::from(steps) * config.dt;
        Ok(steps)
    }
    /// Fraction toward the next fixed step, useful only for render interpolation.
    #[must_use]
    pub fn alpha(self, config: StepConfig) -> f64 {
        self.remainder / config.dt
    }
}

/// A unit of parallel work.
#[derive(Clone, Debug)]
pub struct Job {
    /// Unique job index within a batch.
    pub id: u32,
    /// Work function. Takes (job_id, total_jobs, user_data).
    pub work: fn(u32, u32, &mut [u8]),
}

/// Abstract scheduler trait for parallel execution.
pub trait Scheduler {
    /// Run a batch of jobs. All jobs must complete before the call returns.
    fn run_batch(&mut self, jobs: &mut [Job], user_data: &mut [u8]);
}

/// Single-threaded scheduler that runs jobs sequentially.
#[derive(Default)]
pub struct SingleThreadScheduler;

impl Scheduler for SingleThreadScheduler {
    fn run_batch(&mut self, jobs: &mut [Job], user_data: &mut [u8]) {
        let total = jobs.len() as u32;
        for job in jobs {
            (job.work)(job.id, total, user_data);
        }
    }
}

/// No-op scheduler for testing or disabled parallelism.
pub struct NoopScheduler;

impl Scheduler for NoopScheduler {
    fn run_batch(&mut self, _jobs: &mut [Job], _user_data: &mut [u8]) {}
}

#[cfg(test)]
mod tests {
    use super::*;
    #[test]
    fn stale_handle_fails() {
        let mut p = Pool::default();
        let h = p.insert(3);
        assert_eq!(p.remove(h), Some(3));
        let n = p.insert(4);
        assert_ne!(h.generation(), n.generation());
        assert!(p.get(h).is_none());
    }
    #[test]
    fn rng_replays() {
        let mut a = Rng::new(7);
        let mut b = Rng::new(7);
        for _ in 0..1000 {
            assert_eq!(a.next_u64(), b.next_u64());
        }
    }
    #[test]
    fn randomized_generations_never_resurrect_stale_handles() {
        let seed = 0xC0DE_1234_u64;
        let mut rng = Rng::new(seed);
        let mut pool = Pool::default();
        let mut stale = Vec::new();
        for _ in 0..10_000 {
            let h = pool.insert(rng.next_u64());
            assert!(pool.get(h).is_some());
            assert!(pool.remove(h).is_some());
            stale.push(h);
            let replacement = pool.insert(1);
            assert!(pool.get(h).is_none(), "seed={seed:#x}");
            assert!(pool.remove(replacement).is_some());
        }
        assert!(stale.into_iter().all(|h| pool.get(h).is_none()));
    }
    #[test]
    fn fixed_accumulator_clamps_and_rejects_nonfinite() {
        let config = StepConfig::new(1.0 / 60.0, 2, 0.25).unwrap();
        let mut a = FixedAccumulator::default();
        assert_eq!(a.push_frame(1.0, config), Ok(15));
        assert!(a.alpha(config) < 1.0);
        assert_eq!(a.push_frame(f64::NAN, config), Err(ConfigError));
        assert_eq!(config.substep_dt(), 1.0 / 120.0);
    }
}
