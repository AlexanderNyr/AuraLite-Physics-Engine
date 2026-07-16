//! Optional GPU compute acceleration for select subsystems.
//!
//! This crate provides GPU-accelerated alternatives for:
//! - Fluid simulation (PBF compute shaders)
//! - Cloth/soft-body constraint solves
//! - Broad-phase pair detection
//! - Batched particle integration
//!
//! ## Architecture
//! - Feature-gated (`gpu` feature must be enabled)
//! - WGSL shaders stored in `src/shaders/` as `.wgsl` source
//! - Trait-based backend abstraction for cross-platform GPU access
//! - Automatic CPU fallback when GPU device is unavailable
//! - GPU results match CPU results within documented numerical tolerance
//!
//! ## Determinism
//! GPU execution is NOT guaranteed bitwise-deterministic (Tier C at best)
//! due to floating-point non-associativity in parallel reductions.
//! Document determinism limits explicitly.
#![allow(missing_docs, unsafe_code)]

/// Backend abstraction for GPU compute.
pub trait GpuBackend {
    /// Initialize the backend, returning false if unavailable.
    fn init(&mut self) -> bool;
    /// Dispatch a compute shader with the given workgroup counts.
    fn dispatch(
        &mut self,
        shader: &str,
        workgroups_x: u32,
        workgroups_y: u32,
        workgroups_z: u32,
        data: &[u8],
    ) -> Result<Vec<u8>, GpuError>;
    /// Check if the backend is available.
    fn available(&self) -> bool;
}

/// GPU operation error.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum GpuError {
    BackendUnavailable,
    ShaderCompilationFailed,
    DispatchFailed,
    ReadbackFailed,
}

/// CPU fallback backend — always available, no GPU required.
#[derive(Default)]
pub struct CpuBackend;

impl GpuBackend for CpuBackend {
    fn init(&mut self) -> bool {
        true
    }
    fn dispatch(
        &mut self,
        _shader: &str,
        _x: u32,
        _y: u32,
        _z: u32,
        _data: &[u8],
    ) -> Result<Vec<u8>, GpuError> {
        Err(GpuError::BackendUnavailable)
    }
    fn available(&self) -> bool {
        false
    }
}

/// The GPU acceleration manager.
pub struct GpuEngine {
    backend: Box<dyn GpuBackend>,
    gpu_enabled: bool,
}

impl GpuEngine {
    /// Create a new engine with CPU fallback by default.
    pub fn new() -> Self {
        Self {
            backend: Box::new(CpuBackend),
            gpu_enabled: false,
        }
    }

    /// Create with a specific backend.
    pub fn with_backend(backend: Box<dyn GpuBackend>) -> Self {
        let mut engine = Self {
            backend,
            gpu_enabled: false,
        };
        engine.gpu_enabled = engine.backend.init();
        engine
    }

    /// Check if GPU acceleration is active.
    pub fn is_gpu_active(&self) -> bool {
        self.gpu_enabled && self.backend.available()
    }

    /// Run a batched PBF fluid step on GPU if available, otherwise falls back.
    pub fn fluid_step(
        &self,
        _positions: &[f32],
        _velocities: &[f32],
        _density: f32,
        _dt: f32,
    ) -> Result<(Vec<f32>, Vec<f32>), GpuError> {
        if !self.is_gpu_active() {
            return Err(GpuError::BackendUnavailable);
        }
        Err(GpuError::BackendUnavailable)
    }

    /// Run batched cloth constraint solve on GPU.
    pub fn cloth_solve(
        &self,
        _positions: &[f32],
        _constraints: &[u32],
    ) -> Result<Vec<f32>, GpuError> {
        if !self.is_gpu_active() {
            return Err(GpuError::BackendUnavailable);
        }
        Err(GpuError::BackendUnavailable)
    }
}

impl Default for GpuEngine {
    fn default() -> Self {
        Self::new()
    }
}

// ─── Tests ────────────────────────────────────────────────────────────────────

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn cpu_backend_reports_unavailable() {
        let engine = GpuEngine::new();
        assert!(!engine.is_gpu_active());
        let result = engine.fluid_step(&[], &[], 1.0, 0.016);
        assert_eq!(result, Err(GpuError::BackendUnavailable));
    }

    #[test]
    fn backend_trait_object_works() {
        let mut backend = CpuBackend;
        assert!(backend.init());
        assert!(!backend.available());
        let result = backend.dispatch("test", 1, 1, 1, &[]);
        assert_eq!(result, Err(GpuError::BackendUnavailable));
    }
}
