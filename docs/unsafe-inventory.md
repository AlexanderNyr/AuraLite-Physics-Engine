# Unsafe inventory
- `auralite-ffi`: Uses unsafe pointer writes for C ABI compatibility. Checked for null and documented alignment requirements.
- `auralite-math`: Uses x86_64 intrinsics in `simd.rs` for performance. Protected by `is_x86_feature_detected!` and `target_arch` checks.
- `auralite-core`: Uses `std::slice::from_raw_parts_mut` in `ThreadPoolScheduler` to share user data across threads during job execution. Controlled by `std::thread::scope`.
- `auralite-gpu`: Trait shells for future backend implementation.
