# Unsafe inventory
Only `auralite-ffi::auralite_world2_create` uses an unsafe pointer write. It checks null; its documented C contract requires alignment and writable storage. The block has a `SAFETY` comment. `#[unsafe(no_mangle)]` attributes intentionally export four ABI symbols. Pure logic crates use `#![forbid(unsafe_code)]`.
