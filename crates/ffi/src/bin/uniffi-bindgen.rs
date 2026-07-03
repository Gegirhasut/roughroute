//! Binding generator entry point (UniFFI's recommended per-project binary).
//!
//! Generate the Kotlin binding from the built library:
//!
//! ```sh
//! cargo build -p roughroute-ffi
//! cargo run -p roughroute-ffi --features cli --bin uniffi-bindgen -- \
//!   generate --library target/debug/libroughroute_ffi.so \
//!   --language kotlin --out-dir bindings/kotlin
//! ```

fn main() {
    uniffi::uniffi_bindgen_main()
}
