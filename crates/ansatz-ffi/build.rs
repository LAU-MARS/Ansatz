//! 由 cbindgen 生成 include/ansatz.h，与源码同步再生。

use std::env;
use std::path::PathBuf;

fn main() {
    let crate_dir = PathBuf::from(env::var("CARGO_MANIFEST_DIR").unwrap());
    let config = cbindgen::Config::from_file(crate_dir.join("cbindgen.toml"))
        .expect("cbindgen.toml 必须存在且合法");
    let header = crate_dir.join("include").join("ansatz.h");
    cbindgen::Builder::new()
        .with_config(config)
        .with_crate(&crate_dir)
        .generate()
        .expect("cbindgen 必须能解析 ansatz-ffi 导出的 C ABI")
        .write_to_file(header);
    println!("cargo:rerun-if-changed=build.rs");
    println!("cargo:rerun-if-changed=cbindgen.toml");
    println!("cargo:rerun-if-changed=src/lib.rs");
}
