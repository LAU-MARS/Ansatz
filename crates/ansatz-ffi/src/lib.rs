//! Ansatz C ABI 壳：稳定 FFI 契约，供 HarmonyOS NAPI 及任意 C/C++ 宿主使用。
//!
//! 只做类型转换与错误映射，不含数学逻辑（求解全部在 ansatz-core）。
//!
//! ## 契约
//!
//! ```c
//! char* ansatz_solve_json(const char* model_json);  // 返回 malloc 风格的 JSON 信封字符串
//! void  ansatz_string_free(char* s);                 // 释放 ansatz_solve_json 的返回值
//! const char* ansatz_version(void);                  // 静态字符串，禁止 free
//! ```
//!
//! - `ansatz_solve_json` 的返回值**永远**是合法 JSON（UTF-8、NUL 结尾）：
//!   `{"ok":true,"result":…}` 或 `{"ok":false,"error":{"kind":…,"message":…}}`。
//!   求解失败（Inconsistent 等）也是 `ok:true`——那是求解层结论，不是工具错误。
//!   仅当返回 `NULL` 时表示内存分配失败等极端情形。
//! - **所有 extern "C" 函数内部用 `catch_unwind` 包裹**：Rust panic 绝不
//!   unwind 穿越 FFI 边界（那是 UB），而是被转换为 `panic` 错误信封。
//! - 内存所有权：返回的 `char*` 归宿主所有，用完必须调 `ansatz_string_free`。

use std::ffi::{CStr, CString};
use std::os::raw::c_char;
use std::panic::{catch_unwind, AssertUnwindSafe};
use std::sync::OnceLock;

use ansatz_core::{solve as solve_core, Envelope, Model};

fn envelope_to_c_string(envelope: &Envelope) -> *mut c_char {
    let json = serde_json::to_string(envelope).unwrap_or_else(|_| {
        // 理论上不可达（报告类型无不可序列化字段），防御性兜底
        r#"{"ok":false,"error":{"kind":"serialization","message":"internal serialization failure"}}"#
            .to_string()
    });
    // 内嵌 NUL 会让 CString 构造失败（理论上不可能，JSON 无 NUL），此时返回 NULL
    match CString::new(json) {
        Ok(cstring) => cstring.into_raw(),
        Err(_) => std::ptr::null_mut(),
    }
}

/// 求解模型。`input` 为符合 schema/model.schema.json 的 NUL 结尾 UTF-8 JSON。
///
/// 返回值：JSON 信封字符串（归宿主所有，用 `ansatz_string_free` 释放），
/// 极端分配失败时返回 NULL。
// C ABI 契约（而非 Rust unsafe 契约）：宿主保证传入有效 NUL 结尾指针；
// 这类「安全 C 函数 + 裸指针入参」是 FFI 的标准形态，故豁免该 lint。
#[no_mangle]
#[allow(clippy::not_unsafe_ptr_arg_deref)]
pub extern "C" fn ansatz_solve_json(input: *const c_char) -> *mut c_char {
    catch_unwind(AssertUnwindSafe(|| {
        if input.is_null() {
            return envelope_to_c_string(&Envelope::error(
                "invalid_argument",
                "input 是空指针，期望 NUL 结尾的模型 JSON 字符串。",
            ));
        }
        let json = match unsafe { CStr::from_ptr(input) }.to_str() {
            Ok(json) => json,
            Err(_) => {
                return envelope_to_c_string(&Envelope::error(
                    "invalid_utf8",
                    "input 不是合法 UTF-8。",
                ))
            }
        };
        let model: Model = match serde_json::from_str(json) {
            Ok(model) => model,
            Err(e) => {
                return envelope_to_c_string(&Envelope::error(
                    "invalid_json",
                    &format!("输入不是合法的模型 JSON：{e}"),
                ))
            }
        };
        match solve_core(&model) {
            Ok(report) => envelope_to_c_string(&Envelope::ok(report)),
            Err(e) => envelope_to_c_string(&Envelope::from_solve_error(&e)),
        }
    }))
    .unwrap_or_else(|_| {
        // panic 拦截在 FFI 边界内：转换为错误信封，绝不 unwind 穿越 extern "C"
        let fallback =
            r#"{"ok":false,"error":{"kind":"panic","message":"内部 panic 已在 FFI 边界内拦截"}}"#;
        CString::new(fallback)
            .map(CString::into_raw)
            .unwrap_or(std::ptr::null_mut())
    })
}

/// 释放 `ansatz_solve_json` 返回的字符串。传 NULL 是安全的 no-op。
// 同上：契约要求只传入本库返回的指针（C ABI 标准形态），豁免该 lint。
#[no_mangle]
#[allow(clippy::not_unsafe_ptr_arg_deref)]
pub extern "C" fn ansatz_string_free(s: *mut c_char) {
    // CString::from_raw 对野指针是 UB，但本函数契约上只接收本库返回的指针；
    // catch_unwind 仅为满足「所有 extern "C" 均包裹」的纪律要求。
    let _ = catch_unwind(AssertUnwindSafe(|| {
        if !s.is_null() {
            drop(unsafe { CString::from_raw(s) });
        }
    }));
}

/// 版本号（静态字符串，禁止 free）。
#[no_mangle]
pub extern "C" fn ansatz_version() -> *const c_char {
    static VERSION: OnceLock<CString> = OnceLock::new();
    VERSION
        .get_or_init(|| CString::new(ansatz_core::VERSION).expect("语义化版本号不含 NUL"))
        .as_ptr()
}
