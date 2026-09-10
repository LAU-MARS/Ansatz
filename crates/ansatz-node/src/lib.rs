//! Ansatz Node 壳：napi-rs，Node 原生性能路径。
//!
//! 只做类型转换与错误映射，不含数学逻辑（求解全部在 ansatz-core）。
//!
//! ## JS API（与 ansatz-wasm 同名同签名，宿主可无痛切换）
//!
//! - `solve(model)`：结构化调用，返回信封 `{ ok, result | error }`，永不 throw。
//! - `solveJson(modelJson: string): string`：与 FFI 一致的 JSON 信封字符串。
//! - `version(): string`。

use ansatz_core::{solve as solve_core, Envelope, Model};
use napi_derive::napi;

fn envelope_json(envelope: &Envelope) -> String {
    serde_json::to_string(envelope).unwrap_or_else(|_| {
        r#"{"ok":false,"error":{"kind":"serialization","message":"internal serialization failure"}}"#
            .to_string()
    })
}

fn solve_envelope(model_json: &str) -> Envelope {
    let model: Model = match serde_json::from_str(model_json) {
        Ok(model) => model,
        Err(e) => return Envelope::error("invalid_json", &format!("输入不是合法的模型 JSON：{e}")),
    };
    match solve_core(&model) {
        Ok(report) => Envelope::ok(report),
        Err(e) => Envelope::from_solve_error(&e),
    }
}

/// 结构化求解：`model` 为 JS 对象（符合 model.schema.json），返回信封，永不 throw。
#[napi(js_name = "solve")]
pub fn solve(model: serde_json::Value) -> serde_json::Value {
    let model: Model = match serde_json::from_value(model) {
        Ok(model) => model,
        Err(e) => {
            return serde_json::to_value(Envelope::error(
                "invalid_model",
                &format!("模型反序列化失败：{e}"),
            ))
            .unwrap_or(serde_json::Value::Null)
        }
    };
    match solve_core(&model) {
        Ok(report) => serde_json::to_value(Envelope::ok(report)).unwrap_or(serde_json::Value::Null),
        Err(e) => {
            serde_json::to_value(Envelope::from_solve_error(&e)).unwrap_or(serde_json::Value::Null)
        }
    }
}

/// 字符串求解：与 FFI `ansatz_solve_json` 的 JSON 信封完全一致。
#[napi(js_name = "solveJson")]
pub fn solve_json(model_json: String) -> String {
    envelope_json(&solve_envelope(&model_json))
}

/// 核心版本号。
#[napi(js_name = "version")]
pub fn version() -> String {
    ansatz_core::VERSION.to_string()
}
