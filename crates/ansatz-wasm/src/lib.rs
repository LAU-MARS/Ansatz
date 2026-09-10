//! Ansatz wasm 壳：wasm-bindgen，产出带 .d.ts 的 npm 包（wasm-pack）。
//!
//! 只做类型转换与错误映射，不含数学逻辑（求解全部在 ansatz-core）。
//!
//! ## 线程契约
//!
//! **单线程可用**：不启用 atomics/postMessage 线程化，不依赖
//! SharedArrayBuffer / COOP-COEP，任意页面（含 file://）与 Node 均可加载。
//!
//! ## JS API（与 ansatz-node 同名同签名，宿主可无痛切换）
//!
//! - `solve(model)`：结构化调用。`model` 为符合 schema/model.schema.json 的
//!   JS 对象，返回信封 `{ ok, result | error }`，**永不 throw**。
//! - `solveJson(modelJson: string): string`：字符串调用，返回与 FFI 完全
//!   相同的 JSON 信封字符串。
//! - `version(): string`。
//!
//! 结构化调用的 TypeScript 类型见 `types/ansatz.d.ts`（与 JSON Schema
//! 一一对应；wasm-pack 不会自动拷入 pkg，接入时按需复制并在 tsconfig
//! 里引用）。

use wasm_bindgen::prelude::*;

use ansatz_core::{solve as solve_core, Envelope, Model};
use serde_wasm_bindgen::{from_value, to_value};

fn envelope_to_js(envelope: &Envelope) -> JsValue {
    to_value(envelope).unwrap_or(JsValue::NULL)
}

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
#[wasm_bindgen(js_name = solve)]
pub fn solve(model: &JsValue) -> JsValue {
    let model: Model = match from_value(model.clone()) {
        Ok(model) => model,
        Err(e) => {
            return envelope_to_js(&Envelope::error(
                "invalid_model",
                &format!("模型反序列化失败：{e}"),
            ))
        }
    };
    match solve_core(&model) {
        Ok(report) => envelope_to_js(&Envelope::ok(report)),
        Err(e) => envelope_to_js(&Envelope::from_solve_error(&e)),
    }
}

/// 字符串求解：与 FFI `ansatz_solve_json` 的 JSON 信封完全一致。
#[wasm_bindgen(js_name = solveJson)]
pub fn solve_json(model_json: &str) -> String {
    envelope_json(&solve_envelope(model_json))
}

/// 核心版本号。
#[wasm_bindgen(js_name = version)]
pub fn version() -> String {
    ansatz_core::VERSION.to_string()
}
