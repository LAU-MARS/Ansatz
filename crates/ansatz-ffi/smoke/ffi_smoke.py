"""ctypes 冒烟测试：验证 C ABI 的三个函数（本地开发用，不进 CI）。

用法：python crates/ansatz-ffi/smoke/ffi_smoke.py
前提：cargo build --release -p ansatz-ffi 已执行。
"""

import ctypes
import json
import os
import pathlib
import sys

ROOT = pathlib.Path(__file__).resolve().parents[3]


def load_lib() -> ctypes.CDLL:
    for name in ("ansatz_ffi.dll", "libansatz_ffi.so", "libansatz_ffi.dylib"):
        p = ROOT / "target" / "release" / name
        if p.exists():
            return ctypes.CDLL(str(p))
    sys.exit(f"找不到 ansatz-ffi 产物，请先 cargo build --release -p ansatz-ffi（搜索 {ROOT / 'target' / 'release'}）")


lib = load_lib()
lib.ansatz_solve_json.restype = ctypes.c_void_p
lib.ansatz_solve_json.argtypes = [ctypes.c_char_p]
lib.ansatz_string_free.argtypes = [ctypes.c_void_p]
lib.ansatz_version.restype = ctypes.c_void_p


def solve(model: dict) -> dict:
    raw = lib.ansatz_solve_json(json.dumps(model).encode())
    assert raw, "ansatz_solve_json 返回 NULL"
    try:
        return json.loads(ctypes.string_at(raw).decode())
    finally:
        lib.ansatz_string_free(raw)


version = ctypes.string_at(lib.ansatz_version()).decode()
print(f"ansatz_version() = {version}")

# 1) 平凡算例：3-4-5 → (6, 8)
env = solve({
    "entities": [{"id": 1, "geometry": {"type": "point2", "x": 3.0, "y": 4.0}}],
    "constraints": [{"id": 1, "kind": {"type": "distance", "a": 1, "b": None, "value": 10.0}}],
})
assert env["ok"], env
report = env["result"]
assert report["outcome"] == "underconstrained"
(x, y) = (report["entities"][0]["geometry"]["x"], report["entities"][0]["geometry"]["y"])
assert (x, y) == (6.0, 8.0), (x, y)
print(f"solve ok: (3,4) --d=10--> ({x}, {y}), outcome={report['outcome']}")

# 2) 冲突模型：ok=true（求解层结论），outcome=inconsistent，诊断完整
env = solve({
    "entities": [{"id": 1, "geometry": {"type": "point2", "x": 3.0, "y": 4.0}}],
    "constraints": [
        {"id": 1, "kind": {"type": "distance", "a": 1, "b": None, "value": 3.0}},
        {"id": 2, "kind": {"type": "distance", "a": 1, "b": None, "value": 5.0}},
    ],
})
assert env["ok"]
diag = env["result"]["diagnostics"]
assert env["result"]["outcome"] == "inconsistent"
assert diag["redundant_constraints"][0]["constraint_ids"] == [1, 2]
assert diag["max_residual"]["residual"] == 2.0
print("conflict model: ok=true, outcome=inconsistent, diagnostics complete")

# 3) 工具错误：非法 JSON → ok=false + kind=invalid_json
raw = lib.ansatz_solve_json(b"{ not json")
try:
    env = json.loads(ctypes.string_at(raw).decode())
finally:
    lib.ansatz_string_free(raw)
assert not env["ok"] and env["error"]["kind"] == "invalid_json", env
print("malformed json: ok=false, kind=invalid_json")

print("FFI smoke: ALL PASS")
