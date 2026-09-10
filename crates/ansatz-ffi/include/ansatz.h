#ifndef ANSATZ_H
#define ANSATZ_H

/* 警告：本文件由 crates/ansatz-ffi 的 build.rs（cbindgen）自动生成。
   不要手改；修改 FFI 接口后重新 `cargo build -p ansatz-ffi` 并提交。 */


#include <stdarg.h>
#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>
#include <stdlib.h>

/**
 * 求解模型。`input` 为符合 schema/model.schema.json 的 NUL 结尾 UTF-8 JSON。
 *
 * 返回值：JSON 信封字符串（归宿主所有，用 `ansatz_string_free` 释放），
 * 极端分配失败时返回 NULL。
 */
char *ansatz_solve_json(const char *input);

/**
 * 释放 `ansatz_solve_json` 返回的字符串。传 NULL 是安全的 no-op。
 */
void ansatz_string_free(char *s);

/**
 * 版本号（静态字符串，禁止 free）。
 */
const char *ansatz_version(void);

#endif  /* ANSATZ_H */
