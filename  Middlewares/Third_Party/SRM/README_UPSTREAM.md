# 上游来源与改动说明

## 来源
- 项目：SRMCCRemoter
- 仓库：https://github.com/Dumenghuanxiang/SRMCCRemoter
- 协议：SRM 校内赛 v4（Wire Protocol Version 4）
- 使用日期：<填你下载的日期>
- 许可证：GNU GPL-3.0（见同目录 LICENSE）

## 本工程使用的文件
仅使用上游 `android/firmware/` 下的协议层与 STM32 适配层：

| 本工程文件 | 上游路径 | 改动 |
|---|---|---|
| `srm_protocol.c/.h` | `android/firmware/srm_protocol.c/.h` | 无改动 |
| `srm_uart.c/.h` | `android/firmware/stm32f103_hal/srm_uart.c/.h` | 见下 |
| `srm_remote.c/.h` | `android/firmware/stm32f103_hal/srm_remote.c/.h` | 见下 |

未使用上游的 Android App、PC 中继（Rust/Python）、测试与分析工具。

## 相对上游的改动清单
1. `srm_uart.h`：`#include "stm32f1xx_hal.h"` 改为 `#include "stm32f4xx_hal.h"`
   （目标芯片由 STM32F103 改为 STM32F407VET6，仅头文件引用变化，逻辑未改）。
2. `srm_remote.c/.h`：无逻辑改动。
    - 注：`SRM_BoardApplyControl()` / `SRM_BoardApplyProControl()` 的实现位于本工程
      `App/task_remote.c`，未修改上游的弱符号定义。
3. 调用方式：上游为裸机主循环调用 `SRM_Remote_Poll()`，本工程改为在 FreeRTOS
   任务 `TskRemote`（1 kHz）中调用；上游协议层代码未修改。

## 许可证义务
本工程对外发布（含提交 git 仓库）时遵守 GPL-3.0：
保留原始 LICENSE 文件、保留版权声明、公开本目录下文件的完整源码及上述改动。

## 联系方式
Team Seasons(18621870680)