/**
 * Private configuration file for the SSD1306 library.
 *
 * 本工程（Robo / STM32F407ZGTx）修改记录，改动前先看这里：
 *   1) 芯片族改成 STM32F4。原文件留着的是 afiskon 库的示例默认值 STM32F0，
 *      它会让 ssd1306.h 去 #include "stm32f0xx_hal.h"（本工程没有这个头）。
 *   2) 增加 #include "i2c.h"：ssd1306.h 里只有
 *      `extern I2C_HandleTypeDef SSD1306_I2C_PORT;` 一句声明，而
 *      I2C_HandleTypeDef 的类型定义在 Core/Inc/i2c.h，不包含就编译不过。
 *   3) 字体只保留本工程用得到的 Font_11x18（四行数据）和 Font_6x8（备用），
 *      其余字体的字模表在 Flash 里白占约 25 KB，已关闭。
 */

#ifndef __SSD1306_CONF_H__
#define __SSD1306_CONF_H__

// Choose a microcontroller family
//#define STM32F0
//#define STM32F1
#define STM32F4
//#define STM32L0
//#define STM32L1
//#define STM32L4
//#define STM32F3
//#define STM32H7
//#define STM32F7
//#define STM32G0
//#define STM32C0
//#define STM32U5

// Choose a bus
#define SSD1306_USE_I2C
//#define SSD1306_USE_SPI

// I2C Configuration
/* 必须包含 i2c.h：I2C_HandleTypeDef 的类型定义在那里（同时 extern hi2c1/hi2c2）。 */
#include "i2c.h"

/* OLED 走 **I2C2**（PB10 = SCL / PB11 = SDA，400 kHz）。
 *
 * ★ 为什么从 I2C1 改到 I2C2（别再改回去）★
 *   实机接线是 PB10/PB11，而 CubeMX 里 I2C2 的初始化正是这两个引脚：
 *       i2c.c:125  PB10 ------> I2C2_SCL
 *       i2c.c:126  PB11 ------> I2C2_SDA
 *       i2c.c:71   hi2c2.Init.ClockSpeed = 400000   （SSD1306 支持 400 kHz）
 *       i2c.c:132  GPIO_InitStruct.Alternate = GPIO_AF4_I2C2
 *   原来这里写的是 hi2c1（PB8/PB9），那组引脚上没有接屏，所以 0x3C 一直不应答
 *   （实测现象：OLED 探测失败、"OLED not responding at 0x3C"）。
 *
 * ⚠ 关于引脚的两个硬事实：
 *   1) STM32F407 的 I2C2_SDA 只有 **PB11 / PF0 / PH5** 三个可选（AF4），
 *      **没有 PE11**。所以"SDA 接 PE11"是不成立的。
 *   2) PE11 在本工程里是 **TIM1_CH2 = M2_ENC_B = FR（右前）轮编码器 B 相**
 *      （main.h 的 M2_ENC_B_Pin / tim.c:542）。若把 SDA 挪到 PE11，
 *      FR 轮的编码器会直接失效。
 *
 * ★ 关于 I2C2 的独占性 ★
 *   i2c.c 里 MX_I2C2_Init() 已由 main.c:104 调用，硬件是初始化好的；
 *   而 user/sensorimu.c 目前是空壳（IMU 尚未实现），所以 I2C2 现在完全空闲。
 *   以后若真的往 I2C2 上挂 IMU，**必须复用同一把总线互斥锁**
 *   （user/OLEDpower.c 的 g_oled_i2c_mutex），否则两条任务同时访问会互相打断。
 *
 * ★ 地址 ★
 *   SSD1306 的 7 位地址是 0x3C；HAL 的 xxx_Addr 参数要求 8 位（已左移）格式。
 *   少数模块是 0x3D，若探测仍失败可试着改成 (0x3D << 1)。 */
#define SSD1306_I2C_PORT        hi2c2
/* SSD1306 的 7 位地址是 0x3C；HAL 的 xxx_Addr 参数要求 8 位（已左移）格式。 */
#define SSD1306_I2C_ADDR        (0x3C << 1)

// SPI Configuration
//#define SSD1306_SPI_PORT        hspi1
//#define SSD1306_CS_Port         OLED_CS_GPIO_Port
//#define SSD1306_CS_Pin          OLED_CS_Pin
//#define SSD1306_DC_Port         OLED_DC_GPIO_Port
//#define SSD1306_DC_Pin          OLED_DC_Pin
//#define SSD1306_Reset_Port      OLED_Res_GPIO_Port
//#define SSD1306_Reset_Pin       OLED_Res_Pin

// Mirror the screen if needed
// #define SSD1306_MIRROR_VERT
// #define SSD1306_MIRROR_HORIZ

// Set inverse color if needed
// # define SSD1306_INVERSE_COLOR

// Include only needed fonts
/* 只开这两套。Font_11x18 用于四行数据（11×18 等宽，宽 11 px → 每行 21 字符，
 * 4 行 y = 0/18/36/54 正好铺满 128×64 屏）；Font_6x8 留给以后的密集调试页。
 * 被注释掉的 7x10 / 16x26 / 16x24 / 16x15 四套字模在 Flash 里合计约 25 KB。 */
#define SSD1306_INCLUDE_FONT_6x8
/* ★ Font_7x10 必须启用（本工程的四行显示布局用它）★
 *   为什么不能用 Font_11x18 显示四行：
 *     ssd1306_WriteChar()（ssd1306.c:231）有一条硬检查：
 *         if (... || SSD1306_HEIGHT < (SSD1306.CurrentY + Font.height)) return 0;
 *     屏高 64，Font_11x18 高 18。前三行 y=0/18/36 都满足 (y+18 <= 64)，
 *     但第四行 y=54 时 54+18=72 > 64 → **整行 32 个像素一个都不画**。
 *     （注意：不是"只裁掉下半部分"，而是在 WriteChar 里直接 return，
 *      根本走不到 DrawPixel 的边界裁剪。）
 *   Font_7x10 高 10，行距取 16：y=0/16/32/48，最后一行 48+10=58 <= 64，
 *   四行全部完整可见。 */
#define SSD1306_INCLUDE_FONT_7x10
#define SSD1306_INCLUDE_FONT_11x18
//#define SSD1306_INCLUDE_FONT_16x26
//#define SSD1306_INCLUDE_FONT_16x24
//#define SSD1306_INCLUDE_FONT_16x15

// The width of the screen can be set using this
// define. The default value is 128.
// #define SSD1306_WIDTH           64

// If your screen horizontal axis does not start
// in column 0 you can use this define to
// adjust the horizontal offset
// #define SSD1306_X_OFFSET

// The height can be changed as well if necessary.
// It can be 32, 64 or 128. The default value is 64.
// #define SSD1306_HEIGHT          64

#endif /* __SSD1306_CONF_H__ */
