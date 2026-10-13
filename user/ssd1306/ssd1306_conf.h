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

/* OLED 独占 I2C1（PB8=SCL / PB9=SDA，400 kHz）。
 * IMU 在 I2C2（PB10/PB11，100 kHz），两条总线互相独立；但 I2C1 的总线互斥锁
 * 仍然必须持有（见 user/OLEDpower.c），以后往 I2C1 上再加设备时不会与刷屏冲突。 */
#define SSD1306_I2C_PORT        hi2c1
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
//#define SSD1306_INCLUDE_FONT_7x10
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
