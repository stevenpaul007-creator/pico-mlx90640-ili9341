// 为 TFT_eSPI 库创建的项目专属配置文件 (tft_setup.h)
#define USER_SETUP_INFO "User_Setup"


//#define DISABLE_ALL_LIBRARY_WARNINGS
#define RP2040_DMA
#define RP2040_DMA_PRIORITY 0    // 最高优先级

#define RP2040_PIO_SPI 
#define RP2040_PIO_CLK_DIV 2 // ~60ns write cycle at 250MHz CPU clock

// ---- 1. 关键配置：启用 DMA（这是实现高效刷新的核心）---
#define USE_DMA_TRANSFERS               // 启用 DMA 传输[reference:1]
#define SPI_FREQUENCY  40000000L        // SPI 通讯频率设为 40MHz[reference:2]
#define SPI_READ_FREQUENCY  20000000L   // 读取数据频率设为 20MHz

// ---- 2. 选择屏幕驱动，只需启用 ILI9341 相关定义 ----
#define ILI9341_DRIVER

// ---- 3. 关键步骤：强制使用 SPI1 并重新映射引脚 ----
#define TFT_SPI_PORT 1                  // 关键! 强制 TFT_eSPI 使用 SPI1
#define USE_HSPI_PORT                   // 与上一条配合，指示使用辅助 SPI 端口（即 SPI1）

// ---- 4. 根据你的硬件连接，定义屏幕控制引脚 ----
#define TFT_MOSI 3                      // SPI1 TX 引脚
#define TFT_SCLK 2                      // SPI1 SCK 引脚
#define TFT_CS   4                      // 片选引脚
#define TFT_DC   7                      // 数据/命令引脚
#define TFT_RST  8                      // 复位引脚，如果没接可以不定义
#define TFT_MISO -1                     // 如果未连接 MISO，可以不定义

// ---- 5. 屏幕尺寸定义 ----
#define TFT_WIDTH  320
#define TFT_HEIGHT 240

// ---- 6. 启用硬件随机数生成器（编译时需要，与屏幕无关） ----
#define RP2040_RANDOM_PAD 1
#define LOAD_GLCD
#define LOAD_FONT2
#define LOAD_FONT4
#define LOAD_FONT6
#define LOAD_FONT7
#define LOAD_FONT8
