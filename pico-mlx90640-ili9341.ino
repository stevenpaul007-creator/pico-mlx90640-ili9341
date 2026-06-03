/**
   MLX90640 + ILI9341 (TFT_eSPI + DMA)
   功能:
     - 读取 32x24 热像数据
     - 双线性插值放大至 256x96 (8倍平滑)
     - 动态温度范围 (自动适应，低通滤波)
     - 右侧 64x240 垂直彩虹色条
     - 底部显示当前帧最高/最低温度及动态范围
     - TFT_eSPI 使用 SPI1 硬件 + DMA 加速刷新
     - core0： 操作tft屏幕、串口、IO按键
     - core1： 读取传感器数据、写入屏幕buffer
*/

#include <Adafruit_MLX90640.h>
#include <Wire.h>               // I2C for MLX90640
#include <TFT_eSPI.h>    // TFT_eSPI 库
#include <pico/multicore.h>
#include "batteryservice.h"

// ========== Sensor Object ==========
#define PIN_I2C_SDA 10
#define PIN_I2C_SCL 11
#define PIXEL_COUNT 768
Adafruit_MLX90640 MLX90640;

// 缓冲区
float frame[PIXEL_COUNT];
uint16_t readyBuffer[PIXEL_COUNT];

#define DEF_MINTEMP 200  // 实际是20度
#define DEF_MAXTEMP 600  // 实际是60度

uint16_t mintemp = DEF_MINTEMP;
uint16_t maxtemp = DEF_MAXTEMP;
uint16_t minmaxtempdif = maxtemp - mintemp;


//the colors we will be using
uint16_t camColors[256] ;
/*=
  { 0x480F, 0x400F, 0x400F, 0x400F, 0x4010, 0x3810, 0x3810, 0x3810, 0x3810, 0x3010, 0x3010,
  0x3010, 0x2810, 0x2810, 0x2810, 0x2810, 0x2010, 0x2010, 0x2010, 0x1810, 0x1810,
  0x1811, 0x1811, 0x1011, 0x1011, 0x1011, 0x0811, 0x0811, 0x0811, 0x0011, 0x0011,
  0x0011, 0x0011, 0x0011, 0x0031, 0x0031, 0x0051, 0x0072, 0x0072, 0x0092, 0x00B2,
  0x00B2, 0x00D2, 0x00F2, 0x00F2, 0x0112, 0x0132, 0x0152, 0x0152, 0x0172, 0x0192,
  0x0192, 0x01B2, 0x01D2, 0x01F3, 0x01F3, 0x0213, 0x0233, 0x0253, 0x0253, 0x0273,
  0x0293, 0x02B3, 0x02D3, 0x02D3, 0x02F3, 0x0313, 0x0333, 0x0333, 0x0353, 0x0373,
  0x0394, 0x03B4, 0x03D4, 0x03D4, 0x03F4, 0x0414, 0x0434, 0x0454, 0x0474, 0x0474,
  0x0494, 0x04B4, 0x04D4, 0x04F4, 0x0514, 0x0534, 0x0534, 0x0554, 0x0554, 0x0574,
  0x0574, 0x0573, 0x0573, 0x0573, 0x0572, 0x0572, 0x0572, 0x0571, 0x0591, 0x0591,
  0x0590, 0x0590, 0x058F, 0x058F, 0x058F, 0x058E, 0x05AE, 0x05AE, 0x05AD, 0x05AD,
  0x05AD, 0x05AC, 0x05AC, 0x05AB, 0x05CB, 0x05CB, 0x05CA, 0x05CA, 0x05CA, 0x05C9,
  0x05C9, 0x05C8, 0x05E8, 0x05E8, 0x05E7, 0x05E7, 0x05E6, 0x05E6, 0x05E6, 0x05E5,
  0x05E5, 0x0604, 0x0604, 0x0604, 0x0603, 0x0603, 0x0602, 0x0602, 0x0601, 0x0621,
  0x0621, 0x0620, 0x0620, 0x0620, 0x0620, 0x0E20, 0x0E20, 0x0E40, 0x1640, 0x1640,
  0x1E40, 0x1E40, 0x2640, 0x2640, 0x2E40, 0x2E60, 0x3660, 0x3660, 0x3E60, 0x3E60,
  0x3E60, 0x4660, 0x4660, 0x4E60, 0x4E80, 0x5680, 0x5680, 0x5E80, 0x5E80, 0x6680,
  0x6680, 0x6E80, 0x6EA0, 0x76A0, 0x76A0, 0x7EA0, 0x7EA0, 0x86A0, 0x86A0, 0x8EA0,
  0x8EC0, 0x96C0, 0x96C0, 0x9EC0, 0x9EC0, 0xA6C0, 0xAEC0, 0xAEC0, 0xB6E0, 0xB6E0,
  0xBEE0, 0xBEE0, 0xC6E0, 0xC6E0, 0xCEE0, 0xCEE0, 0xD6E0, 0xD700, 0xDF00, 0xDEE0,
  0xDEC0, 0xDEA0, 0xDE80, 0xDE80, 0xE660, 0xE640, 0xE620, 0xE600, 0xE5E0, 0xE5C0,
  0xE5A0, 0xE580, 0xE560, 0xE540, 0xE520, 0xE500, 0xE4E0, 0xE4C0, 0xE4A0, 0xE480,
  0xE460, 0xEC40, 0xEC20, 0xEC00, 0xEBE0, 0xEBC0, 0xEBA0, 0xEB80, 0xEB60, 0xEB40,
  0xEB20, 0xEB00, 0xEAE0, 0xEAC0, 0xEAA0, 0xEA80, 0xEA60, 0xEA40, 0xF220, 0xF200,
  0xF1E0, 0xF1C0, 0xF1A0, 0xF180, 0xF160, 0xF140, 0xF100, 0xF0E0, 0xF0C0, 0xF0A0,
  0xF080, 0xF060, 0xF040, 0xF020, 0xF800,
  };
*/
TFT_eSPI tft = TFT_eSPI();
TFT_eSprite spr = TFT_eSprite(&tft);

boolean core0inited = false;
uint32_t batterytimestamp = millis();

boolean buttonRight = false;
boolean buttonLeft = false;


// 中断服务函数
void enterBootloader() {
  delay(50);
  rp2040.rebootToBootloader();     // 重启并进入 USB 烧录模式
}

void buttonRightPressed() {
  buttonRight = true;
}
void buttonLeftPressed() {
  buttonLeft = !buttonLeft;
}

void changeMaxTemp() {
  if (!buttonRight) return;

  buttonRight = false;
  maxtemp += 100;
  if (maxtemp > 1000) {
    maxtemp = DEF_MAXTEMP;
  }
}

uint16_t mapTemp(uint16_t temp) {
  int32_t ret = ((temp - mintemp) * 1023) / minmaxtempdif;
  if (ret < 0) ret = 0; else if (ret > 1023) ret = 1023;
  return ret;
}

void checkSerialToBootloader() {
  if (Serial.available() > 0) {
    char incomingChar = Serial.read(); // 读取输入的字符

    // 如果收到字符 'b'
    if (incomingChar == 'b') {
      Serial.println("Rebooting into Bootloader mode...");
      // 核心函数：直接重启并进入 BOOTSEL 模式
      rp2040.rebootToBootloader();
    }
    // r, 重启
    else if (incomingChar == 'r') {
      rp2040.reboot();
    }
    // d, 输出文件表格
    else if (incomingChar == 'd') {
      Serial.print("$$ ");
      for (size_t i = 0; i < PIXEL_COUNT; i++) {
        Serial.printf("%.1f ", frame[i]);
      }
      Serial.println();
    }
    // h, 输出帮助信息
    else if (incomingChar == 'h') {
      Serial.println("h for help; d for data; b for bootloader; r for reboot");
    }
  }
}

// 将 HSV 转换为 RGB565
uint16_t HSVtoRGB565(float h, float s, float v) {
  // h: 0-360, s: 0-1, v: 0-1
  float c = v * s;
  float x = c * (1 - fabs(fmod(h / 60.0, 2) - 1));
  float m = v - c;
  float r, g, b;
  if (h < 60) {
    r = c;
    g = x;
    b = 0;
  }
  else if (h < 120) {
    r = x;
    g = c;
    b = 0;
  }
  else if (h < 180) {
    r = 0;
    g = c;
    b = x;
  }
  else if (h < 240) {
    r = 0;
    g = x;
    b = c;
  }
  else if (h < 300) {
    r = x;
    g = 0;
    b = c;
  }
  else {
    r = c;
    g = 0;
    b = x;
  }
  r = (r + m) * 255;
  g = (g + m) * 255;
  b = (b + m) * 255;
  // RGB565 转换
  return ((uint16_t)(r / 8) << 11) | ((uint16_t)(g / 4) << 5) | (uint16_t)(b / 8);
}

void generateColorMap() {
  for (int i = 0; i < 256; i++) {
    // 色相从 240°（蓝）线性递减到 0°（红）
    float hue = 240.0f - (240.0f * i / 255.0f);
    camColors[i] = HSVtoRGB565(hue, 1.0, 1.0);
  }
}

// CORE 1: 负责读取和准备数据、IO按键、串口
void core1_entry() {
  while (1) {
    // freezed
    if (buttonLeft) {
      continue;
    }

    changeMaxTemp();
    minmaxtempdif = maxtemp - mintemp;

    // 尝试读取一帧MLX90640
    int ret = MLX90640.getFrame(frame);
    if (ret != 0) {
      delay(10);
      continue;
    }

    // 由于rp2040的浮点数运算比较差，我们将温度按uint16_t存储，并放大10倍
    for (size_t i = 0; i < PIXEL_COUNT; i++) {
      readyBuffer[i] = (int16_t)(frame[i] * 10.0f);
    }

    // 1. 计算当前帧的绝对 min/max
    int16_t frameMin = readyBuffer[0], frameMax = readyBuffer[0];
    for (size_t i = 1; i < PIXEL_COUNT; i++) {
      if (readyBuffer[i] < frameMin) frameMin = readyBuffer[i];
      if (readyBuffer[i] > frameMax) frameMax = readyBuffer[i];
    }

    // 假设屏幕宽度 TFT_HEIGHT 为 320，高度 TFT_WIDTH 为 240
    // 缩放因子计算
    int32_t mx = (32 << 7) / TFT_HEIGHT;
    int32_t my = (24 << 7) / TFT_WIDTH;

    for (int y = 0; y < TFT_WIDTH; y++) {
      int32_t y0 = (y * my) >> 7;
      int32_t ty = (y * my) & 0x7f;
      int32_t y1 = (y0 < 23) ? y0 + 1 : y0;

      for (int x = 0; x < TFT_HEIGHT; x++) {
        int32_t x0 = (x * mx) >> 7;
        int32_t tx = (x * mx) & 0x7f;

        // 获取周围 4 个点的原始温度（先转为映射后的数值）
        int32_t mirror_x0 = 31 - x0;
        int32_t mirror_x1 = (mirror_x0 > 0) ? mirror_x0 - 1 : mirror_x0;
        // 注意：如果是水平镜像，需要类似原代码处理：31 - x0
        int16_t t00 = readyBuffer[y0 * 32 + mirror_x0];
        int16_t t10 = readyBuffer[y0 * 32 + mirror_x1]; // Next horizontal pixel
        int16_t t01 = readyBuffer[y1 * 32 + mirror_x0];
        int16_t t11 = readyBuffer[y1 * 32 + mirror_x1]; // Next diagonal pixel

        int16_t v00 = mapTemp(t00);
        int16_t v10 = mapTemp(t10);
        int16_t v01 = mapTemp(t01);
        int16_t v11 = mapTemp(t11);

        // 双线性插值计算
        int32_t s = v00 + ((tx * (v10 - v00)) >> 7);
        int32_t e = v01 + ((tx * (v11 - v01)) >> 7);
        int32_t v = s + ((ty * (e - s)) >> 7);

        // 将插值后的温度（0-1023）映射为颜色索引（0-255）
        int16_t colorIndex = (v * 255) / 1023;
        colorIndex = constrain(colorIndex, 0, 255);

        // 绘制像素点 (由于是逐像素绘制，屏幕刷新率需视硬件 SPI 性能而定)
        spr.drawPixel(x, y, camColors[colorIndex]);
      }
    }

    spr.setCursor(0, 0);
    spr.setTextSize(2);
    spr.printf("MAX: %.1f C", ((float)(frameMax)) / 10.0f);
    spr.println();
    spr.printf("MIN: %.1f C", ((float)(frameMin)) / 10.0f);
    spr.println();
    spr.printf("CEN: %.1f C", ((float)(readyBuffer[PIXEL_COUNT / 2 + 16])) / 10.0f);

    char batBuffer[16];
    snprintf(batBuffer, sizeof(batBuffer), "BAT: %.1fV", BatteryService::getVSYSVoltage());

    spr.setTextDatum(BR_DATUM);
    spr.drawString(batBuffer, 315, 235);

    rp2040.fifo.push_nb((uint32_t)readyBuffer);
    // 适当延时，防止过快刷新，具体根据你的采样率调整
    delay(10);
  }
}
void setup()
{
  Serial.begin(115200);

  // SEL键，重启系统，进入bootloader模式
  pinMode(16, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(16), enterBootloader, FALLING);

  // 右键，改变颜色板的最高温度值
  pinMode(17, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(17), buttonRightPressed, FALLING);

  // 左键，锁定屏幕
  pinMode(18, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(18), buttonLeftPressed, FALLING);

  tft.init();
  tft.setRotation(3);
  spr.setColorDepth(16);
  if (!spr.createSprite(TFT_HEIGHT, TFT_WIDTH)) {
    Serial.println("Core1: Sprite creation failed!");
    while (1) delay(10);
  }
  spr.fillSprite(TFT_BLACK);
  spr.setTextColor(TFT_WHITE);
  spr.setTextSize(2);

  spr.println();
  spr.println();
  spr.println();
  spr.println("==========================");
  spr.println("= MLX90640 32x24 Display =");
  spr.println("==========================");
  spr.println();
  spr.println();
  spr.pushSprite(0, 0);


  Serial.println("======================================");
  Serial.println("  MLX90640 32x24 IR Display (ILI9341)");
  Serial.println("======================================");

  // ----- Configure I2C for MLX90640 (use I2C1 pins) -----
  // 1. 将引脚绑定到 Wire1 (对应硬件 I2C1)
  Wire1.setSDA(PIN_I2C_SDA);
  Wire1.setSCL(PIN_I2C_SCL);

  // 2. 启动 Wire1
  Wire1.begin();
  Wire1.setClock(1000000);

  if (! MLX90640.begin(MLX90640_I2CADDR_DEFAULT, &Wire1))  {
    Serial.println("MLX90640 not found!");
    while (1)    {
      delay(10);
    }
  }

  Serial.println("Found MLX90640");
  spr.println("      Found MLX90640      ");
  spr.pushSprite(0, 0);

  // ----- MLX90640 settings -----
  MLX90640.setMode(MLX90640_CHESS);
  MLX90640.setResolution(MLX90640_ADC_16BIT);
  MLX90640.setRefreshRate(MLX90640_8_HZ);

  BatteryService::initBattery();
  delay(100);
  core0inited = true;
}

// Main Loop  core0 收到信号后刷新屏幕
void loop() {
  uint32_t timestamp = millis();

  // 检查 FIFO 队列中是否有 core1 发来的消息
  if (rp2040.fifo.available()) {
    rp2040.fifo.pop();
    spr.pushSprite(0, 0);
  } else {
    // 检查串口
    checkSerialToBootloader();
    if (timestamp - batterytimestamp >= 30000) {
      BatteryService::measureBattery();
      batterytimestamp = timestamp;
    }
  }

  delay(10);
}

void setup1() {
  generateColorMap();
  while (!core0inited) {
    delay(10);
  }
  MLX90640.getFrame(frame);
}

// This runs automatically on Core 1 in a continuous loop
void loop1() {
  core1_entry();
}
