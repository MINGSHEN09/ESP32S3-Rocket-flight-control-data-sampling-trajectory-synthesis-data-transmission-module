#include <Wire.h>

// I2C 引脚
#define I2C_SDA 41
#define I2C_SCL 42

// ADXL375 地址及寄存器
#define ADXL375_ADDR        0x53   // ← 改为 0x53
#define ADXL375_DATA_FORMAT 0x31
#define ADXL375_POWER_CTL   0x2D
#define ADXL375_DATAZ0      0x36

// 冷湖当地重力加速度 (m/s²)
const float kGravity = 9.7865;

const int SAMPLES = 200;
bool upDone = false, downDone = false;
float Z_up = 0, Z_down = 0;

int16_t readRawZ() {
  Wire.beginTransmission(ADXL375_ADDR);
  Wire.write(ADXL375_DATAZ0);
  Wire.endTransmission(false);
  Wire.requestFrom(ADXL375_ADDR, (uint8_t)2);
  if (Wire.available() >= 2) {
    return (Wire.read() << 8) | Wire.read();
  }
  return 0;
}

float collectAverage() {
  long sum = 0;
  for (int i = 0; i < SAMPLES; i++) {
    sum += readRawZ();
    delay(5);
  }
  return sum / (float)SAMPLES;
}

void setup() {
  Serial.begin(115200);
  delay(500);
  Wire.begin(I2C_SDA, I2C_SCL);
  Wire.setClock(400000);

  Wire.beginTransmission(ADXL375_ADDR);
  Wire.write(ADXL375_DATA_FORMAT);
  Wire.write(0x0B);
  Wire.endTransmission();

  Wire.beginTransmission(ADXL375_ADDR);
  Wire.write(ADXL375_POWER_CTL);
  Wire.write(0x08);
  Wire.endTransmission();

  Serial.println("=== ADXL375 Calibration (ADDR 0x53) ===");
  Serial.println("1. Orient sensor Z-axis UP (pointing to sky), then send 'UP'");
  Serial.println("2. Orient sensor Z-axis DOWN (pointing to ground), then send 'DOWN'");
  Serial.println("3. After both steps, calibration will auto-compute.");
  Serial.println("Send 'RESET' to restart.");
}


void loop() {
  int16_t raw = readRawZ();
  Serial.println(raw);
  delay(100);
}