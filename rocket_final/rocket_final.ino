/*
 * 纯海拔高度记录（增加 time_sec 列）
 * 传感器：ICM20602（陀螺仪+加速度计） + ADXL375（高G值Z轴加速度计）
 * 原理：四元数姿态解算 → 将ADXL375的Z轴加速度转换到世界系 → 积分得到垂直高度和速度
 * CSV列：time_ms, time_sec, accVertical, velZ, altZ, launched
 */

#include <Wire.h>
#include <SPI.h>
#include <SD.h>

// ================= 冷湖发射场参数 =================
const float kGravity = 9.7865;

// ================= 引脚定义 =================
#define I2C_SDA 41
#define I2C_SCL 42
#define SD_CS   17
#define LED_PIN 2

// ================= ICM20602 寄存器 =================
#define ICM_ADDR         0x68
#define PWR_MGMT_1       0x6B
#define ACCEL_CONFIG     0x1C
#define GYRO_CONFIG      0x1B
#define ACCEL_XOUT_H     0x3B
#define GYRO_XOUT_H      0x43

#define ACCEL_SENSITIVITY 2048.0   // ±16G -> 2048 LSB/g
#define GYRO_SENSITIVITY  16.4     // ±2000dps -> 16.4 LSB/(deg/s)

// ================= ADXL375 寄存器 =================
#define ADXL375_ADDR        0x53
#define ADXL375_DATA_FORMAT 0x31
#define ADXL375_POWER_CTL   0x2D
#define ADXL375_DATAZ0      0x36
#define ADXL375_SENSITIVITY 0.48    // ±200g -> ~0.48 m/s² per LSB (实际需校准)

// ================= 简易 Madgwick 姿态滤波器 =================
class SimpleMadgwick {
public:
    SimpleMadgwick(float beta = 0.1f) : _beta(beta), _q0(1.0f), _q1(0.0f), _q2(0.0f), _q3(0.0f) {}

    void update(float gx, float gy, float gz, float ax, float ay, float az, float dt) {
        float recipNorm, s0, s1, s2, s3;
        float qDot1, qDot2, qDot3, qDot4;
        float _2q0, _2q1, _2q2, _2q3, _4q0, _4q1, _4q2, _8q1, _8q2, q0q0, q1q1, q2q2, q3q3;

        // 角速度积分
        qDot1 = 0.5f * (-_q1 * gx - _q2 * gy - _q3 * gz);
        qDot2 = 0.5f * ( _q0 * gx + _q2 * gz - _q3 * gy);
        qDot3 = 0.5f * ( _q0 * gy - _q1 * gz + _q3 * gx);
        qDot4 = 0.5f * ( _q0 * gz + _q1 * gy - _q2 * gx);

        // 加速度归一化
        recipNorm = invSqrt(ax*ax + ay*ay + az*az);
        ax *= recipNorm; ay *= recipNorm; az *= recipNorm;

        // 梯度下降修正
        _2q0 = 2.0f*_q0; _2q1 = 2.0f*_q1; _2q2 = 2.0f*_q2; _2q3 = 2.0f*_q3;
        _4q0 = 4.0f*_q0; _4q1 = 4.0f*_q1; _4q2 = 4.0f*_q2;
        _8q1 = 8.0f*_q1; _8q2 = 8.0f*_q2;
        q0q0 = _q0*_q0; q1q1 = _q1*_q1; q2q2 = _q2*_q2; q3q3 = _q3*_q3;

        s0 = _4q0*q2q2 + _2q2*ax + _4q0*q1q1 - _2q1*ay;
        s1 = _4q1*q3q3 - _2q3*ax + 4.0f*q0q0*_q1 - _2q0*ay - _4q1 + _8q1*q1q1 + _8q1*q2q2 + _4q1*az;
        s2 = 4.0f*q0q0*_q2 + _2q0*ax + _4q2*q3q3 - _2q3*ay - _4q2 + _8q2*q1q1 + _8q2*q2q2 + _4q2*az;
        s3 = 4.0f*q1q1*_q3 - _2q1*ax + 4.0f*q2q2*_q3 - _2q2*ay;
        recipNorm = invSqrt(s0*s0 + s1*s1 + s2*s2 + s3*s3);
        s0 *= recipNorm; s1 *= recipNorm; s2 *= recipNorm; s3 *= recipNorm;

        qDot1 -= _beta*s0; qDot2 -= _beta*s1; qDot3 -= _beta*s2; qDot4 -= _beta*s3;

        // 四元数更新
        _q0 += qDot1*dt; _q1 += qDot2*dt; _q2 += qDot3*dt; _q3 += qDot4*dt;

        // 归一化
        recipNorm = invSqrt(_q0*_q0 + _q1*_q1 + _q2*_q2 + _q3*_q3);
        _q0 *= recipNorm; _q1 *= recipNorm; _q2 *= recipNorm; _q3 *= recipNorm;
    }

    void getQuaternion(float &w, float &x, float &y, float &z) {
        w = _q0; x = _q1; y = _q2; z = _q3;
    }

private:
    float _beta, _q0, _q1, _q2, _q3;
    float invSqrt(float x) {
        float halfx = 0.5f * x;
        float y = x;
        long i = *(long*)&y;
        i = 0x5f3759df - (i >> 1);
        y = *(float*)&i;
        y = y * (1.5f - (halfx * y * y));
        return y;
    }
};

SimpleMadgwick filter(0.1f);

// ================= 全局变量 =================
float velZ = 0, altZ = 0;
unsigned long lastMicros = 0;
bool launched = false;
bool sdCardOK = false;
File dataFile;
char logFileName[20] = "/alt00.csv";  // 默认文件名

// ================= 传感器读取函数 =================
void readICM(float &ax, float &ay, float &az, float &gx, float &gy, float &gz) {
  Wire.beginTransmission(ICM_ADDR);
  Wire.write(ACCEL_XOUT_H);
  Wire.endTransmission(false);
  Wire.requestFrom(ICM_ADDR, (uint8_t)14);
  int16_t rax = (Wire.read()<<8) | Wire.read();
  int16_t ray = (Wire.read()<<8) | Wire.read();
  int16_t raz = (Wire.read()<<8) | Wire.read();
  Wire.read(); Wire.read(); // 跳过温度
  int16_t rgx = (Wire.read()<<8) | Wire.read();
  int16_t rgy = (Wire.read()<<8) | Wire.read();
  int16_t rgz = (Wire.read()<<8) | Wire.read();
  ax = rax / ACCEL_SENSITIVITY * kGravity;
  ay = ray / ACCEL_SENSITIVITY * kGravity;
  az = raz / ACCEL_SENSITIVITY * kGravity;
  gx = rgx / GYRO_SENSITIVITY * DEG_TO_RAD;
  gy = rgy / GYRO_SENSITIVITY * DEG_TO_RAD;
  gz = rgz / GYRO_SENSITIVITY * DEG_TO_RAD;
}

float readADXL375Z() {
  Wire.beginTransmission(ADXL375_ADDR);
  Wire.write(ADXL375_DATAZ0);
  Wire.endTransmission(false);
  Wire.requestFrom(ADXL375_ADDR, (uint8_t)2);
  if (Wire.available() >= 2) {
    int16_t raw = (Wire.read()<<8) | Wire.read();
    return raw * ADXL375_SENSITIVITY;  // 实际使用时应减去校准偏置
  }
  return 0;
}

// ================= 初始化 =================
void setup() {
  Serial.begin(115200);
  delay(500);
  pinMode(LED_PIN, OUTPUT);
  digitalWrite(LED_PIN, HIGH);

  Wire.begin(I2C_SDA, I2C_SCL);
  Wire.setClock(400000);

  // ICM20602 初始化
  Wire.beginTransmission(ICM_ADDR);
  Wire.write(PWR_MGMT_1); Wire.write(0x00);
  Wire.endTransmission();
  delay(100);
  Wire.beginTransmission(ICM_ADDR);
  Wire.write(ACCEL_CONFIG); Wire.write(0x18); // ±16G
  Wire.endTransmission();
  Wire.beginTransmission(ICM_ADDR);
  Wire.write(GYRO_CONFIG); Wire.write(0x18);  // ±2000dps
  Wire.endTransmission();

  // ADXL375 初始化
  Wire.beginTransmission(ADXL375_ADDR);
  Wire.write(ADXL375_DATA_FORMAT); Wire.write(0x0B); // ±200g
  Wire.endTransmission();
  Wire.beginTransmission(ADXL375_ADDR);
  Wire.write(ADXL375_POWER_CTL); Wire.write(0x08);
  Wire.endTransmission();

  // SD 卡初始化
  SPI.begin(14, 15, 16, SD_CS);
  if (!SD.begin(SD_CS)) {
    Serial.println("SD Card Failed!");
    sdCardOK = false;
  } else {
    sdCardOK = true;
    int fileNum = 0;
    while (true) {
      sprintf(logFileName, "/alt%02d.csv", fileNum);
      if (!SD.exists(logFileName)) break;
      fileNum++;
    }
    dataFile = SD.open(logFileName, FILE_WRITE);
    if (dataFile) {
      // 修改表头：增加 time_sec 列
      dataFile.println("time_ms,time_sec,accVertical,velZ,altZ,launched");
      dataFile.close();
      Serial.printf("Logging to %s\n", logFileName);
    }
  }

  velZ = 0; altZ = 0;
  lastMicros = micros();
  Serial.println("Altitude-only ready (with time_sec). Waiting for launch...");
}

// ================= 主循环 =================
void loop() {
  unsigned long now = micros();
  float dt = (now - lastMicros) / 1000000.0f;
  lastMicros = now;
  if (dt <= 0) dt = 0.001f;

  // 读取 ICM20602（用于姿态）
  float ax, ay, az, gx, gy, gz;
  readICM(ax, ay, az, gx, gy, gz);

  // 更新四元数
  filter.update(gx, gy, gz, ax, ay, az, dt);
  float qw, qx, qy, qz;
  filter.getQuaternion(qw, qx, qy, qz);

  // 读取 ADXL375 Z轴
  float rawAccZ = readADXL375Z();

  // 发射检测
  if (!launched && rawAccZ > 15.0f) {
    launched = true;
    velZ = 0;
    altZ = 0;
  }

  // 旋转矩阵第三行：世界系Z轴与体坐标系的关系
  float rotZx = 2.0f*(qx*qz - qw*qy);
  float rotZy = 2.0f*(qy*qz + qw*qx);
  float rotZz = qw*qw - qx*qx - qy*qy + qz*qz;

  // 假设 ADXL375 安装在火箭纵轴（体坐标系Z轴），体加速度为 (0,0,rawAccZ)
  // 转换到世界系垂直加速度 = rotZz * rawAccZ - g
  float accVertical = rotZz * rawAccZ - kGravity;

  // 积分
  velZ += accVertical * dt;
  altZ += velZ * dt;

  // 写入 SD 卡（增加 time_sec 列）
  if (sdCardOK) {
    dataFile = SD.open(logFileName, FILE_APPEND);
    if (dataFile) {
      // 修改：在 millis() 后添加 millis()/1000.0f，保留三位小数
      dataFile.printf("%lu,%.3f,%.2f,%.2f,%.2f,%d\n",
                      millis(),
                      millis() / 1000.0f,   // 新增 time_sec
                      accVertical, velZ, altZ, launched);
      dataFile.close();
    }
  }

  Serial.printf("AccV:%.1f Vel:%.1f Alt:%.1f\n", accVertical, velZ, altZ);
  // 不加延时，约1kHz循环
}    