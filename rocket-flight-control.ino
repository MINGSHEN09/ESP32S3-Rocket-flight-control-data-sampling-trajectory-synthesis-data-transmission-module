#include <Wire.h>
#include <SPI.h>
#include <SD.h>
#include <TinyGPS++.h>
#include <Adafruit_BMP3XX.h>
#include <ICM20602.h> // 确保你安装了对应的库
#include <esp_task_wdt.h>

// --- 4G 网络库 ---
#define TINY_GSM_MODEM_SIM7600
#include <TinyGsmClient.h>
#include <ArduinoHttpClient.h>

// ===================== 用户配置 =====================

// 服务器配置 (请修改为你的实际服务器)
const char server[] = "YOUR_SERVER_IP"; 
const int  port = 5000;
const char resource[] = "/rocket";

// 硬件引脚定义 (基于 ESP32-S3)
#define SD_CS 5
#define SD_MOSI 11
#define SD_MISO 13
#define SD_SCK 12

#define GPS_RX 18
#define GPS_TX 19

#define SIM_RX 17
#define SIM_TX 16
#define SIM_PWR 4   // 某些板子可能是 RST 或 PWR_KEY
#define SIM_BAUD 115200

#define SDA_PIN 21
#define SCL_PIN 22

// 传感器参数
#define SEALEVELPRESSURE_HPA 1013.25
#define LOG_INTERVAL 20     // 50Hz 记录频率 (SD卡)
#define UPLOAD_INTERVAL 1000 // 1Hz 上传频率 (4G)

// APN 配置 (中国移动: cmnet, 联通: 3gnet, 电信: ctnet)
const char apn[]  = "cmnet";
const char user[] = "";
const char pass[] = "";

// ===================== 对象与全局变量 =====================

TinyGPSPlus gps;
Adafruit_BMP3XX bmp;
ICM20602 imu;

// 4G 对象
HardwareSerial SerialAT(2);
TinyGsm modem(SerialAT);
TinyGsmClient client(modem);
HttpClient http(client, server, port);

File dataFile;
SemaphoreHandle_t xMutex; // 用于保护共享数据的互斥锁

// ===================== 数据结构 =====================

enum FlightPhase {
  IDLE,
  ASCENT,
  APOGEE,
  DESCENT,
  LANDED
};

struct SharedData {
  float ax, ay, az;
  float gx, gy, gz;
  float press_alt;
  float gps_alt;
  double lat, lon;
  float vspeed;
  int phase;
  uint32_t timestamp;
  bool gps_valid;
};

// 全局共享数据 (需要互斥锁保护)
SharedData currentData;

// ===================== 任务句柄 =====================
TaskHandle_t TaskTelemetry; 

// ===================== 工具函数 =====================

void sim7600PowerOn() {
  pinMode(SIM_PWR, OUTPUT);
  // 模拟按下开机键
  digitalWrite(SIM_PWR, HIGH);
  delay(300); 
  digitalWrite(SIM_PWR, LOW);
  delay(10000); // 等待模块启动
}

String createFilename() {
  if(!SD.exists("/flight_000.csv")) return "/flight_000.csv";
  for(int i=1; i<999; i++){
    char name[20];
    sprintf(name,"/flight_%03d.csv", i);
    if(!SD.exists(name)) return String(name);
  }
  return "/flight_overflow.csv";
}

// 遥测任务 (4G上传)


void TelemetryLoop(void * parameter) {
  uint32_t lastUploadTime = 0;

  while(true) {
    if (millis() - lastUploadTime > UPLOAD_INTERVAL) {
      
      // 复制一份数据，避免长时间占用锁
      SharedData dataToSend;
      if (xSemaphoreTake(xMutex, (TickType_t)10) == pdTRUE) {
        dataToSend = currentData;
        xSemaphoreGive(xMutex);
      } else {
        vTaskDelay(10);
        continue; // 获取锁失败，稍后重试
      }

      // 2. 仅在有 GPS 或飞行中才上传
      if (dataToSend.gps_valid || dataToSend.phase > IDLE) {
        String json = "{";
        json += "\"ph\":" + String(dataToSend.phase) + ",";
        json += "\"lat\":" + String(dataToSend.lat, 6) + ",";
        json += "\"lon\":" + String(dataToSend.lon, 6) + ",";
        json += "\"alt\":" + String(dataToSend.gps_valid ? dataToSend.gps_alt : dataToSend.press_alt) + ",";
        json += "\"v\":" + String(dataToSend.vspeed);
        json += "}";

        // 发送请求
        http.beginRequest();
        http.post(resource);
        http.sendHeader("Content-Type", "application/json");
        http.sendHeader("Content-Length", json.length());
        http.beginBody();
        http.print(json);
        http.endRequest();

        // 读取响应 (防止 socket 堵塞)，但这步可以设超时
        int statusCode = http.responseStatusCode();
        http.stop(); // 关闭连接，虽然 HTTP Keep-Alive 更好，但为了稳健先每次重连
      }
      lastUploadTime = millis();
    }
    vTaskDelay(100); // 让出 CPU
  }
}

// ===================== 初始化 =====================

void setup() {
  Serial.begin(115200);
  
  // 创建互斥锁
  xMutex = xSemaphoreCreateMutex();

  // 1. 初始化传感器总线
  Wire.begin(SDA_PIN, SCL_PIN);
  SPI.begin(SD_SCK, SD_MISO, SD_MOSI, SD_CS);

  // 2. 初始化 SD 卡
  if (!SD.begin(SD_CS, SPI, 20000000)) { // 提高 SPI 频率
    Serial.println("SD Fail!");
  } else {
    String fname = createFilename();
    dataFile = SD.open(fname.c_str(), FILE_WRITE);
    if(dataFile) {
      dataFile.println("time,phase,ax,ay,az,gx,gy,gz,alt_baro,alt_gps,lat,lon,vspeed");
      dataFile.flush();
      Serial.println("SD OK: " + fname);
    }
  }

  // 3. 初始化传感器
  imu.begin(); 
  // 此处根据实际库配置 IMU 量程，建议火箭用最大量程
  // imu.setAccelRange(16G); 
  
  if (!bmp.begin_I2C()) {
    Serial.println("BMP388 Fail!");
  }
  bmp.setTemperatureOversampling(BMP3_OVERSAMPLING_8X);
  bmp.setPressureOversampling(BMP3_OVERSAMPLING_4X);
  bmp.setIIRFilterCoeff(BMP3_IIR_FILTER_COEFF_3);
  bmp.setOutputDataRate(BMP3_ODR_50_HZ);

  // 4. 初始化 GPS
  Serial1.begin(9600, SERIAL_8N1, GPS_RX, GPS_TX);

  // 5. 初始化 4G (SIM7600)
  SerialAT.begin(SIM_BAUD, SERIAL_8N1, SIM_RX, SIM_TX);
  sim7600PowerOn();
  
  Serial.println("Initializing modem...");
  modem.restart();
  if(modem.waitForNetwork()) {
      Serial.println("Network connected");
      if (modem.gprsConnect(apn, user, pass)) {
          Serial.println("GPRS connected");
          // 只有网络连接成功，才启动遥测任务
          xTaskCreatePinnedToCore(
            TelemetryLoop,   "Telemetry",  8192,  NULL,  1,  &TaskTelemetry,  0 // 运行在 Core 0
          );
      }
  }

  Serial.println("System Ready.");
}

// ===================== 核心 1：主循环 (高频采集) =====================

void loop() {
  static uint32_t lastLogTime = 0;
  static float lastAlt = 0;
  static uint32_t lastAltTime = 0;
  
  // --- 1. 读取 GPS (非阻塞) ---
  while (Serial1.available()) gps.encode(Serial1.read());

  // --- 2. 周期性读取传感器与记录 ---
  if (millis() - lastLogTime >= LOG_INTERVAL) {
    
    // 读取 IMU
    // imu.readSensor(); // 视具体库的函数名而定
    float ax = 0, ay = 0, az = 0; // 替换为实际读取代码
    float gx = 0, gy = 0, gz = 0;
    
    // 读取气压
    bmp.performReading();
    float baro_alt = bmp.readAltitude(SEALEVELPRESSURE_HPA);

    // 计算垂直速度 (简单差分)
    float dt = (millis() - lastAltTime) / 1000.0;
    float vspeed = 0;
    if (dt > 0) {
      vspeed = (baro_alt - lastAlt) / dt;
    }
    lastAlt = baro_alt;
    lastAltTime = millis();

    // 更新状态机 (简单示例)
    static int phase = IDLE;
    if (phase == IDLE && vspeed > 2.0 && baro_alt > 10) phase = ASCENT;
    if (phase == ASCENT && vspeed < -1.0) phase = APOGEE;
    if (phase == APOGEE && vspeed < -2.0) phase = DESCENT;
    if (phase == DESCENT && fabs(vspeed) < 0.5 && baro_alt < 10) phase = LANDED;

    // --- 3. 更新全局数据 (供 Core 0 上传使用) ---
    if (xSemaphoreTake(xMutex, (TickType_t)5) == pdTRUE) {
      currentData.ax = ax; currentData.ay = ay; currentData.az = az;
      currentData.press_alt = baro_alt;
      currentData.vspeed = vspeed;
      currentData.phase = phase;
      currentData.timestamp = millis();
      if (gps.location.isValid()) {
        currentData.lat = gps.location.lat();
        currentData.lon = gps.location.lng();
        currentData.gps_alt = gps.altitude.meters();
        currentData.gps_valid = true;
      } else {
        currentData.gps_valid = false;
      }
      xSemaphoreGive(xMutex);
    }

    // --- 4. 写入 SD 卡 (Core 1 处理) ---
    if (dataFile) {
      dataFile.printf("%lu,%d,%.2f,%.2f,%.2f,%.2f,%.2f,%.2f,%.2f,%.2f,%.6f,%.6f,%.2f\n",
        millis(), phase, ax, ay, az, gx, gy, gz, 
        baro_alt, currentData.gps_alt, currentData.lat, currentData.lon, vspeed
      );
      
      // 只有在关键时刻或缓冲区满时才 flush，减少延迟
      static int flushCounter = 0;
      if (flushCounter++ > 50) { // 每50条记录(1秒)存盘一次
        dataFile.flush();
        flushCounter = 0;
      }
    }
    
    lastLogTime = millis();
  }
}
