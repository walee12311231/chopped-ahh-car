#include <Arduino.h>
#include <Wire.h>
#include <WiFi.h>
#include "rplidar.h"
#include "packet.h"
#include "mpu6050.h"
#include "L9110S_Drive.h"

#define WIFI_SSID     "iShrihanESP"
#define WIFI_PASSWORD "slamcar123"
#define TCP_PORT      8080

#define LIDAR_RX_PIN  16     // ESP32 RX <- LIDAR TX
#define LIDAR_TX_PIN  17     // ESP32 TX -> LIDAR RX
#define LIDAR_MOT_PIN 25     // PWM to MOTOCTL
#define LIDAR_PWMCH   0

// IMU on default ESP32 I2C bus (Wire). SDA=21, SCL=22.
#define IMU_SDA_PIN   21
#define IMU_SCL_PIN   22

// Drivetrain (L9110S H-bridge). Enabled so the host can teleop via WASD.
#define ENABLE_DRIVE 1
#define DRV_BIA 26
#define DRV_BIB 27
#define DRV_AIA 32
#define DRV_AIB 33

#if ENABLE_DRIVE
static L9110SDrive drive(DRV_BIA, DRV_BIB, DRV_AIA, DRV_AIB);
#endif

WiFiServer server(TCP_PORT);
WiFiClient client;
HardwareSerial LidarSerial(2);

static MPU6050 imu(Wire);

static rplidar::Sample g_rev[rplidar::MAX_SAMPLES_PER_REV];
static uint8_t         g_tx[16 + 5 * rplidar::MAX_SAMPLES_PER_REV + 1];
static uint8_t         g_imu_tx[packet::IMU_FRAME_BYTES];
static uint32_t        g_seq = 0;
static bool            g_scanning = false;
static bool            g_imu_ok   = false;

// Yaw integration state. Stored in millidegrees to keep precision over long
// runs; converted to centidegrees only when packed into the IMU frame.
static int32_t  g_yaw_mdeg   = 0;
static uint32_t g_imu_last_us = 0;
static int16_t  g_ax_lsb = 0, g_ay_lsb = 0, g_az_lsb = 0;

static void logLine(const char* s) {
  Serial.println(s);
  if (client && client.connected()) { client.print("#"); client.println(s); }
}

static int16_t getYawCdeg() {
  int32_t cdeg = g_yaw_mdeg / 10;
  if (cdeg >  18000) cdeg =  18000;
  if (cdeg < -18000) cdeg = -18000;
  return (int16_t)cdeg;
}

// Read IMU once and integrate gyro Z into yaw. Call from loop() at >=50 Hz.
static void imuTick() {
  if (!g_imu_ok) return;
  int16_t raw[7];
  if (!imu.readRaw(raw)) return;
  g_ax_lsb = raw[0];
  g_ay_lsb = raw[1];
  g_az_lsb = raw[2];
  // raw[3] = temperature, raw[4..6] = gx, gy, gz (LSB at the configured FS)

  uint32_t now = micros();
  uint32_t dt_us = now - g_imu_last_us;
  g_imu_last_us = now;
  if (dt_us > 200000) return; // first call or stall — skip this dt

  // Convert gyro Z LSB to dps using the class's current sensitivity.
  // We configured GYRO_250 in setup, so 131 LSB/dps.
  constexpr float GZ_LSB_PER_DPS = 131.0f;
  float dps = (float)raw[6] / GZ_LSB_PER_DPS;
  // dps * us * 1e-3 = millidegrees
  g_yaw_mdeg += (int32_t)(dps * (float)dt_us * 0.001f);
  while (g_yaw_mdeg >  180000) g_yaw_mdeg -= 360000;
  while (g_yaw_mdeg < -180000) g_yaw_mdeg += 360000;
}

static void handleCmd(char c) {
  switch (c) {
    case 's':
      rplidar::setMotor(true);
      delay(800);
      rplidar::startScan();
      g_scanning = true;
      logLine("SCAN STARTED");
      break;
    case 'x':
      g_scanning = false;
      rplidar::stopScan();
      rplidar::setMotor(false);
      logLine("SCAN STOPPED");
      break;
    case 'm': rplidar::setMotor(true);  logLine("MOTOR ON");  break;
    case 'o': rplidar::setMotor(false); logLine("MOTOR OFF"); break;
    case 'z': g_yaw_mdeg = 0;           logLine("YAW ZEROED"); break;
#if ENABLE_DRIVE
    case 'f': drive.forward();   logLine("DRIVE FWD");  break;
    case 'b': drive.backward();  logLine("DRIVE BACK"); break;
    case 'l': drive.turnLeft();  logLine("DRIVE LEFT"); break;
    case 'r': drive.turnRight(); logLine("DRIVE RIGHT"); break;
    case 'p': drive.stop();      logLine("DRIVE STOP"); break;
#endif
    case 'h':
#if ENABLE_DRIVE
      logLine("s/x scan  m/o lidar-motor  f/b/l/r drive  p stop  z zero-yaw");
#else
      logLine("s/x scan  m/o lidar-motor  z zero-yaw  (drive disabled)");
#endif
      break;
    case '\r': case '\n': case ' ': break;
    default: { char b[32]; snprintf(b, sizeof b, "UNKNOWN %c", c); logLine(b); }
  }
}

void setup() {
  Serial.begin(115200);
  delay(200);
  Serial.println("[BOOT] RPLIDAR A1 + MPU6050 bridge");

  rplidar::begin(LidarSerial, LIDAR_RX_PIN, LIDAR_TX_PIN, LIDAR_MOT_PIN, LIDAR_PWMCH);

  // MPU6050 calibration is a few-hundred-ms blocking step; robot must be still.
  // GYRO_250 keeps 131 LSB/dps (matches GZ_LSB_PER_DPS in imuTick).
  g_imu_ok = imu.autoInit(Serial, IMU_SDA_PIN, IMU_SCL_PIN, 400000,
                          MPU6050::ACCEL_2G, MPU6050::GYRO_250, 500);
  Serial.printf("[IMU] init %s\n", g_imu_ok ? "OK" : "FAIL");
  g_imu_last_us = micros();

#if ENABLE_DRIVE
  drive.begin();
#endif

  WiFi.persistent(false);
  WiFi.disconnect(true, true);
  WiFi.softAPdisconnect(true);
  delay(100);
  WiFi.mode(WIFI_AP);

  IPAddress apIP(192, 168, 4, 1);
  IPAddress apGW(192, 168, 4, 1);
  IPAddress apMask(255, 255, 255, 0);
  if (!WiFi.softAPConfig(apIP, apGW, apMask)) {
    Serial.println("[FATAL] softAPConfig failed");
    while (1) delay(1000);
  }
  if (!WiFi.softAP(WIFI_SSID, WIFI_PASSWORD, 6, 0, 4)) {
    Serial.println("[FATAL] softAP failed");
    while (1) delay(1000);
  }
  WiFi.setSleep(false);
  WiFi.setTxPower(WIFI_POWER_19_5dBm);
  delay(200);

  Serial.print("[WIFI] SSID: "); Serial.println(WIFI_SSID);
  Serial.print("[WIFI] AP IP: "); Serial.println(WiFi.softAPIP());
  Serial.print("[WIFI] AP MAC: "); Serial.println(WiFi.softAPmacAddress());

  server.begin();
  server.setNoDelay(true);

  rplidar::setMotor(true);
  delay(800);
  rplidar::startScan();
  g_scanning = true;
  Serial.println("[LIDAR] AUTO STARTED");
}

void loop() {
  static uint32_t lastStatus = 0;
  static uint32_t lastImuPoll = 0;
  uint32_t now = millis();

  // Poll IMU at ~200 Hz regardless of scan state. Yaw integration accuracy
  // depends on consistent dt — keep this loop tight (no long blocking work).
  if ((now - lastImuPoll) >= 5) {
    lastImuPoll = now;
    imuTick();
  }

  if (now - lastStatus > 5000) {
    lastStatus = now;
    Serial.printf("[WIFI] stations=%d  client=%d  yaw=%d cdeg\n",
                  WiFi.softAPgetStationNum(),
                  (client && client.connected()) ? 1 : 0,
                  (int)getYawCdeg());
  }

  if (!client || !client.connected()) {
    WiFiClient nc = server.available();
    if (nc) {
      client = nc;
      client.setNoDelay(true);
      logLine("READY");
    }
  }

  if (client && client.connected()) {
    while (client.available()) handleCmd((char)client.read());
  }

  if (g_scanning) {
    size_t n = 0; bool trunc = false;
    if (rplidar::poll(g_rev, rplidar::MAX_SAMPLES_PER_REV, n, trunc) && n > 0) {
      uint32_t t = millis();
      uint32_t scan_seq = ++g_seq;

      size_t bytes = packet::buildScan(g_tx, sizeof(g_tx),
                                       scan_seq, t, g_rev, n, trunc);
      if (bytes && client && client.connected()) {
        client.write(g_tx, bytes);
      }

      // Pair every revolution with one fresh IMU frame so the host can
      // re-orient the scan into world frame. Matching seq makes correlation easy.
      if (g_imu_ok && client && client.connected()) {
        size_t ib = packet::buildImu(g_imu_tx, sizeof(g_imu_tx),
                                     scan_seq, t,
                                     getYawCdeg(),
                                     g_ax_lsb, g_ay_lsb, g_az_lsb);
        if (ib) client.write(g_imu_tx, ib);
      }
    }
  }
}
