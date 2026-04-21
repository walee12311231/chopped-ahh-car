#include <Arduino.h>
#include <WiFi.h>
#include <Wire.h>
#include <Adafruit_PWMServoDriver.h>
#include "slam.h"

static const char *AP_SSID = "SLAM_CAR";
static const char *AP_PASS = "slamcar123";
static const uint16_t TCP_PORT = 8080;

static constexpr int RX2_PIN = 16;   // TF-Luna TX → ESP32 RX2
static constexpr int TX2_PIN = 17;   // TF-Luna RX → ESP32 TX2
// PCA9685 uses default I2C: SDA=21, SCL=22

static constexpr int    SERVO_CHANNEL   = 0;
static constexpr int    SERVO_MIN_PULSE = 150;   // ~0°   (tune for your servo)
static constexpr int    SERVO_MAX_PULSE = 600;   // ~180° (tune for your servo)
static constexpr float  SERVO_MIN_DEG   = 0.0f;
static constexpr float  SERVO_MAX_DEG   = 180.0f;
static constexpr float  SERVO_STEP_DEG  = 5.0f;  // per arrow-key press
static constexpr int    SETTLE_MS       = 20;    // wait for servo to reach position

// Map is 32 KB — must NOT be on the stack.
TFLuna              luna(Serial2);
TinySLAM            slam;
Adafruit_PWMServoDriver pwm;

WiFiServer server(TCP_PORT);
WiFiClient client;

static LidarScan scanBuffer;
static float     servo_angle_deg = 90.0f;

// Motor control stubs — hardware TBD; bodies filled once motor driver is wired.
static void moveForward()  {}
static void moveBackward() {}
static void turnLeft()     {}
static void turnRight()    {}
static void stopMotors()   {}

static int degToPulse(float deg) {
    return SERVO_MIN_PULSE +
           (int)((deg / 180.0f) * (SERVO_MAX_PULSE - SERVO_MIN_PULSE));
}

static void sendMsg(const char *msg) {
    Serial.println(msg);
    if (client.connected()) { client.print('#'); client.println(msg); }
}

static void setServoAngle(float deg) {
    if (deg < SERVO_MIN_DEG) deg = SERVO_MIN_DEG;
    if (deg > SERVO_MAX_DEG) deg = SERVO_MAX_DEG;
    servo_angle_deg = deg;
    pwm.setPWM(SERVO_CHANNEL, 0, degToPulse(deg));
}

static void servoStep(float delta_deg) {
    setServoAngle(servo_angle_deg + delta_deg);
    delay(SETTLE_MS);

    // Drop stale frames so the next read reflects the new servo position.
    while (Serial2.available()) Serial2.read();

    float dist_m = 0.0f;
    const bool ok = luna.readBlocking(dist_m, 50);

    char buf[96];
    if (ok) {
        const float angle_rad = servo_angle_deg * (float)M_PI / 180.0f;
        scanBuffer.addBeam(angle_rad, dist_m);
        snprintf(buf, sizeof(buf),
                 "SERVO deg=%.1f dist=%.2fm beams=%d",
                 servo_angle_deg, dist_m, scanBuffer.num_beams);
    } else {
        snprintf(buf, sizeof(buf),
                 "SERVO deg=%.1f dist=NA beams=%d",
                 servo_angle_deg, scanBuffer.num_beams);
    }
    sendMsg(buf);
}

static void commitScan() {
    if (scanBuffer.num_beams == 0) {
        sendMsg("SCAN_EMPTY");
        return;
    }
    const Pose p = slam.update(scanBuffer);
    char buf[128];
    snprintf(buf, sizeof(buf),
             "SCAN_DONE beams=%d x=%.3f y=%.3f theta=%.1f",
             scanBuffer.num_beams, p.x, p.y,
             p.theta * (180.0f / (float)M_PI));
    sendMsg(buf);
    scanBuffer.reset();
}

// Binary map export protocol:
//   4 bytes  magic   "SMAP"
//   2 bytes  uint16  map_size  (little-endian)
//   4 bytes  float   scale
//   4 bytes  float   pose_x
//   4 bytes  float   pose_y
//   4 bytes  float   pose_theta
//   map_size*map_size*2 bytes  uint16 cells (row-major, little-endian)
static void exportMap() {
    if (!client.connected()) {
        Serial.println("No client for export");
        return;
    }

    client.write((const uint8_t *)"SMAP", 4);

    uint16_t size  = SLAM_MAP_SIZE;
    float    scale = SLAM_MAP_SCALE;
    const Pose &p  = slam.pose();

    client.write((const uint8_t *)&size,    2);
    client.write((const uint8_t *)&scale,   4);
    client.write((const uint8_t *)&p.x,     4);
    client.write((const uint8_t *)&p.y,     4);
    client.write((const uint8_t *)&p.theta, 4);

    // Chunked write to avoid watchdog timeout.
    const uint8_t *data = (const uint8_t *)slam.map().cells;
    const size_t total  = (size_t)size * size * 2;
    const size_t chunk  = 1024;

    for (size_t off = 0; off < total; off += chunk) {
        size_t n = min(chunk, total - off);
        client.write(data + off, n);
        yield();
    }

    client.flush();

    char buf[64];
    snprintf(buf, sizeof(buf), "EXPORT_DONE %dx%d %.1fKB",
             size, size, total / 1024.0f);
    sendMsg(buf);
}

void setup() {
    Serial.begin(115200);
    Serial2.begin(115200, SERIAL_8N1, RX2_PIN, TX2_PIN);

    luna.begin(0);
    slam.init();
    scanBuffer.reset();

    Wire.begin(21, 22);
    pwm.begin();
    pwm.setPWMFreq(50);      // 50 Hz for standard hobby servos
    setServoAngle(90.0f);    // centre position

    WiFi.softAP(AP_SSID, AP_PASS);
    Serial.printf("WiFi AP: %s  pass: %s\n", AP_SSID, AP_PASS);
    Serial.printf("IP: %s  port: %d\n",
                  WiFi.softAPIP().toString().c_str(), TCP_PORT);

    server.begin();
    Serial.println("Commands: wasd=drive x=stop l/r=servo- /servo+ "
                   "c=commit-scan e=export-map");
}

void loop() {
    if (!client || !client.connected()) {
        WiFiClient newClient = server.available();
        if (newClient) {
            client = newClient;
            Serial.println("Client connected");
            client.println("#READY");
        }
    }

    if (!client || !client.connected()) return;

    while (client.available()) {
        char cmd = client.read();
        switch (cmd) {
            case 'w': moveForward();  break;
            case 'a': turnLeft();     break;
            case 's': moveBackward(); break;
            case 'd': turnRight();    break;
            case 'x': stopMotors();   break;

            case 'l': servoStep(-SERVO_STEP_DEG); break;
            case 'r': servoStep(+SERVO_STEP_DEG); break;

            case 'c': commitScan(); break;
            case 'e': exportMap();  break;

            default: break;
        }
    }
}
