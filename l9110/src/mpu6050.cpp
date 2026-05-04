#include "MPU6050.h"

namespace {
constexpr uint8_t REG_SMPLRT_DIV   = 0x19;
constexpr uint8_t REG_CONFIG       = 0x1A;
constexpr uint8_t REG_GYRO_CONFIG  = 0x1B;
constexpr uint8_t REG_ACCEL_CONFIG = 0x1C;
constexpr uint8_t REG_ACCEL_XOUT_H = 0x3B;
constexpr uint8_t REG_PWR_MGMT_1   = 0x6B;
constexpr uint8_t REG_WHO_AM_I     = 0x75;
constexpr uint8_t WHO_AM_I_VALUE   = 0x68;

constexpr float ACCEL_LSB_PER_G[4]  = { 16384.0f, 8192.0f, 4096.0f, 2048.0f };
constexpr float GYRO_LSB_PER_DPS[4] = { 131.0f, 65.5f, 32.8f, 16.4f };
}

MPU6050::MPU6050(TwoWire& wire, uint8_t address)
    : _wire(wire),
      _address(address),
      _accel_lsb_per_g(ACCEL_LSB_PER_G[ACCEL_2G]),
      _gyro_lsb_per_dps(GYRO_LSB_PER_DPS[GYRO_250]),
      _accel_offset{0.0f, 0.0f, 0.0f},
      _gyro_offset{0.0f, 0.0f, 0.0f} {}

bool MPU6050::begin(int sda, int scl, uint32_t i2c_hz) {
    // Arduino-ESP32 TwoWire takes SDA/SCL directly in begin(); there is no
    // setSDA/setSCL on this core (those exist on RP2040/mbed cores).
    _wire.begin(sda, scl, i2c_hz);
    _wire.setClock(i2c_hz);

    if (!writeReg(REG_PWR_MGMT_1, 0x80)) return false;
    delay(100);
    if (!writeReg(REG_PWR_MGMT_1, 0x01)) return false;
    if (!writeReg(REG_CONFIG, 0x03))     return false;
    if (!writeReg(REG_SMPLRT_DIV, 0x00)) return false;

    return testConnection();
}

bool MPU6050::testConnection() {
    uint8_t who = 0;
    if (!readRegs(REG_WHO_AM_I, &who, 1)) return false;
    return who == WHO_AM_I_VALUE;
}

bool MPU6050::setAccelRange(AccelRange range) {
    if (!writeReg(REG_ACCEL_CONFIG, static_cast<uint8_t>(range) << 3)) return false;
    _accel_lsb_per_g = ACCEL_LSB_PER_G[range];
    return true;
}

bool MPU6050::setGyroRange(GyroRange range) {
    if (!writeReg(REG_GYRO_CONFIG, static_cast<uint8_t>(range) << 3)) return false;
    _gyro_lsb_per_dps = GYRO_LSB_PER_DPS[range];
    return true;
}

bool MPU6050::readRaw(int16_t out[7]) {
    uint8_t buf[14];
    if (!readRegs(REG_ACCEL_XOUT_H, buf, sizeof(buf))) return false;
    for (size_t i = 0; i < 7; ++i) {
        out[i] = static_cast<int16_t>((buf[i * 2] << 8) | buf[i * 2 + 1]);
    }
    return true;
}

bool MPU6050::read(Sample& out) {
    int16_t raw[7];
    if (!readRaw(raw)) return false;

    out.accel_g.x  = raw[0] / _accel_lsb_per_g - _accel_offset.x;
    out.accel_g.y  = raw[1] / _accel_lsb_per_g - _accel_offset.y;
    out.accel_g.z  = raw[2] / _accel_lsb_per_g - _accel_offset.z;
    out.temp_c     = raw[3] / 340.0f + 36.53f;
    out.gyro_dps.x = raw[4] / _gyro_lsb_per_dps - _gyro_offset.x;
    out.gyro_dps.y = raw[5] / _gyro_lsb_per_dps - _gyro_offset.y;
    out.gyro_dps.z = raw[6] / _gyro_lsb_per_dps - _gyro_offset.z;
    return true;
}

void MPU6050::setAccelOffsets(float x, float y, float z) { _accel_offset = {x, y, z}; }
void MPU6050::setGyroOffsets(float x, float y, float z)  { _gyro_offset  = {x, y, z}; }

bool MPU6050::calibrate(uint16_t samples) {
    if (samples == 0) return false;

    _accel_offset = {0.0f, 0.0f, 0.0f};
    _gyro_offset  = {0.0f, 0.0f, 0.0f};

    double ax = 0, ay = 0, az = 0;
    double gx = 0, gy = 0, gz = 0;

    for (uint16_t i = 0; i < samples; ++i) {
        Sample s;
        if (!read(s)) return false;
        ax += s.accel_g.x;  ay += s.accel_g.y;  az += s.accel_g.z;
        gx += s.gyro_dps.x; gy += s.gyro_dps.y; gz += s.gyro_dps.z;
        delay(1);
    }

    const double n = samples;
    _accel_offset = {
        static_cast<float>(ax / n),
        static_cast<float>(ay / n),
        static_cast<float>(az / n) - 1.0f
    };
    _gyro_offset = {
        static_cast<float>(gx / n),
        static_cast<float>(gy / n),
        static_cast<float>(gz / n)
    };
    return true;
}

bool MPU6050::autoInit(Stream& log, int sda, int scl, uint32_t i2c_hz, AccelRange accel, GyroRange gyro, uint16_t calibration_samples) {
    if (!begin(sda, scl, i2c_hz)) {
        log.println("MPU6050 init FAILED");
        return false;
    }

    uint8_t who = 0;
    readRegs(REG_WHO_AM_I, &who, 1);
    log.print("MPU6050 OK (WHO_AM_I=0x"); log.print(who, HEX); log.println(")");

    if (!setAccelRange(accel) || !setGyroRange(gyro)) {
        log.println("Range config failed");
        return false;
    }

    log.println("Calibrating - keep sensor still and level...");
    if (calibrate(calibration_samples)) {
        log.println("Calibration done.");
        return true;
    }
    log.println("Calibration failed (bus error).");
    return false;
}

void MPU6050::streamSample(Stream& out) {
    Sample s;
    if (!read(s)) { out.println("read failed"); return; }
    out.print("a[g]=");
    out.print(s.accel_g.x, 3); out.print(',');
    out.print(s.accel_g.y, 3); out.print(',');
    out.print(s.accel_g.z, 3);
    out.print("  g[dps]=");
    out.print(s.gyro_dps.x, 2); out.print(',');
    out.print(s.gyro_dps.y, 2); out.print(',');
    out.print(s.gyro_dps.z, 2);
    out.print("  T[C]=");
    out.println(s.temp_c, 2);
}

bool MPU6050::writeReg(uint8_t reg, uint8_t value) {
    _wire.beginTransmission(_address);
    _wire.write(reg);
    _wire.write(value);
    return _wire.endTransmission() == 0;
}

bool MPU6050::readRegs(uint8_t reg, uint8_t* buffer, size_t length) {
    _wire.beginTransmission(_address);
    _wire.write(reg);
    if (_wire.endTransmission(false) != 0) return false;
    const uint8_t n = static_cast<uint8_t>(length);
    if (_wire.requestFrom(_address, n) != n) return false;
    for (size_t i = 0; i < length; ++i) {
        int b = _wire.read();
        if (b < 0) return false;
        buffer[i] = static_cast<uint8_t>(b);
    }
    return true;
}