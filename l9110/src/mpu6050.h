#pragma once

#include <Arduino.h>
#include <Wire.h>

class MPU6050 {
public:
    enum AccelRange : uint8_t {
        ACCEL_2G  = 0,
        ACCEL_4G  = 1,
        ACCEL_8G  = 2,
        ACCEL_16G = 3
    };

    enum GyroRange : uint8_t {
        GYRO_250  = 0,
        GYRO_500  = 1,
        GYRO_1000 = 2,
        GYRO_2000 = 3
    };

    struct Vec3 {
        float x;
        float y;
        float z;
    };

    struct Sample {
        Vec3  accel_g;
        Vec3  gyro_dps;
        float temp_c;
    };

    explicit MPU6050(TwoWire& wire, uint8_t address = 0x68);

    bool begin(int sda, int scl, uint32_t i2c_hz = 400000);
    bool testConnection();

    bool setAccelRange(AccelRange range);
    bool setGyroRange(GyroRange range);

    bool readRaw(int16_t out[7]);
    bool read(Sample& out);

    void setAccelOffsets(float x, float y, float z);
    void setGyroOffsets(float x, float y, float z);
    bool calibrate(uint16_t samples = 1000);

    bool autoInit(Stream& log,
                  int sda, int scl,
                  uint32_t i2c_hz = 400000,
                  AccelRange accel = ACCEL_4G,
                  GyroRange  gyro  = GYRO_500,
                  uint16_t   calibration_samples = 1000);

    void streamSample(Stream& out);

private:
    bool writeReg(uint8_t reg, uint8_t value);
    bool readRegs(uint8_t reg, uint8_t* buffer, size_t length);

    TwoWire& _wire;
    uint8_t  _address;
    float    _accel_lsb_per_g;
    float    _gyro_lsb_per_dps;
    Vec3     _accel_offset;
    Vec3     _gyro_offset;
};