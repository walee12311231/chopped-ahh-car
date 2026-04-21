#pragma once
#include <Arduino.h>
#include <Wire.h>

#define TFLUNA_FRAME_HEADER  0x59u
#define TFLUNA_FRAME_LEN     9        // 2 hdr + 2 dist + 2 flux + 2 temp + 1 cs
#define TFLUNA_UART_BAUD     115200
#define TFLUNA_I2C_ADDR      0x10
#define TFLUNA_MIN_DIST_M    0.2f
#define TFLUNA_MAX_DIST_M    8.0f

// UART constructor:  TFLuna luna(Serial2);
// I2C  constructor:  TFLuna luna(Wire, 0x10);
class TFLuna {
public:
    explicit TFLuna(HardwareSerial &serial);
    TFLuna(TwoWire &wire, uint8_t addr = TFLUNA_I2C_ADDR);

    // Call once in setup().  For UART mode, begin(baud) configures the serial
    // port; pass 0 to skip (if you already called Serial2.begin() yourself).
    void begin(uint32_t uart_baud = TFLUNA_UART_BAUD);

    // Non-blocking.  strength (optional) = raw signal amplitude.
    bool read(float &distance_m, uint16_t *strength = nullptr);

    bool readBlocking(float &distance_m,
                      uint32_t timeout_ms = 100,
                      uint16_t *strength  = nullptr);

    float lastDistance() const { return _lastDist; }

private:
    enum { UART_MODE, I2C_MODE } _mode;
    HardwareSerial *_serial = nullptr;
    TwoWire        *_wire   = nullptr;
    uint8_t         _addr   = TFLUNA_I2C_ADDR;
    float           _lastDist = 0.0f;

    bool readUART(float &dist, uint16_t *strength);
    bool readI2C (float &dist, uint16_t *strength);

    static uint8_t checksum(const uint8_t *buf, int len);
};
