#include "TFLuna.h"

TFLuna::TFLuna(HardwareSerial &serial)
    : _mode(UART_MODE), _serial(&serial) {}

TFLuna::TFLuna(TwoWire &wire, uint8_t addr)
    : _mode(I2C_MODE), _wire(&wire), _addr(addr) {}

void TFLuna::begin(uint32_t uart_baud) {
    if (_mode == UART_MODE) {
        if (uart_baud) _serial->begin(uart_baud);
    } else {
        _wire->begin();
    }
}

bool TFLuna::read(float &distance_m, uint16_t *strength) {
    return (_mode == UART_MODE)
        ? readUART(distance_m, strength)
        : readI2C (distance_m, strength);
}

bool TFLuna::readBlocking(float &distance_m,
                          uint32_t timeout_ms,
                          uint16_t *strength) {
    const uint32_t deadline = millis() + timeout_ms;
    while (millis() < deadline) {
        if (read(distance_m, strength)) return true;
        yield();
    }
    return false;
}

uint8_t TFLuna::checksum(const uint8_t *buf, int len) {
    uint8_t cs = 0;
    for (int i = 0; i < len; i++) cs += buf[i];
    return cs;
}

// TF-Luna UART frame (9 bytes):
//   [0] 0x59  [1] 0x59  [2] Dist_L  [3] Dist_H
//   [4] Flux_L [5] Flux_H [6] Temp_L [7] Temp_H  [8] checksum
bool TFLuna::readUART(float &dist, uint16_t *strength) {
    while (_serial->available() >= TFLUNA_FRAME_LEN) {
        // Sync on first header byte.
        if ((uint8_t)_serial->peek() != TFLUNA_FRAME_HEADER) {
            _serial->read();
            continue;
        }

        uint8_t buf[TFLUNA_FRAME_LEN];
        if (_serial->readBytes(buf, TFLUNA_FRAME_LEN) < TFLUNA_FRAME_LEN)
            return false;

        if (buf[0] != TFLUNA_FRAME_HEADER ||
            buf[1] != TFLUNA_FRAME_HEADER ||
            buf[8] != checksum(buf, 8)) {
            // Misaligned — try again starting from buf[1].
            continue;
        }

        const uint16_t raw = (uint16_t)buf[2] | ((uint16_t)buf[3] << 8);
        if (strength) *strength = (uint16_t)buf[4] | ((uint16_t)buf[5] << 8);

        const float d = raw * 0.01f;  // cm → metres
        if (d < TFLUNA_MIN_DIST_M || d > TFLUNA_MAX_DIST_M) return false;

        _lastDist = d;
        dist = d;
        return true;
    }
    return false;
}

// I2C registers 0x00-0x05: Dist_L, Dist_H, Flux_L, Flux_H, Temp_L, Temp_H
bool TFLuna::readI2C(float &dist, uint16_t *strength) {
    _wire->beginTransmission(_addr);
    _wire->write(0x00);
    if (_wire->endTransmission(false) != 0) return false;

    if (_wire->requestFrom(_addr, (uint8_t)6) < 6) return false;

    uint8_t buf[6];
    for (int i = 0; i < 6; i++) buf[i] = _wire->read();

    const uint16_t raw = (uint16_t)buf[0] | ((uint16_t)buf[1] << 8);
    if (strength) *strength = (uint16_t)buf[2] | ((uint16_t)buf[3] << 8);

    const float d = raw * 0.01f;  // cm → metres
    if (d < TFLUNA_MIN_DIST_M || d > TFLUNA_MAX_DIST_M) return false;

    _lastDist = d;
    dist = d;
    return true;
}
