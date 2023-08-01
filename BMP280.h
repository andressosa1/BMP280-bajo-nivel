#ifndef BMP280_H
#define BMP280_H

#include <Arduino.h>
#include <Wire.h>

#define BMP280_I2C_ADDRESS 0x76

class BMP280 {
public:
    BMP280();

    bool begin();
    float readTemperature();
    float readPressure();
    float readAltitude(float seaLevelhPa = 1013.25);

private:
    int16_t read16(uint8_t reg);
    uint16_t read16_LE(uint8_t reg);
    int32_t read24(uint8_t reg);
    uint32_t read32(uint8_t reg);
    void write8(uint8_t reg, uint8_t value);
    void readCoefficients();
    int32_t t_fine;

    struct {
        uint16_t dig_T1;
        int16_t  dig_T2;
        int16_t  dig_T3;

        uint16_t dig_P1;
        int16_t  dig_P2;
        int16_t  dig_P3;
        int16_t  dig_P4;
        int16_t  dig_P5;
        int16_t  dig_P6;
        int16_t  dig_P7;
        int16_t  dig_P8;
        int16_t  dig_P9;
    } _bmp280_calib;
};

#endif
