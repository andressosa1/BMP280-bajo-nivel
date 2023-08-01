#include "BMP280.h"

BMP280::BMP280() {}

bool BMP280::begin() {
    if (read8(0xD0) != 0x58)
        return false;

    readCoefficients();
    write8(0xF4, 0x3F);
    return true;
}

float BMP280::readTemperature() {
    int32_t var1, var2;

    int32_t adc_T = read16(BMP280_REGISTER_TEMPDATA);
    adc_T <<= 8;
    adc_T |= read8(BMP280_REGISTER_TEMPDATA+2);
    adc_T >>= 4;

    var1 = ((((adc_T>>3) - ((int32_t)_bmp280_calib.dig_T1 <<1))) *
            ((int32_t)_bmp280_calib.dig_T2)) >> 11;

    var2 = (((((adc_T>>4) - ((int32_t)_bmp280_calib.dig_T1)) *
              ((adc_T>>4) - ((int32_t)_bmp280_calib.dig_T1))) >> 12) *
            ((int32_t)_bmp280_calib.dig_T3)) >> 14;

    t_fine = var1 + var2;

    float T  = (t_fine * 5 + 128) >> 8;
    return T/100;
}

float BMP280::readPressure() {
    int64_t var1, var2, p;

    // Call readTemperature to get t_fine
    readTemperature();

    int32_t adc_P = read16(BMP280_REGISTER_PRESSUREDATA);
    adc_P <<= 8;
    adc_P |= read8(BMP280_REGISTER_PRESSUREDATA+2);
    adc_P >>= 4;

    var1 = ((int64_t)t_fine) - 128000;
    var2 = var1 * var1 * (int64_t)_bmp280_calib.dig_P6;
    var2 = var2 + ((var1*(int64_t)_bmp280_calib.dig_P5)<<17);
    var2 = var2 + (((int64_t)_bmp280_calib.dig_P4)<<35);
    var1 = ((var1 * var1 * (int64_t)_bmp280_calib.dig_P3)>>8) +
            ((var1 * (int64_t)_bmp280_calib.dig_P2)<<12);
    var1 = (((((int64_t)1)<<47)+var1))*((int64_t)_bmp280_calib.dig_P1)>>33;

    if (var1 == 0) {
        return 0;  // avoid exception caused by division by zero
    }
    p = 1048576 - adc_P;
    p = (((p<<31) - var2)*3125) / var1;
    var1 = (((int64_t)_bmp280_calib.dig_P9) * (p>>13) * (p>>13)) >> 25;
    var2 = (((int64_t)_bmp280_calib.dig_P8) * p) >> 19;

    p = ((p + var1 + var2) >> 8) + (((int64_t)_bmp280_calib.dig_P7)<<4);
    return (float)p/256;
}

float BMP280::readAltitude(float seaLevelhPa) {
    float altitude;

    float pressure = readPressure(); // in Si units for Pascal
    pressure /= 100;

    altitude = 44330 * (1.0 - pow(pressure / seaLevelhPa, 0.1903));

    return altitude;
}

uint8_t BMP280::read8(byte reg) {
    Wire.beginTransmission(BMP280_I2C_ADDRESS);
    Wire.write(reg);
    Wire.endTransmission();
    Wire.requestFrom(BMP280_I2C_ADDRESS, 1);
    while(!Wire.available());
    return Wire.read();
}

uint16_t BMP280::read16(byte reg) {
    Wire.beginTransmission(BMP280_I2C_ADDRESS);
    Wire.write(reg);
    Wire.endTransmission();
    Wire.requestFrom(BMP280_I2C_ADDRESS, 2);
    while(Wire.available() < 2);
    uint16_t t = Wire.read() | Wire.read() << 8;
    return t;
}

uint16_t BMP280::read16_LE(byte reg) {
    uint16_t temp = read16(reg);
    return (temp >> 8) | (temp << 8);
}

int32_t BMP280::read24(byte reg) {
    uint32_t data;

    Wire.beginTransmission(BMP280_I2C_ADDRESS);
    Wire.write(reg);
    Wire.endTransmission();
    Wire.requestFrom(BMP280_I2C_ADDRESS, 3);
    while(Wire.available() < 3);
    data = Wire.read();
    data <<= 8;
    data |= Wire.read();
    data <<= 8;
    data |= Wire.read();

    return data;
}

void BMP280::readCoefficients() {
    _bmp280_calib.dig_T1 = read16_LE(BMP280_REGISTER_DIG_T1);
    _bmp280_calib.dig_T2 = read16_LE(BMP280_REGISTER_DIG_T2);
    _bmp280_calib.dig_T3 = read16_LE(BMP280_REGISTER_DIG_T3);

    _bmp280_calib.dig_P1 = read16_LE(BMP280_REGISTER_DIG_P1);
    _bmp280_calib.dig_P2 = read16_LE(BMP280_REGISTER_DIG_P2);
    _bmp280_calib.dig_P3 = read16_LE(BMP280_REGISTER_DIG_P3);
    _bmp280_calib.dig_P4 = read16_LE(BMP280_REGISTER_DIG_P4);
    _bmp280_calib.dig_P5 = read16_LE(BMP280_REGISTER_DIG_P5);
    _bmp280_calib.dig_P6 = read16_LE(BMP280_REGISTER_DIG_P6);
    _bmp280_calib.dig_P7 = read16_LE(BMP280_REGISTER_DIG_P7);
    _bmp280_calib.dig_P8 = read16_LE(BMP280_REGISTER_DIG_P8);
    _bmp280_calib.dig_P9 = read16_LE(BMP280_REGISTER_DIG_P9);
}

void BMP280::write8(byte reg, byte value) {
    Wire.beginTransmission(BMP280_I2C_ADDRESS);
    Wire.write(reg);
    Wire.write(value);
    Wire.endTransmission();
}
