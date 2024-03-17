/**
 * @file esp_bmp280.h
 * @author JanG175
 * @brief ESP-IDF component for the BMP280 temperature and pressure sensor (I2C only)
 * 
 * @copyright Apache 2.0
*/

#include <stdio.h>
#include <math.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/i2c.h"
#include "esp_log.h"

// #define BMP280_I2C_INIT   1 // uncomment to initialize I2C driver

#define BMP_I2C_ADDRESS_0 0x76 // if SDO pin is low
#define BMP_I2C_ADDRESS_1 0x77 // if SDO pin is high

#define BMP280_MAX_FREQ   3400000 // 3.4 MHz

#define BMP280_TIMEOUT    (100 / portTICK_PERIOD_MS)

// register map
#define BMP280_TEMP_XLSB  0xFC
#define BMP280_TEMP_LSB   0xFB
#define BMP280_TEMP_MSB   0xFA
#define BMP280_PRESS_XLSB 0xF9
#define BMP280_PRESS_LSB  0xF8
#define BMP280_PRESS_MSB  0xF7
#define BMP280_CONFIG     0xF5
#define BMP280_CTRL_MEAS  0xF4
#define BMP280_STATUS     0xF3
#define BMP280_RESET      0xE0
#define BMP280_ID         0xD0
#define BMP280_CALIB_00   0x88
#define BMP280_CALIB_25   0xA1

// reset states
#define BMP280_RESET_0    0x00
#define BMP280_RESET_1    0x80
#define BMP280_RESET_2    0x58

// physical constants
#define R    8.314462618 // J/(mol*K)
#define M    0.02896968 // kg/mol
#define G    9.810665 // m/s^2
#define P0   101325.0 // Pa
#define T0   288.15 // K

typedef struct
{
    uint8_t i2c_addr;
    i2c_port_t i2c_port;
    gpio_num_t sda_pin;
    gpio_num_t scl_pin;
    uint32_t i2c_freq;
} bmp280_conf_t;

// BMP280 resolutions
enum bmp280_res
{
    BMP280_ULTRA_LOW_POWER = 0,
    BMP280_LOW_POWER,
    BMP280_STANDARD,
    BMP280_STANDARD_PLUS,
    BMP280_HIGH_RES,
    BMP280_ULTRA_HIGH_RES
};


void bmp280_init(bmp280_conf_t bmp, enum bmp280_res res);

void bmp280_deinit(bmp280_conf_t bmp);

void bmp280_read_id(bmp280_conf_t bmp, uint8_t* id);

void bmp280_reset(bmp280_conf_t bmp);

void bmp280_read_status(bmp280_conf_t bmp, uint8_t* status);

void bmp280_read_ctrl_meas(bmp280_conf_t bmp, uint8_t* ctrl_meas);

void bmp280_write_ctrl_meas(bmp280_conf_t bmp, uint8_t* ctrl_meas);

void bmp280_read_config(bmp280_conf_t bmp, uint8_t* config);

void bmp280_write_config(bmp280_conf_t bmp, uint8_t* config);

void bmp280_read_temp_and_press(bmp280_conf_t bmp, double* temp_degC, double* press_Pa);

void bmp280_read_height(bmp280_conf_t bmp, double* height_m);
