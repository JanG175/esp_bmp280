/**
 * @file esp_bmp280.c
 * @author JanG175
 * @brief ESP-IDF component for the BMP280 temperature and pressure sensor (I2C only)
 * 
 * @copyright Apache 2.0
*/

#include <stdio.h>
#include "esp_bmp280.h"

static double pressure_sea_level = P0;
#ifdef BMP280_I2C_INIT
static i2c_master_bus_handle_t bus_handle;
#else
extern i2c_master_bus_handle_t bus_handle;
#endif
static i2c_master_dev_handle_t dev_handle;

static portMUX_TYPE spinlock = portMUX_INITIALIZER_UNLOCKED;

static const char *TAG = "BMP280";


/**
 * @brief compensate temperature and pressure readings using integer values
 * 
 * @param bmp struct with BMP280 parameters
 * @param adc_T temperature reading
 * @param T pointer to compensated temperature
 * @param adc_P pressure reading
 * @param P pointer to compensated pressure
*/
static void bmp280_compensate_T_P_int(bmp280_conf_t bmp, int32_t adc_T, float* T, int32_t adc_P, float* P)
{
    uint8_t dig_calib[24];
    uint8_t reg = BMP280_CALIB_00;
    ESP_ERROR_CHECK(i2c_master_transmit_receive(dev_handle, &reg, 1, dig_calib, 24, BMP280_TIMEOUT_MS));

    uint16_t dig_T1 = ((uint16_t)dig_calib[1] << 8) | (uint16_t)dig_calib[0];
    int16_t dig_T2 = ((int16_t)dig_calib[3] << 8) | (int16_t)dig_calib[2];
    int16_t dig_T3 = ((int16_t)dig_calib[5] << 8) | (int16_t)dig_calib[4];

    uint16_t dig_P1 = ((uint16_t)dig_calib[7] << 8) | (uint16_t)dig_calib[6];
    int16_t dig_P2 = ((int16_t)dig_calib[9] << 8) | (int16_t)dig_calib[8];
    int16_t dig_P3 = ((int16_t)dig_calib[11] << 8) | (int16_t)dig_calib[10];
    int16_t dig_P4 = ((int16_t)dig_calib[13] << 8) | (int16_t)dig_calib[12];
    int16_t dig_P5 = ((int16_t)dig_calib[15] << 8) | (int16_t)dig_calib[14];
    int16_t dig_P6 = ((int16_t)dig_calib[17] << 8) | (int16_t)dig_calib[16];
    int16_t dig_P7 = ((int16_t)dig_calib[19] << 8) | (int16_t)dig_calib[18];
    int16_t dig_P8 = ((int16_t)dig_calib[21] << 8) | (int16_t)dig_calib[20];
    int16_t dig_P9 = ((int16_t)dig_calib[23] << 8) | (int16_t)dig_calib[22];

    int32_t tvar1, tvar2, t_fine, temp;
    int64_t pvar1, pvar2, p;
    uint32_t press;

    // temperature
    tvar1 = ((((adc_T >> 3) - ((int32_t)dig_T1 << 1))) * ((int32_t)dig_T2)) >> 11;
    tvar2 = (((((adc_T >> 4) - ((int32_t)dig_T1)) * ((adc_T >> 4) - ((int32_t)dig_T1))) >> 12) * ((int32_t)dig_T3)) >> 14;
    t_fine = tvar1 + tvar2;

    temp = (t_fine * 5 + 128) >> 8;

    // pressure
    pvar1 = ((int64_t)t_fine) - 128000;
    pvar2 = pvar1 * pvar1 * (int64_t)dig_P6;
    pvar2 = pvar2 + ((pvar1 * (int64_t)dig_P5) << 17);
    pvar2 = pvar2 + (((int64_t)dig_P4) << 35);
    pvar1 = ((pvar1 * pvar1 * (int64_t)dig_P3) >> 8) + ((pvar1 * (int64_t)dig_P2) << 12);
    pvar1 = (((1LL << 47) + pvar1)) * ((int64_t)dig_P1) >> 33;
    if (pvar1 == 0)
    {
        *P = 0;
        ESP_LOGE(TAG, "Dividing by 0!");
        return;
    }
    p = 1048576 - adc_P;
    p = (((p << 31) - pvar2) * 3125) / pvar1;
    pvar1 = (((int64_t)dig_P9) * (p >> 13) * (p >> 13)) >> 25;
    pvar2 = (((int64_t)dig_P8) * p) >> 19;
    p = ((p + pvar1 + pvar2) >> 8) + (((int64_t)dig_P7) << 4);

    press = (uint32_t)p;

    *T = (float)temp / 100.0f;
    *P = (float)press / 256.0f;
}


/**
 * @brief compensate temperature and pressure readings using double values
 * 
 * @param bmp struct with BMP280 parameters
 * @param adc_T temperature reading
 * @param T pointer to compensated temperature
 * @param adc_P pressure reading
 * @param P pointer to compensated pressure
*/
static void bmp280_compensate_T_P_double(bmp280_conf_t bmp, int32_t adc_T, double* T, int32_t adc_P, double* P)
{
    uint8_t dig_calib[24];
    uint8_t reg = BMP280_CALIB_00;
    ESP_ERROR_CHECK(i2c_master_transmit_receive(dev_handle, &reg, 1, dig_calib, 24, BMP280_TIMEOUT_MS));

    uint16_t dig_T1 = ((uint16_t)dig_calib[1] << 8) | (uint16_t)dig_calib[0];
    int16_t dig_T2 = ((int16_t)dig_calib[3] << 8) | (int16_t)dig_calib[2];
    int16_t dig_T3 = ((int16_t)dig_calib[5] << 8) | (int16_t)dig_calib[4];

    uint16_t dig_P1 = ((uint16_t)dig_calib[7] << 8) | (uint16_t)dig_calib[6];
    int16_t dig_P2 = ((int16_t)dig_calib[9] << 8) | (int16_t)dig_calib[8];
    int16_t dig_P3 = ((int16_t)dig_calib[11] << 8) | (int16_t)dig_calib[10];
    int16_t dig_P4 = ((int16_t)dig_calib[13] << 8) | (int16_t)dig_calib[12];
    int16_t dig_P5 = ((int16_t)dig_calib[15] << 8) | (int16_t)dig_calib[14];
    int16_t dig_P6 = ((int16_t)dig_calib[17] << 8) | (int16_t)dig_calib[16];
    int16_t dig_P7 = ((int16_t)dig_calib[19] << 8) | (int16_t)dig_calib[18];
    int16_t dig_P8 = ((int16_t)dig_calib[21] << 8) | (int16_t)dig_calib[20];
    int16_t dig_P9 = ((int16_t)dig_calib[23] << 8) | (int16_t)dig_calib[22];

    double tvar1, tvar2, t_fine, pvar1, pvar2, p;

    tvar1 = (((double)adc_T) / 16384.0 - ((double)dig_T1) / 1024.0) * ((double)dig_T2);
    tvar2 = ((((double)adc_T) / 131072.0 - ((double)dig_T1) / 8192.0) * (((double)adc_T) / 131072.0 - ((double)dig_T1) / 8192.0)) * ((double)dig_T3);
    t_fine = tvar1 + tvar2;
    *T = (tvar1 + tvar2) / 5120.0; 

    pvar1 = (t_fine / 2.0) - 64000.0;
    pvar2 = pvar1 * pvar1 * ((double)dig_P6) / 32768.0;
    pvar2 = pvar2 + pvar1 * ((double)dig_P5) * 2.0;
    pvar2 = (pvar2 / 4.0) + (((double)dig_P4) * 65536.0);
    pvar1 = (((double)dig_P3) * pvar1 * pvar1 / 524288.0 + ((double)dig_P2) * pvar1) / 524288.0;
    pvar1 = (1.0 + pvar1 / 32768.0) * ((double)dig_P1);

    if (pvar1 == 0.0)
    {
        *P = 0.0;
        ESP_LOGE(TAG, "Dividing by 0!");
        return;
    }

    p = 1048576.0 - (double)adc_P;
    p = (p - (pvar2 / 4096.0)) * 6250.0 / pvar1;
    pvar1 = ((double)dig_P9) * p * p / 2147483648.0;
    pvar2 = p * ((double)dig_P8) / 32768.0;
    p = p + (pvar1 + pvar2 + ((double)dig_P7)) / 16.0;

    *P = p;
}


/**
 * @brief initialize the BMP280
 * 
 * @param bmp struct with BMP280 parameters
 * @param res sensor resolution
*/
void bmp280_init(bmp280_conf_t bmp, enum bmp280_res res)
{
#ifdef BMP280_I2C_INIT
    i2c_master_bus_config_t i2c_mst_config = {
        .clk_source = I2C_CLK_SRC_DEFAULT,
        .i2c_port = bmp.i2c_port,
        .scl_io_num = bmp.scl_pin,
        .sda_io_num = bmp.sda_pin,
        .glitch_ignore_cnt = 7,
        .flags.enable_internal_pullup = true
    };
    ESP_ERROR_CHECK(i2c_new_master_bus(&i2c_mst_config, &bus_handle));
#endif

    i2c_device_config_t dev_cfg = {
        .dev_addr_length = I2C_ADDR_BIT_LEN_7,
        .device_address = bmp.i2c_addr,
        .scl_speed_hz = bmp.i2c_freq
    };
    ESP_ERROR_CHECK(i2c_master_bus_add_device(bus_handle, &dev_cfg, &dev_handle));

    // reset
    bmp280_reset(bmp);
    vTaskDelay(10 / portTICK_PERIOD_MS);

    // set mode
    uint8_t ctrl_meas = 0;
    uint8_t config = 0;
    switch (res)
    {
        case BMP280_ULTRA_LOW_POWER:
            ctrl_meas = 0b00100101; // osrs_t = x1 (0b001), osrs_p = x1 (0b001), mode = forced (0b01)
            config = 0b00000000; // t_sb = 0.5 ms (0b000), filter = off (0b000), spi3w_en = 0 (0b0)
            break;
        case BMP280_LOW_POWER:
            ctrl_meas = 0b00101011; // osrs_t = x1 (0b001), osrs_p = x2 (0b010), mode = normal (0b11)
            config = 0b00000000; // t_sb = 0.5 ms (0b000), filter = off (0b000), spi3w_en = 0 (0b0)
            break;
        case BMP280_STANDARD:
            ctrl_meas = 0b00101111; // osrs_t = x1 (0b001), osrs_p = x4 (0b011), mode = normal (0b11)
            config = 0b00001000; // t_sb = 0.5 ms (0b000), filter = x4 (0b010), spi3w_en = 0 (0b0)
            break;
        case BMP280_STANDARD_PLUS:
            ctrl_meas = 0b00101111; // osrs_t = x1 (0b001), osrs_p = x4 (0b011), mode = normal (0b11)
            config = 0b00010000; // t_sb = 0.5 ms (0b000), filter = x16 (0b100), spi3w_en = 0 (0b0)
            break;
        case BMP280_HIGH_RES:
            ctrl_meas = 0b01010111; // osrs_t = x2 (0b010), osrs_p = x16 (0b101), mode = normal (0b11)
            config = 0b00001000; // t_sb = 0.5 ms (0b000), filter = x4 (0b010), spi3w_en = 0 (0b0)
            break;
        case BMP280_ULTRA_HIGH_RES:
            ctrl_meas = 0b01010111; // osrs_t = x2 (0b010), osrs_p = x16 (0b101), mode = normal (0b11)
            config = 0b00010000; // t_sb = 0.5 ms (0b000), filter = x16 (0b100), spi3w_en = 0 (0b0)
            break;
        default:
            ESP_LOGE(TAG, "Invalid resolution!");
            return;
    }

    bmp280_write_ctrl_meas(bmp, &ctrl_meas);
    bmp280_write_config(bmp, &config);

    vTaskDelay(100 / portTICK_PERIOD_MS);

    // calibrate sea level pressure
    double temp_degC = 0.0;
    double press_Pa[10];

    for (uint32_t i = 0; i < 10; i++)
    {
        bmp280_read_temp_and_press(bmp, &temp_degC, &press_Pa[i]);
        vTaskDelay(40 / portTICK_PERIOD_MS);
    }

    pressure_sea_level = 0.0;
    for (uint32_t i = 0; i < 10; i++)
        pressure_sea_level += press_Pa[i];
    pressure_sea_level /= 10.0;
}


/**
 * @brief deinitialize the BMP280
 * 
 * @param bmp struct with BMP280 parameters
*/
void bmp280_deinit(bmp280_conf_t bmp)
{
    ESP_ERROR_CHECK(i2c_master_bus_rm_device(dev_handle));

#ifdef BMP280_I2C_INIT
    ESP_ERROR_CHECK(i2c_del_master_bus(bus_handle));
#endif
}


/**
 * @brief read BMP280 id register
 * 
 * @param bmp struct with BMP280 parameters
 * @param id pointer to id data
*/
void bmp280_read_id(bmp280_conf_t bmp, uint8_t* id)
{
    uint8_t reg = BMP280_ID;
    ESP_ERROR_CHECK(i2c_master_transmit_receive(dev_handle, &reg, 1, id, 1, BMP280_TIMEOUT_MS));
}


/**
 * @brief reset the BMP280
 * 
 * @param bmp struct with BMP280 parameters
*/
void bmp280_reset(bmp280_conf_t bmp)
{
    uint8_t data[2] = {BMP280_RESET_0, 0xB6};
    ESP_ERROR_CHECK(i2c_master_transmit(dev_handle, data, 2, BMP280_TIMEOUT_MS));
}


/**
 * @brief read BMP280 status register
 * 
 * @param bmp struct with BMP280 parameters
 * @param status pointer to status data
*/
void bmp280_read_status(bmp280_conf_t bmp, uint8_t* status)
{
    uint8_t reg = BMP280_STATUS;
    ESP_ERROR_CHECK(i2c_master_transmit_receive(dev_handle, &reg, 1, status, 1, BMP280_TIMEOUT_MS));

    if (*status & 0b00001000)
        ESP_LOGI(TAG, "BMP280 is measuring!");
    else if (*status & 0b00000001)
        ESP_LOGI(TAG, "BMP280 is updating!");
}


/**
 * @brief read BMP280 ctrl_meas register
 * 
 * @param bmp struct with BMP280 parameters
 * @param ctrl_meas pointer to ctrl_meas data
*/
void bmp280_read_ctrl_meas(bmp280_conf_t bmp, uint8_t* ctrl_meas)
{
    uint8_t reg = BMP280_CTRL_MEAS;
    ESP_ERROR_CHECK(i2c_master_transmit_receive(dev_handle, &reg, 1, ctrl_meas, 1, BMP280_TIMEOUT_MS));

    // osrs_t
    if ((*ctrl_meas & 0b11100000) == 0b00000000)
        ESP_LOGI(TAG, "osrs_t skipped (output set to 0x80000)");
    else if ((*ctrl_meas & 0b11100000) == 0b00100000)
        ESP_LOGI(TAG, "osrs_t oversampling x1");
    else if ((*ctrl_meas & 0b11100000) == 0b01000000)
        ESP_LOGI(TAG, "osrs_t oversampling x2");
    else if ((*ctrl_meas & 0b11100000) == 0b01100000)
        ESP_LOGI(TAG, "osrs_t oversampling x4");
    else if ((*ctrl_meas & 0b11100000) == 0b10000000)
        ESP_LOGI(TAG, "osrs_t oversampling x8");
    else
        ESP_LOGI(TAG, "osrs_t oversampling x16");

    // osrs_p
    if ((*ctrl_meas & 0b00011100) == 0b00000000)
        ESP_LOGI(TAG, "osrs_p skipped (output set to 0x80000)");
    else if ((*ctrl_meas & 0b00011100) == 0b00000100)
        ESP_LOGI(TAG, "osrs_p oversampling x1");
    else if ((*ctrl_meas & 0b00011100) == 0b00001000)
        ESP_LOGI(TAG, "osrs_p oversampling x2");
    else if ((*ctrl_meas & 0b00011100) == 0b00001100)
        ESP_LOGI(TAG, "osrs_p oversampling x4");
    else if ((*ctrl_meas & 0b00011100) == 0b00010000)
        ESP_LOGI(TAG, "osrs_p oversampling x8");
    else
        ESP_LOGI(TAG, "osrs_p oversampling x16");

    // mode
    if ((*ctrl_meas & 0b00000011) == 0b00000000)
        ESP_LOGI(TAG, "sleep mode");
    else if ((*ctrl_meas & 0b00000011) == 0b00000011)
        ESP_LOGI(TAG, "normal mode");
    else
        ESP_LOGI(TAG, "forced mode");
}


/**
 * @brief write to BMP280 ctrl_meas register
 * 
 * @param bmp struct with BMP280 parameters
 * @param ctrl_meas pointer to ctrl_meas data
*/
void bmp280_write_ctrl_meas(bmp280_conf_t bmp, uint8_t* ctrl_meas)
{
    uint8_t data[2] = {BMP280_CTRL_MEAS, *ctrl_meas};
    ESP_ERROR_CHECK(i2c_master_transmit(dev_handle, data, 2, BMP280_TIMEOUT_MS));
}


/**
 * @brief read BMP280 config register
 * 
 * @param bmp struct with BMP280 parameters
 * @param config pointer to config data
*/
void bmp280_read_config(bmp280_conf_t bmp, uint8_t* config)
{
    uint8_t reg = BMP280_CONFIG;
    ESP_ERROR_CHECK(i2c_master_transmit_receive(dev_handle, &reg, 1, config, 1, BMP280_TIMEOUT_MS));

    // t_sb
    if ((*config & 0b11100000) == 0b00000000)
        ESP_LOGI(TAG, "t_stanby = 0.5 ms");
    else if ((*config & 0b11100000) == 0b00100000)
        ESP_LOGI(TAG, "t_stanby = 62.5 ms");
    else if ((*config & 0b11100000) == 0b01000000)
        ESP_LOGI(TAG, "t_stanby = 125 ms");
    else if ((*config & 0b11100000) == 0b01100000)
        ESP_LOGI(TAG, "t_stanby = 250 ms");
    else if ((*config & 0b11100000) == 0b10000000)
        ESP_LOGI(TAG, "t_stanby = 500 ms");
    else if ((*config & 0b11100000) == 0b10100000)
        ESP_LOGI(TAG, "t_stanby = 1000 ms");
    else if ((*config & 0b11100000) == 0b11000000)
        ESP_LOGI(TAG, "t_stanby = 2000 ms");
    else if ((*config & 0b11100000) == 0b11100000)
        ESP_LOGI(TAG, "t_stanby = 4000 ms");

    // filter
    if ((*config & 0b00011100) == 0b00000000)
        ESP_LOGI(TAG, "filter = off");
    else if ((*config & 0b00011100) == 0b00000100)
        ESP_LOGI(TAG, "filter = 2");
    else if ((*config & 0b00011100) == 0b00001000)
        ESP_LOGI(TAG, "filter = 4");
    else if ((*config & 0b00011100) == 0b00001100)
        ESP_LOGI(TAG, "filter = 8");
    else
        ESP_LOGI(TAG, "filter = 16");

    // spi3w_en
    if (*config & 0b00000001)
        ESP_LOGI(TAG, "3-wire SPI interface enabled");
    else
        ESP_LOGI(TAG, "3-wire SPI interface disabled");
}


/**
 * @brief write to BMP280 config register
 * 
 * @param bmp struct with BMP280 parameters
 * @param config pointer to config data
*/
void bmp280_write_config(bmp280_conf_t bmp, uint8_t* config)
{
    uint8_t old_config = 0;
    uint8_t reg = BMP280_CONFIG;
    ESP_ERROR_CHECK(i2c_master_transmit_receive(dev_handle, &reg, 1, &old_config, 1, BMP280_TIMEOUT_MS));

    vTaskDelay(1);

    uint8_t bit1 = (old_config & 0b00000010) >> 1;

    if (bit1 == 1)
        *config |= 0b00000010; // set bit 1 to 1
    else
        *config &= ~(0b00000010); // set bit 1 to 0

    uint8_t data[2] = {BMP280_CONFIG, *config};
    ESP_ERROR_CHECK(i2c_master_transmit(dev_handle, data, 2, BMP280_TIMEOUT_MS));
}


/**
 * @brief read temperature and pressure from BMP280
 * 
 * @param bmp struct with BMP280 parameters
 * @param temp_degC pointer to temperature in degrees Celsius
 * @param press_Pa pointer to pressure in Pascals
*/
void bmp280_read_temp_and_press(bmp280_conf_t bmp, double* temp_degC, double* press_Pa)
{
    uint8_t data[6] = {0, 0, 0, 0, 0, 0};
    uint8_t reg = BMP280_PRESS_MSB;
    ESP_ERROR_CHECK(i2c_master_transmit_receive(dev_handle, &reg, 1, data, 6, BMP280_TIMEOUT_MS));

    int32_t temp = (int32_t)(((uint32_t)data[3] << 12) | ((uint32_t)data[4] << 4) | ((uint32_t)data[5] >> 4));
    int32_t press = (int32_t)(((uint32_t)data[0] << 12) | ((uint32_t)data[1] << 4) | ((uint32_t)data[2] >> 4));

    // compensate measurements
    bmp280_compensate_T_P_double(bmp, temp, temp_degC, press, press_Pa);
}


/**
 * @brief read height from BMP280
 * 
 * @param bmp struct with BMP280 parameters
 * @param height_m pointer to height in meters
*/
void bmp280_read_height(bmp280_conf_t bmp, float* height_m)
{
    double temp_degC = 0.0;
    double press_Pa = 0.0;

    bmp280_read_temp_and_press(bmp, &temp_degC, &press_Pa);

    *height_m = 44330.0f * (1.0f - (float)pow(press_Pa / pressure_sea_level, 1.0 / 5.255));
}


/**
 * @brief set sea level pressure for height calculation
 * 
 * @param bmp struct with BMP280 parameters
*/
void bmp280_set_sea_level_pressure(bmp280_conf_t bmp)
{
    double temp_degC = 0.0;
    double press_Pa = 0.0;

    bmp280_read_temp_and_press(bmp, &temp_degC, &press_Pa);

    portENTER_CRITICAL(&spinlock);
    pressure_sea_level = press_Pa;
    portEXIT_CRITICAL(&spinlock);
}