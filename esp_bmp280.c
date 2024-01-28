/**
 * @file esp_bmp280.c
 * @author JanG175
 * @brief ESP-IDF component for the BMP280 temperature and pressure sensor (I2C only)
 * 
 * @copyright Apache 2.0
*/

#include <stdio.h>
#include "esp_bmp280.h"

static const char *TAG = "BMP280";


/**
 * @brief send data to BMP280 register
 * 
 * @param bmp struct with BMP280 parameters
 * @param reg register to write to
 * @param data pointer data to write
 * @param data_len length of data to write
*/
static void i2c_send(bmp280_conf_t bmp, uint8_t reg, uint8_t* data, uint32_t data_len)
{
    i2c_cmd_handle_t cmd = i2c_cmd_link_create();
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, (bmp.i2c_addr << 1) | I2C_MASTER_WRITE, I2C_MASTER_ACK);
    i2c_master_write_byte(cmd, reg, I2C_MASTER_ACK);
    i2c_master_write(cmd, data, data_len, I2C_MASTER_ACK);
    i2c_master_stop(cmd);
    i2c_master_cmd_begin(bmp.i2c_port, cmd, BMP280_TIMEOUT);
    i2c_cmd_link_delete(cmd);
}


/**
 * @brief read data from BMP280 register
 * 
 * @param bmp struct with BMP280 parameters
 * @param reg register to read from
 * @param data pointer to data to read
 * @param data_len length of data to read
*/
static void i2c_read(bmp280_conf_t bmp, uint8_t reg, uint8_t* data, uint32_t data_len)
{
    i2c_cmd_handle_t cmd = i2c_cmd_link_create();

    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, (bmp.i2c_addr << 1) | I2C_MASTER_WRITE, I2C_MASTER_ACK);
    i2c_master_write_byte(cmd, reg, I2C_MASTER_ACK);
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, (bmp.i2c_port << 1) | I2C_MASTER_READ, I2C_MASTER_ACK);

    for (uint32_t i = 0; i < data_len - 1; i++)
        i2c_master_read_byte(cmd, &data[i], I2C_MASTER_ACK);
    i2c_master_read_byte(cmd, &data[data_len - 1], I2C_MASTER_NACK);

    i2c_master_stop(cmd);
    i2c_master_cmd_begin(bmp.i2c_port, cmd, BMP280_TIMEOUT);
    i2c_cmd_link_delete(cmd);
}


/**
 * @brief compensate temperature and pressure readings
 * 
 * @param bmp struct with BMP280 parameters
 * @param adc_T temperature reading
 * @param T pointer to compensated temperature
 * @param adc_P pressure reading
 * @param P pointer to compensated pressure
*/
static void bmp280_compensate_T_P(bmp280_conf_t bmp, int32_t adc_T, int32_t* T, int32_t adc_P, uint32_t* P)
{
    uint8_t dig_calib[24];
    i2c_read(bmp, BMP280_CALIB_00, dig_calib, 24);

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

    int32_t tvar1, tvar2, t_fine;
    int64_t pvar1, pvar2, p;

    // temperature
    tvar1 = ((((adc_T >> 3) - ((int32_t)dig_T1 << 1))) * ((int32_t)dig_T2)) >> 11;
    tvar2 = (((((adc_T >> 4) - ((int32_t)dig_T1)) * ((adc_T >> 4) - ((int32_t)dig_T1))) >> 12) * ((int32_t)dig_T3)) >> 14;
    t_fine = tvar1 + tvar2;

    *T = (t_fine * 5 + 128) >> 8;

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

    *P = (uint32_t)p;
}


/**
 * @brief initialize the BMP280
 * 
 * @param bmp struct with BMP280 parameters
*/
void bmp280_init(bmp280_conf_t bmp)
{
    if (bmp.i2c_freq > BMP280_MAX_FREQ)
    {
        bmp.i2c_freq = BMP280_MAX_FREQ;
        ESP_LOGW(TAG, "I2C frequency too high, set to max value (%d Hz)", BMP280_MAX_FREQ);
    }

    i2c_config_t i2c_config = {
        .mode = I2C_MODE_MASTER,
        .sda_io_num = bmp.sda_pin,
        .sda_pullup_en = GPIO_PULLUP_ENABLE,
        .scl_io_num = bmp.scl_pin,
        .scl_pullup_en = GPIO_PULLUP_ENABLE,
        .master.clk_speed = bmp.i2c_freq,
        .clk_flags = 0
    };

    ESP_ERROR_CHECK(i2c_param_config(bmp.i2c_port, &i2c_config));
    ESP_ERROR_CHECK(i2c_driver_install(bmp.i2c_port, i2c_config.mode, 0, 0, 0));
}


/**
 * @brief deinitialize the BMP280
 * 
 * @param bmp struct with BMP280 parameters
*/
void bmp280_deinit(bmp280_conf_t bmp)
{
    ESP_ERROR_CHECK(i2c_driver_delete(bmp.i2c_port));
}


/**
 * @brief read BMP280 id register
 * 
 * @param bmp struct with BMP280 parameters
 * @param id pointer to id data
*/
void bmp280_read_id(bmp280_conf_t bmp, uint8_t* id)
{
    i2c_read(bmp, BMP280_ID, id, 1);
}


/**
 * @brief reset the BMP280
 * 
 * @param bmp struct with BMP280 parameters
*/
void bmp280_reset(bmp280_conf_t bmp)
{
    uint8_t data = 0xB6;
    i2c_send(bmp, BMP280_RESET, &data, 1);
}


/**
 * @brief read BMP280 status register
 * 
 * @param bmp struct with BMP280 parameters
 * @param status pointer to status data
*/
void bmp280_read_status(bmp280_conf_t bmp, uint8_t* status)
{
    i2c_read(bmp, BMP280_STATUS, status, 1);

    if (*status & 0b00001000)
        ESP_LOGI(TAG, "BMP280 is measuring!");
    if (*status & 0b00000001)
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
    i2c_read(bmp, BMP280_CTRL_MEAS, ctrl_meas, 1);

    // osrs_t
    if ((*ctrl_meas & 0b11100000) == 0b00000000)
        ESP_LOGI(TAG, "skipped (output set to 0x80000)");
    else if ((*ctrl_meas & 0b11100000) == 0b00100000)
        ESP_LOGI(TAG, "oversampling x1");
    else if ((*ctrl_meas & 0b11100000) == 0b01000000)
        ESP_LOGI(TAG, "oversampling x2");
    else if ((*ctrl_meas & 0b11100000) == 0b01100000)
        ESP_LOGI(TAG, "oversampling x4");
    else if ((*ctrl_meas & 0b11100000) == 0b10000000)
        ESP_LOGI(TAG, "oversampling x8");
    else
        ESP_LOGI(TAG, "oversampling x16");

    // osrs_p
    if ((*ctrl_meas & 0b00011100) == 0b00000000)
        ESP_LOGI(TAG, "skipped (output set to 0x80000)");
    else if ((*ctrl_meas & 0b00011100) == 0b00000100)
        ESP_LOGI(TAG, "oversampling x1");
    else if ((*ctrl_meas & 0b00011100) == 0b00001000)
        ESP_LOGI(TAG, "oversampling x2");
    else if ((*ctrl_meas & 0b00011100) == 0b00001100)
        ESP_LOGI(TAG, "oversampling x4");
    else if ((*ctrl_meas & 0b00011100) == 0b00010000)
        ESP_LOGI(TAG, "oversampling x8");
    else
        ESP_LOGI(TAG, "oversampling x16");

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
    i2c_send(bmp, BMP280_CTRL_MEAS, ctrl_meas, 1);
}


/**
 * @brief read BMP280 config register
 * 
 * @param bmp struct with BMP280 parameters
 * @param config pointer to config data
*/
void bmp280_read_config(bmp280_conf_t bmp, uint8_t* config)
{
    i2c_read(bmp, BMP280_CONFIG, config, 1);

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
    i2c_read(bmp, BMP280_CONFIG, &old_config, 1);

    uint8_t bit1 = (old_config & 0b00000010) >> 1;

    if (bit1)
        *config |= 0b00000010; // set bit 1 to 1
    else
        *config &= ~(0b00000010); // set bit 1 to 0

    i2c_send(bmp, BMP280_CONFIG, config, 1);
}


/**
 * @brief read temperature and pressure from BMP280
 * 
 * @param bmp struct with BMP280 parameters
 * @param temp_degC pointer to temperature in degrees Celsius
 * @param press_Pa pointer to pressure in Pascals
*/
void bmp280_read_temp_and_press(bmp280_conf_t bmp, float* temp_degC, float* press_Pa)
{
    uint8_t data[6] = {0, 0, 0, 0, 0, 0};

    i2c_read(bmp, BMP280_TEMP_MSB, data, 6);

    int32_t temp = (int32_t)((uint32_t)data[0] << 12) | ((uint32_t)data[1] << 4) | ((uint32_t)data[2] >> 4);
    int32_t press = (int32_t)((uint32_t)data[3] << 12) | ((uint32_t)data[4] << 4) | ((uint32_t)data[5] >> 4);

    int32_t comp_temp = 0;
    uint32_t comp_press = 0;
    bmp280_compensate_T_P(bmp, temp, &comp_temp, press, &comp_press);

    *temp_degC = (float)comp_temp / 100.0f;
    *press_Pa = (float)comp_press / 256.0f;
}


