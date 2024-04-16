# ESP-IDF component for BMP280 I2C pressure sensor

## Notes
* If `i2c_master_bus_handle_t bus_handle` is defined outside of this component, please comment out `// #define BMP280_I2C_INIT   1 // uncomment to initialize I2C driver` in `include/esp_bmp280.h`.

## Sources
* https://www.bosch-sensortec.com/media/boschsensortec/downloads/datasheets/bst-bmp280-ds001.pdf