#ifndef _IOTEX_I2C_H_
#define _IOTEX_I2C_H_

#include "freertos/semphr.h"
#include "driver/i2c.h"

esp_err_t iotex_i2c_init();
esp_err_t iotex_i2c_write(uint8_t slave_addr, uint8_t* buf, size_t size, TickType_t timeout);
esp_err_t iotex_i2c_read(uint8_t slave_addr, uint8_t* buf, size_t size, TickType_t timeout);
esp_err_t iotex_i2c_SE050_read(uint8_t slave_addr, uint8_t* buf, size_t size, TickType_t timeout);
esp_err_t iotex_i2c_write_ready_poll(uint8_t slave_addr, TickType_t timeout);
esp_err_t iotex_PC9420_read_reg(uint8_t *regaddr, uint8_t* regval);
esp_err_t iotex_PC9420_write_reg(uint8_t *regaddr, uint8_t* regval);
esp_err_t iotex_BMP388_read_reg(uint8_t *regaddr, uint8_t* regval);
esp_err_t iotex_BMP388_write_reg(uint8_t *regaddr, uint8_t* regval);
esp_err_t iotex_KXTJ3_read_reg(uint8_t *regaddr, uint8_t* regval);
esp_err_t iotex_KXTJ3_write_reg(uint8_t *regaddr, uint8_t* regval);
esp_err_t iotex_KTD2026_read_reg(uint8_t *regaddr, uint8_t* regval);
esp_err_t iotex_KTD2026_write_reg(uint8_t *regaddr, uint8_t* regval);
esp_err_t iotex_BH1730_read_reg(uint8_t *regaddr, uint8_t* regval);
esp_err_t iotex_BH1730_write_reg(uint8_t *regaddr, uint8_t* regval);
esp_err_t iotex_i2c_deinit(void);


#endif