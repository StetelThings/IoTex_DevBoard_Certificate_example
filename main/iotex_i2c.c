#include "esp_log.h"
#include "driver/i2c.h"
//#include "iotex_board.h"

#define I2C_MASTER_RX_BUF_DISABLE       0
#define I2C_MASTER_TX_BUF_DISABLE       0
#define I2C_MASTER_CLK_FREQ_HZ          100000
#define I2C_MASTER_PORT                 I2C_NUM_0

#define ACK_CHECK_EN                    0x1              /*!< I2C master will check ack from slave*/
#define ACK_CHECK_DIS                   0x0              /*!< I2C master will not check ack from slave */
#define ACK_VAL                         0x0              /*!< I2C ack value */
#define NACK_VAL                        0x1              /*!< I2C nack value */

// I2C ADDRESS
#define I2C_ADDR_ACC            0x0e
#define I2C_ADDR_PRES           0x76
#define I2C_ADDR_LED            0x30
#define I2C_ADDR_SE050          0x48
#define I2C_ADDR_NXP            0x61
#define I2C_ADDR_LIGHT          0x29

static const char *TAG = "PERIPH";

esp_err_t iotex_i2c_init() {
    i2c_config_t conf_master;
    esp_err_t ret;
    
    conf_master.mode = I2C_MODE_MASTER;
    conf_master.sda_io_num = 5;
    conf_master.sda_pullup_en = GPIO_PULLUP_DISABLE;
    conf_master.scl_io_num = 6;
    conf_master.scl_pullup_en = GPIO_PULLUP_DISABLE;
    conf_master.master.clk_speed = I2C_MASTER_CLK_FREQ_HZ;
    conf_master.clk_flags = I2C_SCLK_SRC_FLAG_FOR_NOMAL;
    ret = i2c_param_config(I2C_MASTER_PORT, &conf_master);
    if (ret != ESP_OK)
        return ret;
    ret = i2c_driver_install(I2C_MASTER_PORT, conf_master.mode, I2C_MASTER_RX_BUF_DISABLE, I2C_MASTER_TX_BUF_DISABLE, 0);
    return ret;
}

static esp_err_t iotex_i2c_write_to_slave(i2c_port_t i2c_port, uint8_t slave_addr, uint8_t* buf, size_t size, TickType_t timeout) {
    esp_err_t ret;
    
    if (size == 0) {
        return ESP_OK;
    }

    i2c_cmd_handle_t cmd = i2c_cmd_link_create();
    ret = i2c_master_start(cmd);
    if (ret != ESP_OK) {
        return ret;
    }
    ret = i2c_master_write_byte(cmd, (slave_addr << 1) | I2C_MASTER_WRITE, ACK_CHECK_EN);
    if (ret != ESP_OK) {
        return ret;
    }
    ret = i2c_master_write(cmd, buf, size, ACK_CHECK_EN);
    if (ret != ESP_OK) {
        return ret;
    }
    ret = i2c_master_stop(cmd);
    if (ret != ESP_OK) {
        return ret;
    }
    ret = i2c_master_cmd_begin(i2c_port, cmd, timeout);
    i2c_cmd_link_delete(cmd);
    return ret;
}

static esp_err_t iotex_i2c_read_from_slave(i2c_port_t i2c_port, uint8_t slave_addr, uint8_t* buf, size_t size, i2c_ack_type_t ackval, TickType_t timeout) {
    esp_err_t ret;

    if (size == 0) {
        return ESP_OK;
    }
    
    i2c_cmd_handle_t cmd = i2c_cmd_link_create();
    ret = i2c_master_start(cmd);
    if (ret != ESP_OK) {
        return ret;
    }
    ret = i2c_master_write_byte(cmd, (slave_addr << 1) | I2C_MASTER_READ, ACK_CHECK_EN);
    if (ret != ESP_OK) {
        return ret;
    }

    ret = i2c_master_read(cmd, buf, size, ackval);
    if (ret != ESP_OK) {
        return ret;
    }
    
    ret = i2c_master_stop(cmd);
    if (ret != ESP_OK) {
        return ret;
    }
    ret = i2c_master_cmd_begin(i2c_port, cmd, timeout);
    i2c_cmd_link_delete(cmd);
    return ret;
}

esp_err_t iotex_i2c_write(uint8_t slave_addr, uint8_t* buf, size_t size, TickType_t timeout) {
    return iotex_i2c_write_to_slave(I2C_MASTER_PORT, slave_addr, buf, size, timeout);
}

esp_err_t iotex_i2c_read(uint8_t slave_addr, uint8_t* buf, size_t size, TickType_t timeout) {
    return iotex_i2c_read_from_slave(I2C_MASTER_PORT, slave_addr, buf, size, NACK_VAL, timeout);
}

esp_err_t iotex_i2c_SE050_read(uint8_t slave_addr, uint8_t* buf, size_t size, TickType_t timeout) {
    return iotex_i2c_read_from_slave(I2C_MASTER_PORT, slave_addr, buf, size, ACK_VAL, timeout);
}

esp_err_t iotex_i2c_write_ready_poll(uint8_t slave_addr, TickType_t timeout) {
    esp_err_t ret;
    
    //int s_time, h_time;
    //i2c_get_start_timing(I2C_MASTER_PORT, &s_time, &h_time);
    //ESP_LOGI(TAG, "Start timing: %x, %x", s_time, h_time);
    i2c_cmd_handle_t cmd = i2c_cmd_link_create();
    ret = i2c_master_start(cmd);
    if (ret != ESP_OK) {
        return ret;
    }
    ret = i2c_master_write_byte(cmd, (slave_addr << 1) | I2C_MASTER_WRITE, ACK_CHECK_EN);
    if (ret != ESP_OK) {
        return ret;
    }
    
    ret = i2c_master_cmd_begin(I2C_MASTER_PORT, cmd, timeout);
    if (ret == ESP_FAIL) {
        ESP_LOGW(TAG, "ACK not received");
    } else if (ret == ESP_OK) {
        ESP_LOGI(TAG, "ACK received!");
    } else {
        ESP_LOGE(TAG, "ready poll ERROR");
    }
    i2c_cmd_link_delete(cmd);
    return ret;
}

esp_err_t iotex_PC9420_read_reg(uint8_t *regaddr, uint8_t* regval) {
    esp_err_t ret;
    
    ret=iotex_i2c_write_to_slave(I2C_MASTER_PORT, I2C_ADDR_NXP, regaddr, 1, 1000 / portTICK_PERIOD_MS);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "FAILED writing register address to PC9420: ERROR %d", ret);
        return ret;
    }
    ret=iotex_i2c_read(I2C_ADDR_NXP, regval, 1, 1000 / portTICK_PERIOD_MS);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "FAILED reading PC9420: ERROR %d", ret);
        return ret;
    }
    return ESP_OK;
}

esp_err_t iotex_PC9420_write_reg(uint8_t *regaddr, uint8_t* regval) {
    esp_err_t ret;    
    uint8_t buf[2];

    buf[0] = *regaddr;
    buf[1] = *regval;

    ret=iotex_i2c_write_to_slave(I2C_MASTER_PORT, I2C_ADDR_NXP, buf, 2, 1000 / portTICK_PERIOD_MS);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "FAILED writing register to PC9420: ERROR %d", ret);
        return ret;
    }
    return ESP_OK;
}

esp_err_t iotex_BMP388_read_reg(uint8_t *regaddr, uint8_t* regval) {
    esp_err_t ret;
    
    ret=iotex_i2c_write_to_slave(I2C_MASTER_PORT, I2C_ADDR_PRES, regaddr, 1, 1000 / portTICK_PERIOD_MS);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "FAILED writing register address to BMP388: ERROR %d", ret);
        return ret;
    }
    ret=iotex_i2c_read(I2C_ADDR_PRES, regval, 1, 1000 / portTICK_PERIOD_MS);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "FAILED reading BMP388: ERROR %d", ret);
        return ret;
    }
    return ESP_OK;
}

esp_err_t iotex_BMP388_write_reg(uint8_t *regaddr, uint8_t* regval) {
    esp_err_t ret;    
    uint8_t buf[2];

    buf[0] = *regaddr;
    buf[1] = *regval;

    ret=iotex_i2c_write_to_slave(I2C_MASTER_PORT, I2C_ADDR_PRES, buf, 2, 1000 / portTICK_PERIOD_MS);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "FAILED writing register to BMP388: ERROR %d", ret);
        return ret;
    }
    return ESP_OK;
}

esp_err_t iotex_KXTJ3_read_reg(uint8_t *regaddr, uint8_t* regval) {
    esp_err_t ret;
    
    ret=iotex_i2c_write_to_slave(I2C_MASTER_PORT, I2C_ADDR_ACC, regaddr, 1, 1000 / portTICK_PERIOD_MS);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "FAILED writing register address to KXTJ3: ERROR %d", ret);
        return ret;
    }
    ret=iotex_i2c_read(I2C_ADDR_ACC, regval, 1, 1000 / portTICK_PERIOD_MS);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "FAILED reading KXTJ3: ERROR %d", ret);
        return ret;
    }
    return ESP_OK;

    // uint8_t slave_addr = I2C_ADDR_ACC;
    // i2c_cmd_handle_t cmd = i2c_cmd_link_create();
    
    // ret = i2c_master_start(cmd);
    // if (ret != ESP_OK) {
    //     return ret;
    // }

    // //Address the accelerometer in write
    // ret = i2c_master_write_byte(cmd, (slave_addr << 1) | I2C_MASTER_READ, ACK_CHECK_EN);
    // if (ret != ESP_OK) {
    //     return ret;
    // }

    // //Write register address
    // ret = i2c_master_write(cmd, regaddr, 1, ACK_CHECK_EN);
    // if (ret != ESP_OK) {
    //     return ret;
    // }

    // ret = i2c_master_start(cmd);
    // if (ret != ESP_OK) {
    //     return ret;
    // }  


    // //Address the accelerometer in read
    // ret = i2c_master_write_byte(cmd, (slave_addr << 1) | I2C_MASTER_WRITE, ACK_CHECK_EN);
    // if (ret != ESP_OK) {
    //     return ret;
    // }

    // //Read register value
    // ret = i2c_master_read_byte(cmd, regval, NACK_VAL);
    // if (ret != ESP_OK) {
    //     return ret;
    // }

    // ret = i2c_master_stop(cmd);
    // if (ret != ESP_OK) {
    //     return ret;
    // }

    // ret = i2c_master_cmd_begin(I2C_MASTER_PORT, cmd, 1000 / portTICK_PERIOD_MS);
    // i2c_cmd_link_delete(cmd);
    // if (ret != ESP_OK) {
    //     return ret;
    // }

    // return ESP_OK;

}

esp_err_t iotex_KXTJ3_write_reg(uint8_t *regaddr, uint8_t* regval) {
    esp_err_t ret;    
    uint8_t buf[2];

    buf[0] = *regaddr;
    buf[1] = *regval;

    ret=iotex_i2c_write_to_slave(I2C_MASTER_PORT, I2C_ADDR_ACC, buf, 2, 1000 / portTICK_PERIOD_MS);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "FAILED writing register to KXTJ3: ERROR %d", ret);
        return ret;
    }
    return ESP_OK;
}


esp_err_t iotex_KTD2026_read_reg(uint8_t *regaddr, uint8_t* regval) {
    esp_err_t ret;
    
    ret=iotex_i2c_write_to_slave(I2C_MASTER_PORT, I2C_ADDR_LED, regaddr, 1, 1000 / portTICK_PERIOD_MS);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "FAILED writing register address to KTD2026: ERROR %d", ret);
        return ret;
    }
    ret=iotex_i2c_read(I2C_ADDR_LED, regval, 1, 1000 / portTICK_PERIOD_MS);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "FAILED reading KTD2026: ERROR %d", ret);
        return ret;
    }
    return ESP_OK;
}

esp_err_t iotex_KTD2026_write_reg(uint8_t *regaddr, uint8_t* regval) {
    esp_err_t ret;    
    uint8_t buf[2];

    buf[0] = *regaddr;
    buf[1] = *regval;

    ret=iotex_i2c_write_to_slave(I2C_MASTER_PORT, I2C_ADDR_LED, buf, 2, 1000 / portTICK_PERIOD_MS);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "FAILED writing register to KTD2026: ERROR %d", ret);
        return ret;
    }
    return ESP_OK;
}

esp_err_t iotex_BH1730_read_reg(uint8_t *regaddr, uint8_t* regval) {
    esp_err_t ret;
    
    ret=iotex_i2c_write_to_slave(I2C_MASTER_PORT, I2C_ADDR_LIGHT, regaddr, 1, 1000 / portTICK_PERIOD_MS);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "FAILED writing register address to BH1730: ERROR %d", ret);
        return ret;
    }
    ret=iotex_i2c_read(I2C_ADDR_LIGHT, regval, 1, 1000 / portTICK_PERIOD_MS);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "FAILED reading BH1730: ERROR %d", ret);
        return ret;
    }
    return ESP_OK;
}

esp_err_t iotex_BH1730_write_reg(uint8_t *regaddr, uint8_t* regval) {
    esp_err_t ret;    
    uint8_t buf[2];

    buf[0] = *regaddr;
    buf[1] = *regval;

    ret=iotex_i2c_write_to_slave(I2C_MASTER_PORT, I2C_ADDR_LIGHT, buf, 2, 1000 / portTICK_PERIOD_MS);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "FAILED writing register to BH1730: ERROR %d", ret);
        return ret;
    }
    return ESP_OK;
}

/*
esp_err_t iotex_se050_getData (uint8_t *respBuffer, size_t *respBufferLen) {

    uint8_t getDataCmd[SEMS_LITE_GET_DATA_CMD_BUF_LEN] = {
        0x80, // CLA '80' / '00' GlobalPlatform / ISO / IEC
        0xCA, // INS 'CA' GET DATA(IDENTIFY)
        0x00, // P1 '00' High order tag value
        0x00, // P2  - proprietary data coming from respective function
        0x00, // Lc is Le'00' Case 2 command
    };

}
*/

esp_err_t iotex_i2c_deinit(void) {
    i2c_driver_delete(I2C_MASTER_PORT);
    return ESP_OK;
}