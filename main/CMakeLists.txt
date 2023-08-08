idf_component_register(SRCS "main.c" "iotex_i2c.c" "base64.c"
                    INCLUDE_DIRS "."
                    REQUIRES nvs_flash se050)
