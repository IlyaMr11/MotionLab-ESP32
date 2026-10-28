#pragma once

#include <stdint.h>
#include "esp_err.h"
#include "driver/gpio.h"

esp_err_t as5600_encoder_init(gpio_num_t sda_pin, gpio_num_t scl_pin);
esp_err_t as5600_read_angle(uint16_t *angle);
esp_err_t as5600_read_status(uint8_t *status);