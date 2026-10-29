#include "as5600_encoder.h"
#include "montion_controller.h"
#include "console.h"

#include <stdio.h>
#include <stdbool.h>
#include <stdatomic.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/gpio.h"
#include "driver/ledc.h"
#include "driver/i2c_master.h"
#include "esp_err.h"

#define STEP_PIN GPIO_NUM_5
#define DIR_PIN GPIO_NUM_4
#define EN_PIN GPIO_NUM_6
#define LIMIT_PIN GPIO_NUM_18
#define WARN_DIOD_PIN GPIO_NUM_48

#define HIGH 1
#define LOW 0

void setup_encoder(void);
void get_agnle(i2c_master_dev_handle_t dev_handle);

void app_main(void)
{
    motion_controller_init(DIR_PIN, STEP_PIN, EN_PIN, LIMIT_PIN);
    ESP_ERROR_CHECK(console_start());
}