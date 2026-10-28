#include "as5600_encoder.h"
#include "montion_controller.h"

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
#define SDA_PIN GPIO_NUM_3
#define SCL_PIN GPIO_NUM_12

#define HIGH 1
#define LOW 0

void setup_encoder(void);
void get_agnle(i2c_master_dev_handle_t dev_handle);

void app_main(void)
{
    motion_controller_init(DIR_PIN, STEP_PIN, EN_PIN, LIMIT_PIN);

    esp_err_t err = motion_home_position();

    printf("HOME START: %s\n", esp_err_to_name(err));

    if (err != ESP_OK) {
        return;
    }

    // Ожидаем завершение HOME/BACKOFF.
    while (motion_get_state() != MOTOR_HOME)
    {
        motor_state_t state = motion_get_state();

        if (state == MOTOR_ERROR ||
            state == MOTOR_LIMIT_HIT)
        {
            printf("HOME FAILED: state=%d\n", state);
            return;
        }

        vTaskDelay(pdMS_TO_TICKS(50));
    }

    printf("HOME COMPLETED\n");

    // TEST 1: 20 STEP, 200 Hz
    ESP_ERROR_CHECK(motion_set_freq(200));
    ESP_ERROR_CHECK(motion_move_to(20));

    while (motion_get_state() == MOTOR_RUNNING) {
        vTaskDelay(pdMS_TO_TICKS(10));
    }

    printf("TEST 1: state=%d\n", motion_get_state());

    if (motion_get_state() != MOTOR_ON_STOP) {
        return;
    }

    vTaskDelay(pdMS_TO_TICKS(1000));

    // TEST 2: 20 STEP, 400 Hz
    ESP_ERROR_CHECK(motion_set_freq(3000));
    ESP_ERROR_CHECK(motion_move_to(30));

    while (motion_get_state() == MOTOR_RUNNING) {
        vTaskDelay(pdMS_TO_TICKS(10));
    }

    printf("TEST 2: state=%d\n", motion_get_state());

    ESP_ERROR_CHECK(motion_move_to(-55));

    while (motion_get_state() == MOTOR_RUNNING) {
        vTaskDelay(pdMS_TO_TICKS(10));
    }

    motor_state_t final_state = motion_get_state();

    if (final_state == MOTOR_LIMIT_HIT) {
        printf("TEST 3 PASSED: emergency limit triggered!\n");
    }
    else if (final_state == MOTOR_ON_STOP) {
        printf("TEST 3 FAILED: MOVE completed without LIMIT\n");
    }
    else {
        printf("TEST 3: unexpected state=%d\n", (int)final_state);
    }
}