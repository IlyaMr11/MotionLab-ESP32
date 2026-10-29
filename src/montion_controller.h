#pragma once 

#include "driver/gpio.h"
#include "esp_err.h"

typedef enum
{
    MOTOR_RUNNING,
    MOTOR_ON_STOP,
    MOTOR_LIMIT_HIT,
    MOTOR_BACKOF,
    MOTOR_ERROR,
    MOTOR_HOME,
    MOTOR_HOMMING,
} motor_state_t;

typedef struct {
    motor_state_t state;
    uint32_t frequency_hz;
    bool limit_switch_active;
} motion_status_t;

void motion_controller_init(
    gpio_num_t dir_pin, 
    gpio_num_t step_pin,
    gpio_num_t en_pin,
    gpio_num_t limit_pin
    );
esp_err_t motion_set_freq(uint32_t target_freq);
esp_err_t motion_move(int32_t steps_cnt);
esp_err_t motion_home_position(void);
esp_err_t motion_stop(void);
esp_err_t motion_get_status(motion_status_t *status);
motor_state_t motion_get_state(void);