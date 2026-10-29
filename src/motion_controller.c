#include <montion_controller.h>

#include <stdatomic.h>
#include "driver/gpio.h"
#include "driver/ledc.h"
#include "esp_err.h"
#include "driver/rmt_tx.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#define HIGH 1
#define LOW 0
#define SAFETY_RECOVER_DIRECTION 0
#define HOME_DIRECTION 1
#define HOME_BACKOF_FREQ 400
#define RTM_TICKS_PER_SEC 1000000
#define BACKOFF_BATCH_STEPS 10
#define MAXIMUM_BACKOFF_STPES 500

// events
#define MOTION_EVT_LIMIT (1UL << 0)
#define MOTION_EVT_TX_DONE (1UL << 2)

typedef struct
{
    gpio_num_t dir_pin;
    gpio_num_t step_pin;
    gpio_num_t en_pin;
    gpio_num_t limit_pin;

    uint32_t current_freq;
    uint32_t safety_backof_freq;
    uint8_t current_direction;
    motor_state_t _Atomic motor_state;
    rmt_symbol_word_t step_symbol;
    uint32_t backoff_steps_done;
} montion_controller_t;

typedef struct
{
    uint8_t direction;
    uint32_t duration_time;
    uint32_t steps_count;
} motion_plan_t;

static rmt_channel_handle_t step_channel;
static rmt_encoder_handle_t step_encoder;
static montion_controller_t controller;
static TaskHandle_t motion_task_handle = NULL;
static void motion_task(void *arg);

static esp_err_t calculate_motion(int32_t target_steps, motion_plan_t *result);
static esp_err_t change_direction(uint8_t dircetion);
static esp_err_t change_freq(uint32_t target_freq);
static esp_err_t create_and_run_rmt_plan(motion_plan_t *plan);
static esp_err_t setup_rmt(void);
static void setup_driver_pins(void);
static void setup_limit_logic(void);
static uint32_t calc_duration(void);
static esp_err_t go_home_postion(void);
static void update_step_symbol(uint32_t duration);
static esp_err_t backoff_motion(void);
static esp_err_t start_backoff_batch(void);
static bool confirm_limit_released(void);
static bool rmt_tx_done_callback(
    rmt_channel_handle_t channel,
    const rmt_tx_done_event_data_t *edata,
    void *user_ctx);

void motion_controller_init(
    gpio_num_t dir_pin,
    gpio_num_t step_pin,
    gpio_num_t en_pin,
    gpio_num_t limit_pin)
{
    controller.dir_pin = dir_pin;
    controller.step_pin = step_pin;
    controller.en_pin = en_pin;
    controller.limit_pin = limit_pin;
    controller.current_freq = 200;
    controller.current_direction = 1;
    controller.safety_backof_freq = 200;
    controller.backoff_steps_done = 0;
    atomic_store(&controller.motor_state, MOTOR_ON_STOP);

    setup_driver_pins();
    ESP_ERROR_CHECK(setup_rmt());

    BaseType_t result = xTaskCreate(
        motion_task,        // pxTaskCode
        "MotionTask",       // pcName
        4096,               // usStackDepth — байты в ESP-IDF
        NULL,               // pvParameters
        10,                 // uxPriority
        &motion_task_handle // pxCreatedTask
    );

    configASSERT(result == pdPASS);
    setup_limit_logic();
}

esp_err_t motion_set_freq(uint32_t target_freq)
{
    motor_state_t state = atomic_load(&controller.motor_state);
    if (state == MOTOR_ON_STOP || state == MOTOR_HOME)
    {
        return change_freq(target_freq);
    }
    else
    {
        return ESP_ERR_INVALID_STATE;
    }
}

esp_err_t motion_move(int32_t steps_cnt)
{
    motor_state_t state = atomic_load(&controller.motor_state);
    if (state == MOTOR_ON_STOP || state == MOTOR_HOME)
    {
        motion_plan_t motion_plan;
        esp_err_t err = calculate_motion(steps_cnt, &motion_plan);
        if (err != ESP_OK)
        {
            return err;
        }

        return create_and_run_rmt_plan(&motion_plan);
    }

    return ESP_ERR_INVALID_STATE;
}

esp_err_t motion_home_position(void)
{
    motor_state_t state = atomic_load(&controller.motor_state);
    if (state == MOTOR_ON_STOP)
    {
        return go_home_postion();
    }
    return ESP_ERR_INVALID_STATE;
}

motor_state_t motion_get_state(void)
{
    return atomic_load(&controller.motor_state);
}

esp_err_t motion_stop(void) {
    motor_state_t state = atomic_load(&controller.motor_state);
    if (state == MOTOR_LIMIT_HIT || state == MOTOR_BACKOF || state == MOTOR_HOME) {
        return ESP_ERR_INVALID_STATE;
    }
    if (state == MOTOR_ON_STOP) {
        return ESP_OK;
    }
    atomic_store(&controller.motor_state, MOTOR_ON_STOP);
    return rmt_disable(step_channel);
}

esp_err_t motion_get_status(motion_status_t *status)
{
    if (status == NULL) {
        return ESP_ERR_INVALID_ARG;
    }

    status->state = atomic_load(&controller.motor_state);
    status->frequency_hz = controller.current_freq;
    status->limit_switch_active =
        gpio_get_level(controller.limit_pin) == HIGH;

    return ESP_OK;
}
// PRIVATE HELPERS

static void motion_task(void *arg)
{
    while (1)
    {
        uint32_t events = 0;

        BaseType_t received = xTaskNotifyWait(
            0,
            UINT32_MAX,
            &events,
            portMAX_DELAY);

        if (received != pdTRUE)
        {
            continue;
        }

        if (events & MOTION_EVT_LIMIT)
        {
            motor_state_t state = atomic_load(&controller.motor_state);

            if (state != MOTOR_RUNNING && state != MOTOR_HOMMING)
            {
                continue;
            }

            ESP_ERROR_CHECK(rmt_disable(step_channel));

            if (state == MOTOR_RUNNING)
            {
                printf("LIMIT HIT during MOVE\n");
                atomic_store(&controller.motor_state, MOTOR_LIMIT_HIT);
            }
            else if (state == MOTOR_HOMMING)
            {
                ESP_ERROR_CHECK(rmt_enable(step_channel));
                ESP_ERROR_CHECK(backoff_motion());
            }
            continue;
        }

        if (events & MOTION_EVT_TX_DONE)
        {
            motor_state_t state = atomic_load(&controller.motor_state);

            if (state == MOTOR_BACKOF)
            {
                if (confirm_limit_released())
                {
                    controller.backoff_steps_done = 0;
                    atomic_store(&controller.motor_state, MOTOR_HOME);
                }
                else
                {
                    esp_err_t err = start_backoff_batch();

                    if (err == ESP_OK)
                    {
                        controller.backoff_steps_done += BACKOFF_BATCH_STEPS;
                    }
                    else
                    {
                        atomic_store(&controller.motor_state, MOTOR_ERROR);
                        printf("BACKOFF ERROR: %s\n", esp_err_to_name(err));
                    }
                }
                continue;
            }
            else if (state == MOTOR_RUNNING)
            {
                // На случай почти одновременного LIMIT и TX_DONE
                if (gpio_get_level(controller.limit_pin) == HIGH)
                {
                    atomic_store(&controller.motor_state, MOTOR_LIMIT_HIT);
                }
                else
                {
                    atomic_store(&controller.motor_state, MOTOR_ON_STOP);
                    printf("MOVE completed\n");
                }
            }
        }
    }
}

static esp_err_t calculate_motion(int32_t target_steps, motion_plan_t *result)
{
    if (target_steps > 200 || target_steps < -200 || target_steps == 0)
    {
        return ESP_ERR_INVALID_ARG;
    }
    uint8_t direction = target_steps > 0 ? 0 : 1;
    uint32_t duration_time = calc_duration();
    uint32_t steps_count = (uint32_t)(abs(target_steps) * 80);
    motion_plan_t plan = {
        .direction = direction,
        .duration_time = duration_time,
        .steps_count = steps_count,
    };
    *result = plan;
    return ESP_OK;
}

static esp_err_t change_direction(uint8_t direction)
{
    if (direction != 0 && direction != 1)
    {
        return ESP_ERR_INVALID_ARG;
    }
    controller.current_direction = direction;
    return gpio_set_level(controller.dir_pin, direction);
}

static esp_err_t change_freq(uint32_t target_freq)
{
    if (target_freq > 50 && target_freq < 4000)
    {
        controller.current_freq = target_freq;

        return ESP_OK;
    }
    return ESP_ERR_INVALID_ARG;
}

static esp_err_t go_home_postion(void)
{
    change_direction(HOME_DIRECTION);
    change_freq(HOME_BACKOF_FREQ);
    uint32_t duration = calc_duration();
    update_step_symbol(duration);

    rmt_transmit_config_t tx_config = {
        .loop_count = -1,
        .flags = {.eot_level = 0}};

    atomic_store(&controller.motor_state, MOTOR_HOMMING);

    return rmt_transmit(
        step_channel,
        step_encoder,
        &controller.step_symbol,
        sizeof(controller.step_symbol),
        &tx_config);
}

static void limit_reached(void *arg)
{
    motor_state_t state = atomic_load(&controller.motor_state);
    if (state == MOTOR_RUNNING || state == MOTOR_HOMMING)
    {
        BaseType_t woken = pdFALSE;

        xTaskNotifyFromISR(
            motion_task_handle,
            MOTION_EVT_LIMIT,
            eSetBits,
            &woken);

        if (woken == pdTRUE)
        {
            portYIELD_FROM_ISR();
        }
    }
}

static esp_err_t create_and_run_rmt_plan(motion_plan_t *plan)
{
    change_direction(plan->direction);
    rmt_symbol_word_t step_symbol = {
        .level0 = 1,
        .duration0 = plan->duration_time,
        .level1 = 0,
        .duration1 = plan->duration_time,
    };
    controller.step_symbol = step_symbol;

    rmt_transmit_config_t tx_config = {
        .loop_count = plan->steps_count,
        .flags = {.eot_level = 0}};

    atomic_store(&controller.motor_state, MOTOR_RUNNING);

    return rmt_transmit(
        step_channel,
        step_encoder,
        &controller.step_symbol,
        sizeof(controller.step_symbol),
        &tx_config);
}

static uint32_t calc_duration(void)
{
    return RTM_TICKS_PER_SEC / (2 * controller.current_freq);
}

static void update_step_symbol(uint32_t duration)
{
    rmt_symbol_word_t new_symbol = {
        .level0 = 1,
        .duration0 = duration,
        .level1 = 0,
        .duration1 = duration,
    };
    controller.step_symbol = new_symbol;
}

// SETUP BLOCK:

static esp_err_t setup_rmt(void)
{
    rmt_tx_channel_config_t config = {
        .gpio_num = controller.step_pin,
        .clk_src = RMT_CLK_SRC_DEFAULT,
        .resolution_hz = RTM_TICKS_PER_SEC,
        .mem_block_symbols = 48,
        .trans_queue_depth = 1,
    };

    ESP_ERROR_CHECK(rmt_new_tx_channel(&config, &step_channel));

    rmt_copy_encoder_config_t encoder_config = {};
    ESP_ERROR_CHECK(rmt_new_copy_encoder(&encoder_config, &step_encoder));

    rmt_tx_event_callbacks_t callbacks = {
        .on_trans_done = rmt_tx_done_callback,
    };

    ESP_ERROR_CHECK(
        rmt_tx_register_event_callbacks(
            step_channel,
            &callbacks,
            NULL));
    return rmt_enable(step_channel);
}

static void setup_limit_logic(void)
{
    gpio_config_t limit_logic_config = {
        .pin_bit_mask = 1ULL << controller.limit_pin,
        .mode = GPIO_MODE_INPUT,
        .pull_up_en = GPIO_PULLUP_ENABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type = GPIO_INTR_POSEDGE};

    ESP_ERROR_CHECK(gpio_config(&limit_logic_config));
    ESP_ERROR_CHECK(gpio_install_isr_service(0));
    ESP_ERROR_CHECK(gpio_isr_handler_add(controller.limit_pin, limit_reached, NULL));
}

static void setup_driver_pins(void)
{
    gpio_set_direction(controller.en_pin, GPIO_MODE_OUTPUT);
    gpio_set_level(controller.en_pin, LOW);
    gpio_set_direction(controller.dir_pin, GPIO_MODE_OUTPUT);
}

static esp_err_t backoff_motion(void)
{
    atomic_store(&controller.motor_state, MOTOR_BACKOF);
    esp_err_t err = change_direction(SAFETY_RECOVER_DIRECTION);
    if (err != ESP_OK)
    {
        return err;
    }

    controller.current_freq = HOME_BACKOF_FREQ;
    update_step_symbol(calc_duration());

    controller.backoff_steps_done = 0;
    err = start_backoff_batch();

    if (err == ESP_OK)
    {
        controller.backoff_steps_done = BACKOFF_BATCH_STEPS;
    }

    return err;
}

static bool rmt_tx_done_callback(
    rmt_channel_handle_t channel,
    const rmt_tx_done_event_data_t *edata,
    void *user_ctx)
{
    BaseType_t woken = pdFALSE;

    xTaskNotifyFromISR(
        motion_task_handle,
        MOTION_EVT_TX_DONE,
        eSetBits,
        &woken);

    return woken == pdTRUE;
}

static esp_err_t start_backoff_batch(void)
{
    if (controller.backoff_steps_done + BACKOFF_BATCH_STEPS >
        MAXIMUM_BACKOFF_STPES)
    {
        return ESP_ERR_INVALID_STATE;
    }

    rmt_transmit_config_t tx_config = {
        .loop_count = BACKOFF_BATCH_STEPS,
        .flags = {.eot_level = 0}};

    return rmt_transmit(
        step_channel,
        step_encoder,
        &controller.step_symbol,
        sizeof(controller.step_symbol),
        &tx_config);
}

static bool confirm_limit_released(void)
{
    for (int i = 0; i < 5; i++)
    {
        vTaskDelay(pdMS_TO_TICKS(5));

        if (gpio_get_level(controller.limit_pin) != LOW)
        {
            return false;
        }
    }
    return true;
}