#include "as5600_encoder.h"
#include "driver/i2c_master.h"

typedef struct
{
    gpio_num_t sda_pin;
    gpio_num_t scl_pin;

    i2c_master_bus_handle_t bus_handle;
    i2c_master_dev_handle_t dev_handle;
} as5600_encoder_t;

static as5600_encoder_t encoder;

esp_err_t as5600_encoder_init(gpio_num_t sda_pin, gpio_num_t scl_pin)
{
    encoder.sda_pin = sda_pin;
    encoder.scl_pin = scl_pin;

    i2c_master_bus_config_t bus_config = {
        .i2c_port = I2C_NUM_0,
        .sda_io_num = encoder.sda_pin,
        .scl_io_num = encoder.scl_pin,
        .clk_source = I2C_CLK_SRC_DEFAULT,
        .glitch_ignore_cnt = 7,
        .flags.enable_internal_pullup = true};

    ESP_ERROR_CHECK(i2c_new_master_bus(&bus_config, &encoder.bus_handle));

    i2c_device_config_t dev_config = {
        .dev_addr_length = I2C_ADDR_BIT_LEN_7,
        .device_address = 0x36,
        .scl_speed_hz = 100000,
    };

    ESP_ERROR_CHECK(i2c_master_bus_add_device(
        encoder.bus_handle, &dev_config, &encoder.dev_handle));

    ESP_ERROR_CHECK(i2c_master_probe(
        encoder.bus_handle, 0x36, 100));

    return ESP_OK;
}

esp_err_t as5600_read_angle(uint16_t *angle)
{
    if (angle == NULL)
    {
        return ESP_ERR_INVALID_ARG;
    }
    if (encoder.dev_handle == NULL)
    {
        return ESP_ERR_INVALID_STATE;
    }

    uint8_t register_address = 0x0C;
    uint8_t data[2];

    esp_err_t err = i2c_master_transmit_receive(
        encoder.dev_handle,
        &register_address,
        1,
        data,
        2,
        100);

    if (err != ESP_OK)
    {
        return err;
    }

    uint16_t raw_angle =
        ((uint16_t)(data[0] & 0x0F) << 8) | data[1];

    *angle = raw_angle;

    return ESP_OK;
}

esp_err_t as5600_read_status(uint8_t *status)
{
    uint8_t reg = 0x0B;
    uint8_t status_reg = 0;

    esp_err_t err = i2c_master_transmit_receive(
        encoder.dev_handle,
        &reg,
        1,
        &status_reg,
        1,
        100
    );

    if (err != ESP_OK)
    {
        return err;
    }

    *status = status_reg;

    return ESP_OK;
};
