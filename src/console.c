#include "console.h"

#include "esp_console.h"
#include "esp_err.h"
#include <errno.h>
#include <inttypes.h>
#include "montion_controller.h"

static esp_console_repl_t *s_repl = NULL;
static esp_console_dev_usb_serial_jtag_config_t usb_config = ESP_CONSOLE_DEV_USB_SERIAL_JTAG_CONFIG_DEFAULT();
static esp_console_repl_config_t repl_config = ESP_CONSOLE_REPL_CONFIG_DEFAULT();

static int command_status(int argc, char **argv);
static int command_home(int argc, char **argv);
static int command_freq(int argc, char **argv);
static int command_move(int argc, char **argv);
static int command_stop(int argc, char **argv);

static bool parse_int32(char* text, int32_t* result);
static const char *motion_state_name(motor_state_t state);
static esp_err_t register_commands(void);

esp_err_t console_start(void)
{
    repl_config.prompt = "motionlab> ";
    repl_config.max_history_len = 10;
    repl_config.max_cmdline_length = 32;

    esp_err_t err1 = esp_console_new_repl_usb_serial_jtag(
        &usb_config,
        &repl_config,
        &s_repl
    );

    esp_err_t err2 = register_commands();

    if (err1 != ESP_OK || err2 != ESP_OK) {
        return ESP_ERR_INVALID_RESPONSE;
    }

    return esp_console_start_repl(s_repl);
}

// DESCRIBES OF COMAND

static const esp_console_cmd_t status_command = {
    .command = "status",
    .help = "Print motor state",
    .hint = NULL,
    .func = command_status,
    .argtable = NULL,
};

static const esp_console_cmd_t home_command = {
    .command = "home",
    .help = "Start homing sequence",
    .hint = NULL,
    .func = command_home,
    .argtable = NULL,
};

static const esp_console_cmd_t move_command = {
    .command = "move",
    .help = "Move command - need one arg - number of steps of GT2 belt\n also support direction logic with negative numbers",
    .hint = "<steps>",
    .func = command_move,
    .argtable = NULL,
};

static const esp_console_cmd_t freq_command = {
    .command = "freq",
    .help = "Set step frequency",
    .hint = "<hz>",
    .func = command_freq,
    .argtable = NULL,
};

static const esp_console_cmd_t stop_command = {
    .command = "stop",
    .help = "Stop current motion",
    .hint = NULL,
    .func = command_stop,
    .argtable = NULL,
};


static esp_err_t register_commands(void)
{
    esp_err_t err;

    if ((err = esp_console_register_help_command()) != ESP_OK ||
        (err = esp_console_cmd_register(&status_command)) != ESP_OK ||
        (err = esp_console_cmd_register(&home_command)) != ESP_OK ||
        (err = esp_console_cmd_register(&freq_command)) != ESP_OK ||
        (err = esp_console_cmd_register(&move_command)) != ESP_OK ||
        (err = esp_console_cmd_register(&stop_command)) != ESP_OK) {
        return err;
    }

    return ESP_OK;
}

static int command_move(int argc, char **argv) {
    int32_t movement_steps;
    if (argc != 2 || !parse_int32(argv[1], &movement_steps)) {
        printf("usage: move");
        return 1;
    }

    esp_err_t err = motion_move(movement_steps);

    if (err != ESP_OK) {
        printf("status: failed: %s", esp_err_to_name(err));
        return 1;
    }

    printf("Move successfully start\n");
    return 0;
}

static int command_freq(int argc, char **argv)
{
    int32_t frequency;

    if (argc != 2) {
        printf("usage: freq <hz>\n");
        return 1;
    }

    if (!parse_int32(argv[1], &frequency) || frequency <= 0) {
        printf("freq: expected a positive integer\n");
        return 1;
    }

    esp_err_t err = motion_set_freq((uint32_t)frequency);

    if (err == ESP_ERR_INVALID_ARG) {
        printf("freq: invalid value, expected 51..3999 Hz\n");
        return 1;
    }

    if (err == ESP_ERR_INVALID_STATE) {
        printf("freq: rejected, motor must be stopped or home\n");
        return 1;
    }

    if (err != ESP_OK) {
        printf("freq: failed: %s\n", esp_err_to_name(err));
        return 1;
    }

    printf("freq: set to %" PRId32 " Hz\n", frequency);
    return 0;
}


static int command_home(int argc, char **argv)
{
    (void)argv;

    if (argc != 1) {
        printf("usage: home\n");
        return 1;
    }

    esp_err_t err = motion_home_position();

    if (err == ESP_ERR_INVALID_STATE) {
        printf("home: rejected, motor must be stopped\n");
        return 1;
    }

    if (err != ESP_OK) {
        printf("home: failed: %s\n", esp_err_to_name(err));
        return 1;
    }

    printf("home: accepted\n");
    return 0;
}

static int command_status(int argc, char **argv) {
    if (argc != 1) {
        printf("usage: status\n");
        return 1;
    }

    motion_status_t status;
    esp_err_t err = motion_get_status(&status);

    if (err != ESP_OK) {
        printf("status: failed: %s\n", esp_err_to_name(err));
        return 1;
    }

    printf("state: %s\n", motion_state_name(status.state));
    printf("frequency: %" PRIu32 "Hz\n", status.frequency_hz);
    printf("limit: %s\n", status.limit_switch_active ? "active" : "released");

    return 0;
}

static int command_stop(int argc, char**argv) {
    if (argc != 1) {
        printf("usage: stop");
        return 1;
    }

    esp_err_t err = motion_stop();

    if (err != ESP_OK) {
        printf("stop: failed: %s\n", esp_err_to_name(err));
        return 1;
    }

    printf("successfully stopeed!\n");
    return 0;
}

// helpers 

static bool parse_int32(char* text, int32_t* result) {
    char *end = NULL;
    errno = 0;

    long value = strtol(text, &end, 10);

    if (text == end || *end != '\0' || errno == ERANGE || value < INT32_MIN || value > INT32_MAX) {
        return false;
    }

    *result = (int32_t)value;
    return true;
}

static const char *motion_state_name(motor_state_t state)
{
    switch (state) {
    case MOTOR_RUNNING:
        return "running";

    case MOTOR_ON_STOP:
        return "stopped";

    case MOTOR_LIMIT_HIT:
        return "limit_hit";

    case MOTOR_BACKOF:
        return "backoff";

    case MOTOR_ERROR:
        return "error";

    case MOTOR_HOME:
        return "home";

    case MOTOR_HOMMING:
        return "homing";

    default:
        return "unknown";
    }
}