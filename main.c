
#include <stdio.h>
#include <stdbool.h>
#include <stdint.h>
#include <string.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/gpio.h"
#include "driver/i2c_master.h"
#include "esp_timer.h"
#include "esp_rom_sys.h"
#include "esp_err.h"

/* ==================== PUMP CONFIGURATION ==================== */

#define PUMP_START_LEVEL          30.0f
#define PUMP_STOP_LEVEL           90.0f
#define PUMP_STARTUP_GRACE_MS     3000
#define PUMP_STALL_TIMEOUT_MS     10000
#define MIN_LEVEL_RISE_PERCENT    2.0f

static bool pump_running = false;
static bool pump_fault = false;
static bool sensor_fault = false;

static float pump_progress_reference_level = 0.0f;
static int64_t pump_start_time_us = 0;
static int64_t pump_last_progress_time_us = 0;

/* ==================== PIN CONFIGURATION ==================== */

#define TRIG_PIN          GPIO_NUM_5
#define ECHO_PIN          GPIO_NUM_18
#define GREEN_LED_PIN     GPIO_NUM_20
#define RED_LED_PIN       GPIO_NUM_19

#define OLED_SDA_PIN      GPIO_NUM_8
#define OLED_SCL_PIN      GPIO_NUM_9
#define OLED_ADDRESS      0x3C

#define BUZZER_PIN        GPIO_NUM_21
#define BUTTON_PIN        GPIO_NUM_10

/* ==================== SENSOR CONFIGURATION ==================== */

#define ECHO_TIMEOUT_US        30000
#define SAMPLE_INTERVAL_MS     1000

#define SOUND_SPEED_CM_US      0.0343f
#define MIN_DISTANCE_CM        2.0f
#define MAX_DISTANCE_CM        400.0f

#define TANK_EMPTY_DISTANCE_CM 50.0f
#define TANK_FULL_DISTANCE_CM  5.0f
#define HIGH_LEVEL_THRESHOLD   90.0f

/* ==================== OLED CONFIGURATION ==================== */

#define OLED_WIDTH  128
#define OLED_PAGES  8

static i2c_master_bus_handle_t oled_bus;
static i2c_master_dev_handle_t oled_device;

/* ==================== ALARM STATE ==================== */

/*
 * These variables are shared between app_main() and the
 * independent button task.
 *
 * volatile prevents the compiler from assuming that their
 * values never change outside the current execution flow.
 */
static volatile bool alarm_muted = false;
static volatile bool alarm_condition_active = false;

/* ==================== FUNCTION DECLARATIONS ==================== */

static void setup_gpio(void);
static bool wait_for_echo_level(int level, int timeout_us);
static float read_distance_cm(void);

static bool calculate_water_level(
    float distance_cm,
    float *water_level
);

static void update_status_leds(bool normal_status);
static void monitor_pump(bool reading_valid, float water_level);
static void button_task(void *pvParameters);

static void init_oled(void);
static void oled_command(uint8_t command);
static void oled_send_data(const uint8_t *data, size_t length);
static void oled_set_position(uint8_t page);
static void oled_clear(void);
static void oled_write_line(uint8_t page, const char *text);
static const uint8_t *oled_glyph(char c);

static void update_oled(
    float distance_cm,
    float water_level,
    bool reading_valid,
    bool alarm_condition,
    bool alarm_is_muted
);

/* ==================== PUMP MONITORING ==================== */

static void monitor_pump(bool reading_valid, float water_level)
{
    /*
     * Fail safe: stop simulated pumping when the water-level
     * measurement is invalid.
     */
    if (!reading_valid) {
        if (pump_running) {
            printf(
                "PUMP STOPPED: invalid sensor measurement.\n"
            );
        }

        pump_running = false;
        sensor_fault = true;

        pump_start_time_us = 0;
        pump_last_progress_time_us = 0;

        return;
    }

    /* A valid calibrated measurement clears the sensor fault. */
    if (sensor_fault) {
        printf("Sensor readings restored.\n");
    }

    sensor_fault = false;

    int64_t now_us = esp_timer_get_time();

    /*
     * Stop at the target level.
     * Reaching the target also clears a latched pump fault.
     */
    if (water_level >= PUMP_STOP_LEVEL) {
        if (pump_running) {
            printf(
                "PUMP STOPPED: target level reached (%.1f%%).\n",
                water_level
            );
        }

        if (pump_fault) {
            printf(
                "Pump fault cleared: target level reached.\n"
            );
        }

        pump_running = false;
        pump_fault = false;

        pump_start_time_us = 0;
        pump_last_progress_time_us = 0;
        pump_progress_reference_level = water_level;

        return;
    }

    /*
     * A pump fault remains latched below the target level.
     * Do not automatically restart the simulated pump.
     */
    if (pump_fault) {
        return;
    }

    /*
     * Start at 30% or below. The <= comparison is intentional:
     * a reading of exactly 30.0% must start the pump too.
     */
    if (!pump_running && water_level <= PUMP_START_LEVEL) {
        pump_running = true;

        pump_progress_reference_level = water_level;
        pump_start_time_us = now_us;

        /* Begin stall monitoring after the startup grace period. */
        pump_last_progress_time_us =
            pump_start_time_us +
            ((int64_t)PUMP_STARTUP_GRACE_MS * 1000);

        printf(
            "PUMP STARTED: water level %.1f%%.\n",
            water_level
        );

        return;
    }

    if (!pump_running) {
        return;
    }

    int64_t grace_end_us =
        pump_start_time_us +
        ((int64_t)PUMP_STARTUP_GRACE_MS * 1000);

    /* Allow time for pumping to begin. */
    if (now_us < grace_end_us) {
        return;
    }

    /*
     * A rise of at least two percentage points indicates
     * measurable progress.
     */
    if (water_level >=
        pump_progress_reference_level + MIN_LEVEL_RISE_PERCENT) {

        pump_progress_reference_level = water_level;
        pump_last_progress_time_us = now_us;

        printf(
            "PUMP PROGRESS: water level %.1f%%.\n",
            water_level
        );

        return;
    }

    /*
     * Stop the simulated pump and latch a fault if sufficient
     * water-level progress is not detected before the timeout.
     */
    if ((now_us - pump_last_progress_time_us) >=
        ((int64_t)PUMP_STALL_TIMEOUT_MS * 1000)) {

        pump_running = false;
        pump_fault = true;

        pump_start_time_us = 0;
        pump_last_progress_time_us = 0;

        printf(
            "PUMP FAULT: insufficient water-level rise. "
            "Pump stopped.\n"
        );
    }
}

/* ==================== ALARM MUTE BUTTON ==================== */

static void button_task(void *pvParameters)
{
    (void)pvParameters;

    bool button_was_pressed = false;

    while (true) {
        bool button_pressed =
            (gpio_get_level(BUTTON_PIN) == 0);

        /*
         * Detect a new press and debounce it.
         * The button is active-low and uses an internal pull-up.
         */
        if (button_pressed && !button_was_pressed) {
            vTaskDelay(pdMS_TO_TICKS(40));

            if (gpio_get_level(BUTTON_PIN) == 0) {
                button_was_pressed = true;

                if (alarm_condition_active) {
                    alarm_muted = true;

                    /* Silence the buzzer immediately. */
                    gpio_set_level(BUZZER_PIN, 0);

                    printf(
                        "BUTTON: Press detected.\n"
                        "ALARM: MUTED. Warning remains active.\n"
                    );
                } else {
                    printf(
                        "BUTTON: Press detected, "
                        "but no active alarm to mute.\n"
                    );
                }
            }
        }

        /* Wait for release before accepting another press. */
        if (gpio_get_level(BUTTON_PIN) == 1) {
            button_was_pressed = false;
        }

        vTaskDelay(pdMS_TO_TICKS(20));
    }
}

/* ==================== MAIN APPLICATION ==================== */

void app_main(void)
{
    setup_gpio();
    init_oled();

    bool previous_alarm_condition = false;

    printf("\n");
    printf("================================\n");
    printf("       EDGEGUARD PUMP\n");
    printf("================================\n");
    printf("Water-level monitoring started.\n");
    printf("Pump startup grace: %d ms\n", PUMP_STARTUP_GRACE_MS);
    printf("Pump stall timeout: %d ms\n", PUMP_STALL_TIMEOUT_MS);
    printf("Pump start threshold: %.1f%%\n", PUMP_START_LEVEL);
    printf("Pump stop threshold: %.1f%%\n", PUMP_STOP_LEVEL);
    printf("Invalid readings stop simulated pumping.\n");
    printf("Pump faults remain latched until target or reset.\n");
    printf("Alarm mute button: GPIO %d\n", BUTTON_PIN);
    printf("================================\n\n");

    /* Run button detection independently of sensor sampling. */
    BaseType_t task_result = xTaskCreate(
        button_task,
        "alarm_button",
        2048,
        NULL,
        5,
        NULL
    );

    if (task_result != pdPASS) {
        printf("ERROR: Failed to create alarm button task.\n");
    }

    while (true) {
        float distance_cm = read_distance_cm();
        float water_level = 0.0f;

        bool reading_valid = false;

        /* 1. Read and validate the sensor. */

        if (distance_cm < 0.0f) {
            printf(
                "SENSOR FAULT: no valid distance reading.\n"
            );
        } else if (!calculate_water_level(
                       distance_cm,
                       &water_level)) {

            printf(
                "SENSOR FAULT: distance %.2f cm is outside "
                "the calibrated tank range.\n",
                distance_cm
            );
        } else {
            reading_valid = true;

            printf(
                "Distance: %.2f cm | Water level: %.1f%%\n",
                distance_cm,
                water_level
            );
        }

        /* 2. Update the pump controller. */

        monitor_pump(reading_valid, water_level);

        /* 3. Determine the alarm condition. */

        bool alarm_condition =
            !reading_valid ||
            sensor_fault ||
            water_level >= HIGH_LEVEL_THRESHOLD ||
            pump_fault;

        /*
         * A newly appearing alarm re-enables the buzzer.
         * A cleared alarm resets the mute state.
         */
        if (alarm_condition && !previous_alarm_condition) {
            alarm_muted = false;
        }

        if (!alarm_condition) {
            alarm_muted = false;
        }

        /*
         * Publish the current alarm state for the button task.
         */
        alarm_condition_active = alarm_condition;

        /* 4. Update the status LEDs. */

        bool normal_status =
            reading_valid &&
            !sensor_fault &&
            water_level < HIGH_LEVEL_THRESHOLD &&
            !pump_fault;

        update_status_leds(normal_status);

        /* 5. Control the audible alarm. */

        bool buzzer_on =
            alarm_condition && !alarm_muted;

        gpio_set_level(
            BUZZER_PIN,
            buzzer_on ? 1 : 0
        );

        if (alarm_condition && !alarm_muted) {
            if (!reading_valid || sensor_fault) {
                printf("ALARM: Check water-level sensor.\n");
            } else if (pump_fault) {
                printf("ALARM: Pump progress fault.\n");
            } else if (water_level >= HIGH_LEVEL_THRESHOLD) {
                printf("ALARM: High water level detected.\n");
            }
        }

        /* 6. Refresh the OLED display. */

        update_oled(
            distance_cm,
            water_level,
            reading_valid,
            alarm_condition,
            alarm_muted
        );

        previous_alarm_condition = alarm_condition;

        /* 7. Wait before the next sensor sample. */

        vTaskDelay(pdMS_TO_TICKS(SAMPLE_INTERVAL_MS));
    }
}

/* ==================== GPIO SETUP ==================== */

static void setup_gpio(void)
{
    const gpio_config_t output_config = {
        .pin_bit_mask =
            (1ULL << TRIG_PIN) |
            (1ULL << GREEN_LED_PIN) |
            (1ULL << RED_LED_PIN) |
            (1ULL << BUZZER_PIN),

        .mode = GPIO_MODE_OUTPUT,
        .pull_up_en = GPIO_PULLUP_DISABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type = GPIO_INTR_DISABLE
    };

    const gpio_config_t input_config = {
        .pin_bit_mask = (1ULL << ECHO_PIN),
        .mode = GPIO_MODE_INPUT,
        .pull_up_en = GPIO_PULLUP_DISABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type = GPIO_INTR_DISABLE
    };

    const gpio_config_t button_config = {
        .pin_bit_mask = (1ULL << BUTTON_PIN),
        .mode = GPIO_MODE_INPUT,
        .pull_up_en = GPIO_PULLUP_ENABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type = GPIO_INTR_DISABLE
    };

    ESP_ERROR_CHECK(gpio_config(&output_config));
    ESP_ERROR_CHECK(gpio_config(&input_config));
    ESP_ERROR_CHECK(gpio_config(&button_config));

    gpio_set_level(TRIG_PIN, 0);
    gpio_set_level(GREEN_LED_PIN, 0);
    gpio_set_level(RED_LED_PIN, 0);
    gpio_set_level(BUZZER_PIN, 0);
}

/* ==================== ULTRASONIC SENSOR ==================== */

static bool wait_for_echo_level(int level, int timeout_us)
{
    int64_t start_time = esp_timer_get_time();

    while (gpio_get_level(ECHO_PIN) != level) {
        if ((esp_timer_get_time() - start_time) >= timeout_us) {
            return false;
        }
    }

    return true;
}

static float read_distance_cm(void)
{
    /* Generate the HC-SR04 trigger pulse. */

    gpio_set_level(TRIG_PIN, 0);
    esp_rom_delay_us(2);

    gpio_set_level(TRIG_PIN, 1);
    esp_rom_delay_us(10);
    gpio_set_level(TRIG_PIN, 0);

    /* Wait for the echo pulse to begin. */

    if (!wait_for_echo_level(1, ECHO_TIMEOUT_US)) {
        return -1.0f;
    }

    int64_t pulse_start = esp_timer_get_time();

    /* Wait for the echo pulse to end. */

    if (!wait_for_echo_level(0, ECHO_TIMEOUT_US)) {
        return -1.0f;
    }

    int64_t pulse_end = esp_timer_get_time();
    int64_t pulse_duration = pulse_end - pulse_start;

    /*
     * Distance = (time x speed of sound) / 2.
     * Division by two accounts for the outgoing and returning
     * ultrasonic sound wave.
     */
    float distance_cm =
        ((float)pulse_duration * SOUND_SPEED_CM_US) / 2.0f;

    if (distance_cm < MIN_DISTANCE_CM ||
        distance_cm > MAX_DISTANCE_CM) {
        return -1.0f;
    }

    return distance_cm;
}

/* ==================== WATER LEVEL CALCULATION ==================== */

static bool calculate_water_level(
    float distance_cm,
    float *water_level
)
{
    if (distance_cm < TANK_FULL_DISTANCE_CM ||
        distance_cm > TANK_EMPTY_DISTANCE_CM) {
        return false;
    }

    *water_level =
        ((TANK_EMPTY_DISTANCE_CM - distance_cm) /
         (TANK_EMPTY_DISTANCE_CM - TANK_FULL_DISTANCE_CM))
        * 100.0f;

    return true;
}

/* ==================== STATUS LEDS ==================== */

static void update_status_leds(bool normal_status)
{
    gpio_set_level(
        GREEN_LED_PIN,
        normal_status ? 1 : 0
    );

    gpio_set_level(
        RED_LED_PIN,
        normal_status ? 0 : 1
    );
}

/* ==================== OLED LOW-LEVEL DRIVER ==================== */

static void oled_command(uint8_t command)
{
    uint8_t packet[] = {0x00, command};

    ESP_ERROR_CHECK(
        i2c_master_transmit(
            oled_device,
            packet,
            sizeof(packet),
            1000
        )
    );
}

static void oled_send_data(
    const uint8_t *data,
    size_t length
)
{
    uint8_t packet[OLED_WIDTH + 1];

    if (length > OLED_WIDTH) {
        return;
    }

    packet[0] = 0x40;

    memcpy(&packet[1], data, length);

    ESP_ERROR_CHECK(
        i2c_master_transmit(
            oled_device,
            packet,
            length + 1,
            1000
        )
    );
}

static void oled_set_position(uint8_t page)
{
    oled_command(0xB0 | page);
    oled_command(0x00);
    oled_command(0x10);
}

/* ==================== OLED INITIALIZATION ==================== */

static void init_oled(void)
{
    i2c_master_bus_config_t bus_config = {
        .i2c_port = I2C_NUM_0,
        .sda_io_num = OLED_SDA_PIN,
        .scl_io_num = OLED_SCL_PIN,
        .clk_source = I2C_CLK_SRC_DEFAULT,
        .glitch_ignore_cnt = 7,
        .flags.enable_internal_pullup = true
    };

    ESP_ERROR_CHECK(
        i2c_new_master_bus(
            &bus_config,
            &oled_bus
        )
    );

    i2c_device_config_t device_config = {
        .dev_addr_length = I2C_ADDR_BIT_LEN_7,
        .device_address = OLED_ADDRESS,
        .scl_speed_hz = 400000
    };

    ESP_ERROR_CHECK(
        i2c_master_bus_add_device(
            oled_bus,
            &device_config,
            &oled_device
        )
    );

    const uint8_t init_commands[] = {
        0x00,
        0xAE,
        0xD5, 0x80,
        0xA8, 0x3F,
        0xD3, 0x00,
        0x40,
        0x8D, 0x14,
        0x20, 0x02,
        0xA1,
        0xC8,
        0xDA, 0x12,
        0x81, 0x7F,
        0xD9, 0xF1,
        0xDB, 0x40,
        0xA4,
        0xA6,
        0xAF
    };

    ESP_ERROR_CHECK(
        i2c_master_transmit(
            oled_device,
            init_commands,
            sizeof(init_commands),
            1000
        )
    );

    oled_clear();

    printf("OLED initialized.\n");
}

/* ==================== OLED CLEAR ==================== */

static void oled_clear(void)
{
    uint8_t blank[OLED_WIDTH] = {0};

    for (uint8_t page = 0; page < OLED_PAGES; page++) {
        oled_set_position(page);
        oled_send_data(blank, sizeof(blank));
    }
}

/* ==================== BASIC 5x7 FONT ==================== */

static const uint8_t *oled_glyph(char c)
{
    static const uint8_t blank[5] = {0, 0, 0, 0, 0};

    #define GLYPH(ch,a,b,c,d,e) \
        case ch: { \
            static const uint8_t g[5] = {a,b,c,d,e}; \
            return g; \
        }

    switch (c) {
        GLYPH('A',0x7E,0x11,0x11,0x11,0x7E)
        GLYPH('B',0x7F,0x49,0x49,0x49,0x36)
        GLYPH('C',0x3E,0x41,0x41,0x41,0x22)
        GLYPH('D',0x7F,0x41,0x41,0x22,0x1C)
        GLYPH('E',0x7F,0x49,0x49,0x49,0x41)
        GLYPH('F',0x7F,0x09,0x09,0x09,0x01)
        GLYPH('G',0x3E,0x41,0x49,0x49,0x7A)
        GLYPH('H',0x7F,0x08,0x08,0x08,0x7F)
        GLYPH('I',0x00,0x41,0x7F,0x41,0x00)
        GLYPH('J',0x20,0x40,0x41,0x3F,0x01)
        GLYPH('K',0x7F,0x08,0x14,0x22,0x41)
        GLYPH('L',0x7F,0x40,0x40,0x40,0x40)
        GLYPH('M',0x7F,0x02,0x0C,0x02,0x7F)
        GLYPH('N',0x7F,0x04,0x08,0x10,0x7F)
        GLYPH('O',0x3E,0x41,0x41,0x41,0x3E)
        GLYPH('P',0x7F,0x09,0x09,0x09,0x06)
        GLYPH('Q',0x3E,0x41,0x51,0x21,0x5E)
        GLYPH('R',0x7F,0x09,0x19,0x29,0x46)
        GLYPH('S',0x46,0x49,0x49,0x49,0x31)
        GLYPH('T',0x01,0x01,0x7F,0x01,0x01)
        GLYPH('U',0x3F,0x40,0x40,0x40,0x3F)
        GLYPH('V',0x1F,0x20,0x40,0x20,0x1F)
        GLYPH('W',0x3F,0x40,0x38,0x40,0x3F)
        GLYPH('X',0x63,0x14,0x08,0x14,0x63)
        GLYPH('Y',0x07,0x08,0x70,0x08,0x07)
        GLYPH('Z',0x61,0x51,0x49,0x45,0x43)

        GLYPH('0',0x3E,0x51,0x49,0x45,0x3E)
        GLYPH('1',0x00,0x42,0x7F,0x40,0x00)
        GLYPH('2',0x42,0x61,0x51,0x49,0x46)
        GLYPH('3',0x21,0x41,0x45,0x4B,0x31)
        GLYPH('4',0x18,0x14,0x12,0x7F,0x10)
        GLYPH('5',0x27,0x45,0x45,0x45,0x39)
        GLYPH('6',0x3C,0x4A,0x49,0x49,0x30)
        GLYPH('7',0x01,0x71,0x09,0x05,0x03)
        GLYPH('8',0x36,0x49,0x49,0x49,0x36)
        GLYPH('9',0x06,0x49,0x49,0x29,0x1E)

        GLYPH(':',0x00,0x36,0x36,0x00,0x00)
        GLYPH('.',0x00,0x60,0x60,0x00,0x00)
        GLYPH('%',0x63,0x13,0x08,0x64,0x63)
        GLYPH('-',0x08,0x08,0x08,0x08,0x08)
        GLYPH('/',0x20,0x10,0x08,0x04,0x02)
        GLYPH(' ',0,0,0,0,0)

        default:
            return blank;
    }

    #undef GLYPH
}

/* ==================== OLED TEXT OUTPUT ==================== */

static void oled_write_line(uint8_t page, const char *text)
{
    uint8_t pixels[OLED_WIDTH] = {0};
    size_t x = 0;

    oled_set_position(page);

    while (*text && x + 6 <= OLED_WIDTH) {
        char c = *text++;

        if (c >= 'a' && c <= 'z') {
            c = c - 'a' + 'A';
        }

        const uint8_t *glyph = oled_glyph(c);

        for (int col = 0; col < 5; col++) {
            pixels[x++] = glyph[col];
        }

        pixels[x++] = 0x00;
    }

    oled_send_data(pixels, sizeof(pixels));
}

/* ==================== EDGEGUARD OLED DISPLAY ==================== */

static void update_oled(
    float distance_cm,
    float water_level,
    bool reading_valid,
    bool alarm_condition,
    bool alarm_is_muted
)
{
    char line[24];

    oled_clear();

    oled_write_line(0, "EDGEGUARD PUMP");

    if (!reading_valid || sensor_fault) {
        oled_write_line(2, "SENSOR ERROR");
        oled_write_line(4, "CHECK SENSOR");
        oled_write_line(5, "PUMP: OFF");
        oled_write_line(6, "STATUS: SENSOR ERROR");
    } else {
        snprintf(
            line,
            sizeof(line),
            "LEVEL: %.1f%%",
            water_level
        );

        oled_write_line(2, line);

        snprintf(
            line,
            sizeof(line),
            "DIST: %.1f CM",
            distance_cm
        );

        oled_write_line(4, line);

        if (pump_fault) {
            oled_write_line(5, "PUMP: FAULT");
        } else if (pump_running) {
            oled_write_line(5, "PUMP: ON");
        } else {
            oled_write_line(5, "PUMP: OFF");
        }

        if (pump_fault) {
            oled_write_line(6, "STATUS: PUMP FAULT");
        } else if (water_level >= HIGH_LEVEL_THRESHOLD) {
            oled_write_line(6, "STATUS: HIGH");
        } else {
            oled_write_line(6, "STATUS: NORMAL");
        }
    }

    /*
     * The alarm status is independent of the underlying fault.
     * Muting silences the buzzer, not the warning.
     */
    if (alarm_condition && alarm_is_muted) {
        oled_write_line(7, "ALARM: MUTED");
    } else if (alarm_condition) {
        oled_write_line(7, "ALARM: ACTIVE");
    } else {
        oled_write_line(7, "ALARM: READY");
    }
}

