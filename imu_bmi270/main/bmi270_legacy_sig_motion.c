#include <stdio.h>
#include "esp_system.h"
#include "esp_log.h"
#include "driver/rmt_tx.h"
#include "led_strip.h"
#include "driver/gpio.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "bmi270_legacy.h"
#include "common/common.h"

#define WS2812_GPIO 48
#define LED_COUNT 1

#define HW_ESP_SPOT_C5 0
#define HW_ESP_SPOT_S3 1
#define HW_ESP_ASTOM_S3 0

#if HW_ESP_SPOT_C5
#define I2C_INT_IO 3
#define I2C_MASTER_SCL_IO 26
#define I2C_MASTER_SDA_IO 25
#elif HW_ESP_SPOT_S3
#define I2C_INT_IO 5
#define I2C_MASTER_SCL_IO 1
#define I2C_MASTER_SDA_IO 2
#elif HW_ESP_ASTOM_S3
#define I2C_INT_IO 16
#define I2C_MASTER_SCL_IO 0
#define I2C_MASTER_SDA_IO 45
#endif

#define I2C_MASTER_FREQ_HZ (100 * 1000)

static i2c_bus_handle_t i2c_bus = NULL;
static led_strip_handle_t led_strip;

static const char *TAG = "gesture_led";

static led_strip_handle_t configure_led(void)
{
    led_strip_config_t strip_config = {
        .strip_gpio_num = WS2812_GPIO,
        .max_leds = 1,
        .led_model = LED_MODEL_WS2812,
        .color_component_format = {
            .format = {
                .r_pos = 1, // GRB排列
                .g_pos = 0,
                .b_pos = 2,
                .num_components = 3,
            },
        },
        .flags = {.invert_out = false},
    };

    led_strip_spi_config_t spi_config = {
        .clk_src = SPI_CLK_SRC_DEFAULT,
        .spi_bus = SPI2_HOST,
        .flags = {.with_dma = true}};

    led_strip_handle_t led_strip;
    ESP_ERROR_CHECK(led_strip_new_spi_device(&strip_config, &spi_config, &led_strip));
    ESP_LOGI(TAG, "LED strip initialized (SPI)");
    return led_strip;
}

// 设置LED颜色
static void set_led_color(led_strip_handle_t led_strip, uint8_t r, uint8_t g, uint8_t b)
{
    ESP_ERROR_CHECK(led_strip_set_pixel(led_strip, 0, r, g, b));
    ESP_ERROR_CHECK(led_strip_refresh(led_strip));
}

static esp_err_t i2c_bus_init(void)
{
    const i2c_config_t i2c_bus_conf = {
        .mode = I2C_MODE_MASTER,
        .sda_io_num = I2C_MASTER_SDA_IO,
        .sda_pullup_en = GPIO_PULLUP_ENABLE,
        .scl_io_num = I2C_MASTER_SCL_IO,
        .scl_pullup_en = GPIO_PULLUP_ENABLE,
        .master.clk_speed = I2C_MASTER_FREQ_HZ};
    i2c_bus = i2c_bus_create(I2C_NUM_0, &i2c_bus_conf);
    if (!i2c_bus)
    {
        ESP_LOGE(TAG, "I2C bus create failed");
        return ESP_FAIL;
    }

    return ESP_OK;
}

void legacy_task(void *arg)
{
    /* Status of api are returned to this variable. */
    int8_t rslt;
    /* Variable to get interrupt status. */
    uint16_t int_status = 0;

    /* Sensor initialization configuration. */
    struct bmi2_dev dev;
    /* Structure to define the type of sensor and its configurations. */
    struct bmi2_sens_config sens_config;
    /* Select features and their pins to be mapped to. */
    struct bmi2_sens_int_config sens_int_config[] = {
        {.type = BMI2_SIG_MOTION, .hw_int_pin = BMI2_INT1},
    };
    /* Accel sensor and feature are listed in array. */
    uint8_t sens_list[] = {BMI2_ACCEL, BMI2_SIG_MOTION};

    rslt = bmi2_interface_init(&dev, BMI2_I2C_INTF, 0x68, i2c_bus);
    bmi2_error_codes_print_result(rslt);
    rslt = bmi270_legacy_init(&dev);
    bmi2_error_codes_print_result(rslt);
    rslt = bmi270_legacy_sensor_enable(sens_list, sizeof(sens_list), &dev);
    bmi2_error_codes_print_result(rslt);

    // 设置参数
    sens_config.type = BMI2_SIG_MOTION;
    rslt = bmi270_legacy_get_sensor_config(&sens_config, 1, &dev);
    bmi2_error_codes_print_result(rslt);

    /* Map the tap feature interrupt. */
    rslt = bmi270_legacy_map_feat_int(sens_int_config, 1, &dev);
    bmi2_error_codes_print_result(rslt);

    ESP_LOGI(TAG, "Legacy detection started");
    while (1)
    {
        
        bmi2_get_int_status(&int_status, &dev);

        if (int_status & BMI270_LEGACY_SIG_MOT_STATUS_MASK)
        {
            printf("Significant motion interrupt is generated\n");
            set_led_color(led_strip,  0,0,255);
            vTaskDelay(pdMS_TO_TICKS(500));
            set_led_color(led_strip, 0, 0, 0);
        }
    }
    vTaskDelay(pdMS_TO_TICKS(100));
}

void app_main(void)
{
    ESP_ERROR_CHECK(i2c_bus_init());
    led_strip = configure_led();

    xTaskCreate(legacy_task, "legacy_task", 4096, NULL, 5, NULL);
}
