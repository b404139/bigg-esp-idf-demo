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

/**\name Orientation output macros */
#define BMI270_LEGACY_FACE_UP UINT8_C(0x00)
#define BMI270_LEGACY_FACE_DOWN UINT8_C(0x01)

#define BMI270_LEGACY_PORTRAIT_UP_RIGHT UINT8_C(0x00)
#define BMI270_LEGACY_LANDSCAPE_LEFT UINT8_C(0x01)
#define BMI270_LEGACY_PORTRAIT_UP_DOWN UINT8_C(0x02)
#define BMI270_LEGACY_LANDSCAPE_RIGHT UINT8_C(0x03)

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
    uint8_t data = 0;
    /* Variable to get interrupt status. */
    uint16_t int_status = 0;
    /* Variables to store the output of orientation. */
    uint8_t orientation_out = 0;
    uint8_t orientation_faceup_down = 0;
    /* Structure to define type of sensor and their respective data. */
    struct bmi2_feat_sensor_data sensor_data = {0};

    /* Sensor initialization configuration. */
    struct bmi2_dev dev;
    /* Structure to define the type of sensor and its configurations. */
    struct bmi2_sens_config sens_config;
    /* Select features and their pins to be mapped to. */
    struct bmi2_sens_int_config sens_int_config[] = {
        {.type = BMI2_TAP, .hw_int_pin = BMI2_INT1},
        {.type = BMI2_ORIENTATION, .hw_int_pin = BMI2_INT1},
    };
    /* Accel sensor and feature are listed in array. */
    uint8_t sens_list[] = {BMI2_ACCEL, BMI2_SINGLE_TAP, BMI2_DOUBLE_TAP, BMI2_TRIPLE_TAP, BMI2_ORIENTATION};

    rslt = bmi2_interface_init(&dev, BMI2_I2C_INTF, 0x68, i2c_bus);
    bmi2_error_codes_print_result(rslt);
    rslt = bmi270_legacy_init(&dev);
    bmi2_error_codes_print_result(rslt);
    rslt = bmi270_legacy_sensor_enable(sens_list, sizeof(sens_list), &dev);
    bmi2_error_codes_print_result(rslt);

    // 设置ORIENTATION参数
    sens_config.type = BMI2_ORIENTATION;
    rslt = bmi270_legacy_get_sensor_config(&sens_config, 1, &dev);
    bmi2_error_codes_print_result(rslt);
    sens_config.cfg.orientation.ud_en = 1;
    sens_config.cfg.orientation.mode = 0;
    sens_config.cfg.orientation.blocking = 3;
    sens_config.cfg.orientation.theta = 0x30;
    sens_config.cfg.orientation.hysteresis = 0x20;
    rslt = bmi270_legacy_set_sensor_config(&sens_config, 1, &dev);
    bmi2_error_codes_print_result(rslt);

    /* Map the tap feature interrupt. */
    rslt = bmi270_legacy_map_feat_int(sens_int_config, 2, &dev);
    bmi2_error_codes_print_result(rslt);

    ESP_LOGI(TAG, "Legacy detection started");

    while (1)
    {
        vTaskDelay(pdMS_TO_TICKS(100));
        bmi2_get_int_status(&int_status, &dev);

        if (int_status & BMI270_LEGACY_TAP_STATUS_MASK)
        {

            printf("Tap interrupt is generated\n");
            rslt = bmi2_get_regs(BMI270_LEGACY_TAP_STATUS_REG, &data, 1, &dev);
            bmi2_error_codes_print_result(rslt);

            if (data & BMI270_LEGACY_SINGLE_TAP_MASK)
            {
                printf("Single tap asserted\n");
                set_led_color(led_strip, 255, 0, 0);
            }

            if (data & BMI270_LEGACY_DOUBLE_TAP_MASK)
            {
                printf("Double tap asserted\n");
                set_led_color(led_strip, 0, 255, 0);
            }

            if (data & BMI270_LEGACY_TRIPLE_TAP_MASK)
            {
                printf("Triple tap asserted\n");
                set_led_color(led_strip, 0, 0, 255);
            }
            vTaskDelay(pdMS_TO_TICKS(500));
            set_led_color(led_strip, 0, 0, 0);
        }
        else if (int_status & BMI270_LEGACY_ORIENT_STATUS_MASK)
        {
            printf("Orientation interrupt is generated---------------\n");
            sensor_data.type = BMI2_ORIENTATION;
            rslt = bmi270_legacy_get_feature_data(&sensor_data, 1, &dev);

            orientation_out = sensor_data.sens_data.orient_output.portrait_landscape;
            orientation_faceup_down = sensor_data.sens_data.orient_output.faceup_down;

            printf("The Orientation output is %d \n", orientation_out);
            printf("The Orientation faceup/down output is %d\n", orientation_faceup_down);

            switch (orientation_out)
            {
            case BMI270_LEGACY_LANDSCAPE_LEFT:
                printf("Orientation state is landscape left\n");
                break;
            case BMI270_LEGACY_LANDSCAPE_RIGHT:
                printf("Orientation state is landscape right\n");
                break;
            case BMI270_LEGACY_PORTRAIT_UP_DOWN:
                printf("Orientation state is portrait upside down\n");
                break;
            case BMI270_LEGACY_PORTRAIT_UP_RIGHT:
                printf("Orientation state is portrait upright\n");
                break;
            default:
                printf("orientation_out state is default\n");
                break;
            }

            switch (orientation_faceup_down)
            {
            case BMI270_LEGACY_FACE_UP:
                printf("Orientation state is face up\n");
                break;
            case BMI270_LEGACY_FACE_DOWN:
                printf("Orientation state is face down\n");
                break;
            default:
                printf("orientation_faceup_down state is default\n");
                break;
            }
        }
    }
}

void app_main(void)
{
    ESP_ERROR_CHECK(i2c_bus_init());
    led_strip = configure_led();

    xTaskCreate(legacy_task, "legacy_task", 4096, NULL, 5, NULL);
}
