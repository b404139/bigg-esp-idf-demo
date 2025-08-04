#include <stdio.h>
#include "esp_system.h"
#include "esp_log.h"
#include "driver/rmt_tx.h"
#include "led_strip.h"
#include "driver/gpio.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "bmi270.h"
#include "common/common.h"

#define WS2812_GPIO 48
#define LED_COUNT 1


#define HW_ESP_SPOT_C5      0
#define HW_ESP_SPOT_S3      1
#define HW_ESP_ASTOM_S3     0

#if HW_ESP_SPOT_C5
#define I2C_INT_IO              3
#define I2C_MASTER_SCL_IO       26
#define I2C_MASTER_SDA_IO       25
#elif HW_ESP_SPOT_S3
#define I2C_INT_IO              5
#define I2C_MASTER_SCL_IO       1
#define I2C_MASTER_SDA_IO       2
#elif HW_ESP_ASTOM_S3
#define I2C_INT_IO              16
#define I2C_MASTER_SCL_IO       0
#define I2C_MASTER_SDA_IO       45
#endif

#define I2C_MASTER_FREQ_HZ      (100 * 1000)

static bmi270_handle_t bmi_handle = NULL;
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
                .r_pos = 1,  // GRB排列
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
        .flags = {.with_dma = true}
    };

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

// BMI270 初始化
static esp_err_t i2c_sensor_bmi270_init(void)
{
    const i2c_config_t i2c_bus_conf = {
        .mode = I2C_MODE_MASTER,
        .sda_io_num = I2C_MASTER_SDA_IO,
        .sda_pullup_en = GPIO_PULLUP_ENABLE,
        .scl_io_num = I2C_MASTER_SCL_IO,
        .scl_pullup_en = GPIO_PULLUP_ENABLE,
        .master.clk_speed = I2C_MASTER_FREQ_HZ
    };
    i2c_bus = i2c_bus_create(I2C_NUM_0, &i2c_bus_conf);
    if (!i2c_bus) {
        ESP_LOGE(TAG, "I2C bus create failed");
        return ESP_FAIL;
    }

    bmi270_i2c_config_t i2c_bmi270_conf = {
        .i2c_handle = i2c_bus,
        .i2c_addr = BMI270_I2C_ADDRESS,
    };
    if (bmi270_sensor_create(&i2c_bmi270_conf, &bmi_handle) != ESP_OK || !bmi_handle) {
        ESP_LOGE(TAG, "BMI270 create failed");
        return ESP_FAIL;
    }

    return ESP_OK;
}

void gesture_task(void *arg)
{
    uint16_t int_status = 0;
    struct bmi2_feat_sensor_data sens_data = { .type = BMI2_WRIST_GESTURE };
    const char *gesture_output[6] = {
        "unknown_gesture", "push_arm_down", "pivot_up",
        "wrist_shake_jiggle", "flick_in", "flick_out"
    };

    // 配置并启动手势识别
    struct bmi2_sens_config config = {.type = BMI2_WRIST_GESTURE};
    uint8_t sens_list[] = {BMI2_ACCEL, BMI2_WRIST_GESTURE};
    bmi270_sensor_enable(sens_list, 2, bmi_handle);
    bmi270_get_sensor_config(&config, 1, bmi_handle);
    config.cfg.wrist_gest.wearable_arm = BMI2_ARM_LEFT;
    bmi270_set_sensor_config(&config, 1, bmi_handle);

    struct bmi2_sens_int_config sens_int = {
        .type = BMI2_WRIST_GESTURE,
        .hw_int_pin = BMI2_INT1
    };
    bmi270_map_feat_int(&sens_int, 1, bmi_handle);

    ESP_LOGI(TAG, "Gesture detection started");

    while (1) {
        bmi2_get_int_status(&int_status, bmi_handle);

        if (int_status & BMI270_WRIST_GEST_STATUS_MASK) {
            bmi270_get_feature_data(&sens_data, 1, bmi_handle);
            int gesture = sens_data.sens_data.wrist_gesture_output;
            ESP_LOGI(TAG, "Detected gesture: %s", gesture_output[gesture]);

            switch (gesture) {
                case 0: // unknown
                    set_led_color(led_strip, 0, 0, 0);
                    break;
                case 1: // push_arm_down 红色
                    set_led_color(led_strip, 255, 0, 0);
                    break;
                case 2: // pivot_up 绿色
                    set_led_color(led_strip, 0, 255, 0);
                    break;
                case 3: // wrist_shake_jiggle 蓝色
                    set_led_color(led_strip, 0, 0, 255);
                    break;
                case 4: // flick_in 黄色
                    set_led_color(led_strip, 255, 255, 0);
                    break;
                case 5: // flick_out 紫色
                    set_led_color(led_strip, 128, 0, 128);
                    break;
                default:
                    set_led_color(led_strip, 0, 0, 0);
                    break;
            }
            vTaskDelay(pdMS_TO_TICKS(500));
            set_led_color(led_strip, 0, 0, 0);
        }
        vTaskDelay(pdMS_TO_TICKS(100));
    }
}

void app_main(void)
{
    ESP_ERROR_CHECK(i2c_sensor_bmi270_init());
    led_strip = configure_led();

    xTaskCreate(gesture_task, "gesture_task", 4096, NULL, 5, NULL);
}
