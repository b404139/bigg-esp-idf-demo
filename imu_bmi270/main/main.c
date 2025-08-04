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

#include "component/touch_button.h"
#include "iot_button.h"
#include "touch_sensor_lowlevel.h"

static const char *TAG = "main";

#define WS2812_GPIO 48
#define LED_COUNT 1

#define I2C_MASTER_SCL_IO 1
#define I2C_MASTER_SDA_IO 2
#define I2C_MASTER_FREQ_HZ (100 * 1000)
static i2c_bus_handle_t i2c_bus = NULL;
static led_strip_handle_t led_strip;

/**\name Orientation output macros */
#define BMI270_LEGACY_FACE_UP UINT8_C(0x00)
#define BMI270_LEGACY_FACE_DOWN UINT8_C(0x01)
#define BMI270_LEGACY_PORTRAIT_UP_RIGHT UINT8_C(0x00)
#define BMI270_LEGACY_LANDSCAPE_LEFT UINT8_C(0x01)
#define BMI270_LEGACY_PORTRAIT_UP_DOWN UINT8_C(0x02)
#define BMI270_LEGACY_LANDSCAPE_RIGHT UINT8_C(0x03)

#define TOUCH_CHANNEL_IO8        (8)
#define TOUCH_CHANNEL_IO9        (9)
#define TOUCH_CHANNEL_IO10        (10)
#define LIGHT_TOUCH_THRESHOLD  (0.1)
#define HEAVY_TOUCH_THRESHOLD  (0.4)

static esp_err_t configure_led(void)
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

    ESP_ERROR_CHECK(led_strip_new_spi_device(&strip_config, &spi_config, &led_strip));
    ESP_LOGI(TAG, "LED strip initialized (SPI)");
    return ESP_OK;
}

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
    ESP_LOGI(TAG, "I2C bus initialized");
    return ESP_OK;
}

static void touch_button_event_cb9(void *arg, void *data)
{
    button_event_t event = iot_button_get_event(arg);
    set_led_color(led_strip,  255,255,0);
    vTaskDelay(pdMS_TO_TICKS(200));
    set_led_color(led_strip, 0, 0, 0);
    ESP_LOGI(TAG, "Light Button 1: %s", iot_button_get_event_str(event));
}
static void touch_button_event_cb10(void *arg, void *data)
{
    button_event_t event = iot_button_get_event(arg);
    set_led_color(led_strip,  51,51,200);
    vTaskDelay(pdMS_TO_TICKS(200));
    set_led_color(led_strip, 0, 0, 0);
    ESP_LOGI(TAG, "Light Button 1: %s", iot_button_get_event_str(event));
}
void legacy_task(void *arg)
{
    /* Status of api are returned to this variable. */
    int8_t rslt;
    uint8_t data = 0;
    /* Variable to get interrupt status. */
    uint16_t int_status = 0;
    /* Variables to store the output of orientation. */
    uint8_t orientation_faceup_down = 0;
    uint8_t last_orientation_faceup_down = 0xFF;
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
        {.type = BMI2_SIG_MOTION, .hw_int_pin = BMI2_INT1},
    };
    /* Accel sensor and feature are listed in array. */
    uint8_t sens_list[] = {BMI2_ACCEL, BMI2_DOUBLE_TAP, BMI2_ORIENTATION, BMI2_SIG_MOTION};

    rslt = bmi2_interface_init(&dev, BMI2_I2C_INTF, 0x68, i2c_bus);
    bmi2_error_codes_print_result(rslt);
    rslt = bmi270_legacy_init(&dev);
    bmi2_error_codes_print_result(rslt);
    rslt = bmi270_legacy_sensor_enable(sens_list, sizeof(sens_list), &dev);
    bmi2_error_codes_print_result(rslt);

    // 设置 BMI2_SIG_MOTION 参数
    sens_config.type = BMI2_SIG_MOTION;
    rslt = bmi270_legacy_get_sensor_config(&sens_config, 1, &dev);
    bmi2_error_codes_print_result(rslt);
    // sens_config.cfg.sig_motion.block_size = 2;
    // rslt = bmi270_legacy_set_sensor_config(&sens_config, 1, &dev);
    // bmi2_error_codes_print_result(rslt);

    // 设置 BMI2_ORIENTATION 参数
    sens_config.type = BMI2_ORIENTATION;
    rslt = bmi270_legacy_get_sensor_config(&sens_config, 1, &dev);
    bmi2_error_codes_print_result(rslt);
    sens_config.cfg.orientation.ud_en = 1;
    sens_config.cfg.orientation.mode = 0;
    sens_config.cfg.orientation.blocking = 2;
    sens_config.cfg.orientation.theta = 0x30;
    sens_config.cfg.orientation.hysteresis = 0x20;
    rslt = bmi270_legacy_set_sensor_config(&sens_config, 1, &dev);
    bmi2_error_codes_print_result(rslt);

    /* Map the tap feature interrupt. */
    rslt = bmi270_legacy_map_feat_int(sens_int_config, 3, &dev);
    bmi2_error_codes_print_result(rslt);

    ESP_LOGI(TAG, "Legacy detection started");


    /* ============================= Init touch start============================= */ 
    // Register all touch channel
    uint32_t touch_channel_list[] = {TOUCH_CHANNEL_IO8, TOUCH_CHANNEL_IO9, TOUCH_CHANNEL_IO10};
    int total_channel_num = sizeof(touch_channel_list) / sizeof(touch_channel_list[0]);

    // calloc channel_type for every button from the list
    touch_lowlevel_type_t *channel_type = calloc(total_channel_num, sizeof(touch_lowlevel_type_t));
    assert(channel_type);
    for (int i = 0; i  < total_channel_num; i++) {
        channel_type[i] = TOUCH_LOWLEVEL_TYPE_TOUCH;
    }

    touch_lowlevel_config_t low_config = {
        .channel_num = total_channel_num,
        .channel_list = touch_channel_list,
        .channel_type = channel_type,
    };
    esp_err_t ret = touch_sensor_lowlevel_create(&low_config);
    assert(ret == ESP_OK);
    free(channel_type);
    const button_config_t btn_cfg = {
        .short_press_time = 100,
        .long_press_time = 500,
    };

    // ----------- Init TOUCH_CHANNEL_IO9 start----------- 
    button_touch_config_t touch_cfg_9 = {
        .touch_channel = TOUCH_CHANNEL_IO9,
        .channel_threshold = LIGHT_TOUCH_THRESHOLD,
        .skip_lowlevel_init = true,
    };
    button_handle_t btn_light_9 = NULL;
    ret = iot_button_new_touch_button_device(&btn_cfg, &touch_cfg_9, &btn_light_9);
    assert(ret == ESP_OK);

    // Register touch callback
    iot_button_register_cb(btn_light_9, BUTTON_PRESS_DOWN, NULL, touch_button_event_cb9, NULL);
    // ----------- Init TOUCH_CHANNEL_IO9 end----------- 

    // ----------- Init TOUCH_CHANNEL_IO10 start----------- 
    button_touch_config_t touch_cfg_10 = {
        .touch_channel = TOUCH_CHANNEL_IO10,
        .channel_threshold = LIGHT_TOUCH_THRESHOLD,
        .skip_lowlevel_init = true,
    };

    // Create light press button
    button_handle_t btn_light_10 = NULL;
    ret = iot_button_new_touch_button_device(&btn_cfg, &touch_cfg_10, &btn_light_10);
    assert(ret == ESP_OK);

    // Register touch callback
    iot_button_register_cb(btn_light_10, BUTTON_PRESS_DOWN, NULL, touch_button_event_cb10, NULL);
    // ----------- Init TOUCH_CHANNEL_IO10 end----------- 

    touch_sensor_lowlevel_start();
    /* ============================= Init touch end============================= */ 

    static volatile bool legacy_task_running = false;

    while (1)
    {
         if (legacy_task_running) {
            ESP_LOGW(TAG, "Legacy task already running, skipping...");
            continue;
        }
        legacy_task_running = true;
        bmi2_get_int_status(&int_status, &dev);

        if (int_status & BMI270_LEGACY_SIG_MOT_STATUS_MASK)
        {
            printf("Significant motion interrupt is generated\n");
			const TickType_t delay = pdMS_TO_TICKS(100);
			for (int i = 0; i < 5; i++) {
				set_led_color(led_strip, 255, 0, 0);
				vTaskDelay(delay);
				set_led_color(led_strip, 0, 0, 0);
				vTaskDelay(delay);
			}
			
        }

        if (int_status & BMI270_LEGACY_TAP_STATUS_MASK)
        {
            rslt = bmi2_get_regs(BMI270_LEGACY_TAP_STATUS_REG, &data, 1, &dev);
            bmi2_error_codes_print_result(rslt);
            if (data & BMI270_LEGACY_DOUBLE_TAP_MASK || data & BMI270_LEGACY_TRIPLE_TAP_MASK)
            {
                printf("tap asserted\n");
                set_led_color(led_strip, 255, 0, 255);
            }
        }
        
        if (int_status & BMI270_LEGACY_ORIENT_STATUS_MASK)
        {
            sensor_data.type = BMI2_ORIENTATION;
            rslt = bmi270_legacy_get_feature_data(&sensor_data, 1, &dev);

            orientation_faceup_down = sensor_data.sens_data.orient_output.faceup_down;

            // 状态没有变化，不打印、不响应
            if (orientation_faceup_down == last_orientation_faceup_down)
            {
                legacy_task_running = false;
                continue;  // 跳过这次循环
            }
            // 更新记录的状态
            last_orientation_faceup_down = orientation_faceup_down;

            switch (orientation_faceup_down)
            {
                case BMI270_LEGACY_FACE_UP:
                    printf("Orientation state is face up\n");
                    set_led_color(led_strip,  0,255,0);
                    break;
                case BMI270_LEGACY_FACE_DOWN:
                    printf("Orientation state is face down\n");
                    set_led_color(led_strip,  0,0,255);
                    break;
                default:
                    break;
            }
        }
        
        vTaskDelay(pdMS_TO_TICKS(500));
        set_led_color(led_strip, 0, 0, 0);
        legacy_task_running = false;
    }
}

//检测动作：触摸、双击拍打、正立/倒立、晃动
void app_main(void)
{
    i2c_bus_init();
    configure_led();
    xTaskCreate(legacy_task, "legacy_task", 4096, NULL, 5, NULL);
}
