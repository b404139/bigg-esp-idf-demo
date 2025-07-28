#include <stdio.h>
#include <string.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "adc_battery_estimation.h"

#define TEST_ADC_UNIT (ADC_UNIT_1)
#define TEST_ADC_BITWIDTH (ADC_BITWIDTH_DEFAULT)
#define TEST_ADC_ATTEN (ADC_ATTEN_DB_12)
#define TEST_ADC_CHANNEL (ADC_CHANNEL_1)
#define TEST_RESISTOR_UPPER (10000)//分压电阻，比例相同即可
#define TEST_RESISTOR_LOWER (10000)
#define TEST_ESTIMATION_TIME (50)

void app_main(void)
{
    //测试乐鑫的电量检测组件adc_battery_estimation
    //ADC_UNIT_1.ADC_CHANNEL_0 = ESP32S3.GPIO1
    //ADC_UNIT_1.ADC_CHANNEL_1 = ESP32S3.GPIO2

    //测试阶段，注释adc_battery_estimation.c的以下代码，以禁用低通滤波和单调性判断。
    //否则给GPIO拔插跳线时，测量电量会锁定为0%
    /*
    // // Apply low-pass filter and handle capacity monotonicity
    // if (!ctx->is_first_read) {
    //     // Apply low-pass filter
    //     ---
    // } else {
    //     // First reading, just store it
    //     ctx->is_first_read = false;
    // }
     */
    //可选配置：sdkconfig-> ADC Battery Estimation-> ADC Filter Window size

    //GPIO2=1.6V capacity=60%
    //GPIO2=3.3V capacity=100%
    static const battery_point_t battery_ponint_table[]={
            { 6.6 ,  100},
            { 3.3 ,  60}, //1:1分压的ADC读取到的是1.6V，即实际电池电压3.3V
            { 1.6 ,  30},
            { 1 ,  0}
    };
    adc_battery_estimation_t config = {
        .internal = {
            .adc_unit = TEST_ADC_UNIT,
            .adc_bitwidth = TEST_ADC_BITWIDTH,
            .adc_atten = TEST_ADC_ATTEN,
        },
        .adc_channel = TEST_ADC_CHANNEL,
        .lower_resistor = TEST_RESISTOR_LOWER,
        .upper_resistor = TEST_RESISTOR_UPPER,
        .battery_points = battery_ponint_table,
        .battery_points_count = sizeof(battery_ponint_table) / sizeof(battery_ponint_table[0])
    };

    adc_battery_estimation_handle_t adc_battery_estimation_handle = adc_battery_estimation_create(&config);

    while (1){
    //for (int i = 0; i < TEST_ESTIMATION_TIME; i++) {
        float capacity = 0;
        adc_battery_estimation_get_capacity(adc_battery_estimation_handle, &capacity);
        printf("Battery capacity: %.1f%%\n", capacity);
        vTaskDelay(pdMS_TO_TICKS(500));
    }

    adc_battery_estimation_destroy(adc_battery_estimation_handle);
}
