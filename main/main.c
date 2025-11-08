#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"
#include "esp_heap_caps.h"
#include "motor_foc.h"
#include "esp_rom_gpio.h"

static const char* TAG = "main";
struct Motor *left_motor, *right_motor;

void foc_driver() {
    while (1) {
        ulTaskNotifyTake(pdTRUE, portMAX_DELAY);
        motor_run(left_motor);
        motor_run(right_motor);
    }
}

void app_main(void)
{
    TaskHandle_t foc_driver_task_handle;
    foc_init();
    gpio_set_direction(7, GPIO_MODE_OUTPUT);
    gpio_set_level(7, 1);
    // init motor
    // adc unit
    adc_oneshot_unit_handle_t adc_handle = new_lowside_current_sense_adc_unit();
    left_motor = new_foc_motor(12, 11, 10, LEDC_CHANNEL_0, LEDC_CHANNEL_1, LEDC_CHANNEL_2, LEDC_TIMER_0, 7);
    left_motor->i2c_dev_handle = foc_motor_i2c_init(8, 9, I2C_NUM_0);
    left_motor->name = "left";
    // left_motor->lowside_current_sense = new_lowside_current_sense(2, 3, 0);
    left_motor->lowside_current_sense = new_lowside_current_sense(adc_handle, 0.005f, 50.0f, 2, 3, 0);
    lowside_current_sense_init(left_motor->lowside_current_sense);

    right_motor = new_foc_motor(35, 34, 33, LEDC_CHANNEL_3, LEDC_CHANNEL_4, LEDC_CHANNEL_5, LEDC_TIMER_1, 7);
    right_motor->i2c_dev_handle = foc_motor_i2c_init(37, 36, I2C_NUM_1);
    right_motor->name = "right";
    right_motor->lowside_current_sense = new_lowside_current_sense(adc_handle, 0.005f, 50.0f, 4, 5, 0);
    lowside_current_sense_init(right_motor->lowside_current_sense);

    left_motor->target_velocity = 50;
    right_motor->target_velocity = 50;

    motor_align(left_motor);
    motor_align(right_motor);

    xTaskCreatePinnedToCore(&foc_driver, "foc_driver", 4 * 1024, NULL, 5, &foc_driver_task_handle, 1);

    while(1)
    {
        // vTaskDelay(pdMS_TO_TICKS(1));
        xTaskNotifyGive(foc_driver_task_handle);
        /*
        BaseType_t status = pdFALSE;
        vTaskNotifyGiveFromISR(foc_driver_task_handle, &status);
        if (status == pdTRUE) {
            YIELD_FROM_ISR();
        }
        */
    }
}