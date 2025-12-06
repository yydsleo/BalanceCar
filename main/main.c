#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"
#include "esp_heap_caps.h"
#include "motor_foc.h"
#include "esp_rom_gpio.h"
#include "as5600.h"
#include "lowside_current_sense.h"

static const char* TAG = "main";
struct Motor *left_motor, *right_motor;

void foc_driver() {
    while (1) {
        ulTaskNotifyTake(pdTRUE, portMAX_DELAY);
        left_motor->run(left_motor);
        right_motor->run(right_motor);
    }
}

void app_main(void)
{
    TaskHandle_t foc_driver_task_handle;
    // foc_init();
    gpio_set_direction(7, GPIO_MODE_OUTPUT);
    gpio_set_level(7, 1);
    // init motor
    // adc unit
    left_motor = new_foc_motor(12, 11, 10, 0, 7);
    left_motor->sensor = new_as5600(8, 9, I2C_NUM_0);
    left_motor->name = "left";
    left_motor->current_sense = new_lowside_current_sense(0.005f, 50.0f, 4, 5, 0, left_motor->mcpwm_timer_1);
    // 电流环初始化之前必须要这样
    motor_set_pwm(left_motor, left_motor->voltage_power_supply / 2, left_motor->voltage_power_supply / 2, left_motor->voltage_power_supply / 2);
    left_motor->current_sense->init(left_motor->current_sense);
    left_motor->init(left_motor);

    right_motor = new_foc_motor(35, 34, 33, 1, 7);
    right_motor->sensor = new_as5600(37, 36, I2C_NUM_1);
    right_motor->name = "right";
    right_motor->current_sense = new_lowside_current_sense(0.005f, 50.0f, 2, 3, 0, right_motor->mcpwm_timer_1);
    // 电流环初始化之前必须要这样
    motor_set_pwm(right_motor, right_motor->voltage_power_supply / 2, right_motor->voltage_power_supply / 2, right_motor->voltage_power_supply / 2);
    right_motor->current_sense->init(right_motor->current_sense);
    right_motor->init(right_motor);

    left_motor->target_velocity = 10;
    right_motor->target_velocity = 10;

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