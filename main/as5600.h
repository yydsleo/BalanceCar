#ifndef _AS5600_H_
#define _AS5600_H_ 
#include "sensor.h"

#include <driver/pulse_cnt.h>
#include <driver/pcnt_types_legacy.h>
#include <driver/i2c_master.h>
#include "esp_log.h"


#define AS5600_ADDRESS 0x36
// 原始角度值
#define AS5600_REG_RAW_ANGLE 0x0C
// 滤波后的角度值
#define AS5600_REG_ANGLE 0x0E
// 状态寄存器
#define AS5600_REG_STATUS 0x0B
// 配置寄存器
#define AS5600_REG_CONFIG 0x09

struct AS5600 {
    i2c_master_dev_handle_t i2c_dev_handle;
    float full_ratations;
    float angle_without_track;
    float angle;
    float velocity;

    // 计算速度相关参数
    int64_t prev_us;
    float prev_angle;
};

struct Sensor* new_as5600(gpio_num_t pin_sda, gpio_num_t pin_scl, i2c_port_t port);

#endif