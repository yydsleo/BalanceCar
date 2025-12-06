#ifndef _MOTOR_H_
#define _MOTOR_H_
#include "sensor.h"
#include "current_sense.h"

#include "driver/ledc.h"
#include "driver/gpio.h"
#include "esp_heap_caps.h"
#include <driver/pulse_cnt.h>
#include <driver/pcnt_types_legacy.h>
#include <driver/i2c_master.h>
#include "esp_adc/adc_oneshot.h"
#include "esp_adc/adc_cali.h"
#include "esp_adc/adc_cali_scheme.h"
// #include <driver/mcpwm.h>
// #include <driver/mcpwm_types_legacy.h>
#include <driver/mcpwm_prelude.h>

#define emalloc(size) malloc(size)

#define MOTOR_DIRECTION_CW 1
#define MOTOR_DIRECTION_CCW -1

struct Motor {
    char *name;
    gpio_num_t pin_pwm_1;
    gpio_num_t pin_pwm_2;
    gpio_num_t pin_pwm_3;
    gpio_num_t pin_en;

    ledc_channel_t channel1;
    ledc_channel_t channel2;
    ledc_channel_t channel3;

    int mcpwm_unit;
    mcpwm_timer_handle_t mcpwm_timer_1;
    mcpwm_timer_handle_t mcpwm_timer_2;
    mcpwm_timer_handle_t mcpwm_timer_3;
    mcpwm_oper_handle_t mcpwm_operator_1;
    mcpwm_oper_handle_t mcpwm_operator_2;
    mcpwm_oper_handle_t mcpwm_operator_3;
    mcpwm_cmpr_handle_t mcpwm_comparator_1;
    mcpwm_cmpr_handle_t mcpwm_comparator_2;
    mcpwm_cmpr_handle_t mcpwm_comparator_3;
    mcpwm_gen_handle_t mcpwm_generator_1;
    mcpwm_gen_handle_t mcpwm_generator_2;
    mcpwm_gen_handle_t mcpwm_generator_3;

    int loop;
    int cnt;

    // FOC参数
    int pp; // pole pair number
    float voltage_power_supply;
    float voltage_limit;
    float shaft_angle;
    float open_loop_timestamp;
    float zero_electric_angle;
    float Ualpha;
    float Ubeta;
    float Ua;
    float Ub;
    float Uc;
    float dc_a;
    float dc_b;
    float dc_c;
    int direction;
    // 速度PID参数
    float integral;
    float prev_dvelocity;
    float target_velocity;

    int64_t start_ts;
    // 编码器
    struct Sensor* sensor;
    // 电流采样
    struct CurrentSense *current_sense;

    void (*init)(struct Motor* motor);
    void (*run)(struct Motor* motor);
};

void foc_init();
struct Motor* new_foc_motor(gpio_num_t pin_in1, gpio_num_t pin_in2, gpio_num_t pin_in3,
                        int mcpwm_unit,
                        int pp);

// foc
#define _constrain(amt,low,high) ((amt)<(low)?(low):((amt)>(high)?(high):(amt)))
void motor_set_pwm(struct Motor* motor, float ua, float ub, float uc);
float _normalizeAngle(float angle);
float _electricalAngle(struct Motor* motor);
void setPhaseVoltage(struct Motor* motor, float Uq,float Ud, float angle_el);
void setTorque(struct Motor* motor, float Uq, float angle_el);
void setTorqueSVPWM(struct Motor* motor, float Uq, float angle_el);
float getIQ(struct Motor* motor, struct PhaseCurrent current, float angle);
float velocityOpenloop(struct Motor* motor, float target_velocity);
float positionClosedloop(struct Motor* motor, float target_angle);
float velocityClosedloop(struct Motor* motor, float target_velocity);

#endif