#ifndef _MOTOR_H_
#define _MOTOR_H_
#include "sensor.h"

#include "driver/ledc.h"
#include "driver/gpio.h"
#include "esp_heap_caps.h"
#include <driver/pulse_cnt.h>
#include <driver/pcnt_types_legacy.h>
#include <driver/i2c_master.h>
#include "esp_adc/adc_oneshot.h"
#include "esp_adc/adc_cali.h"
#include "esp_adc/adc_cali_scheme.h"

#define emalloc(size) malloc(size)

struct LowsideCurrentSense {
    adc_oneshot_unit_handle_t adc_handle;

    adc_channel_t channel_a;
    adc_channel_t channel_b;
    adc_channel_t channel_c;

    float adc_value_a;
    float adc_value_b;
    float adc_value_c;
    float voltage_a;
    float voltage_b;
    float voltage_c;
    float current_a;
    float current_b;
    float current_c;
    float iq;
    float lowpass_filter;

    float offset_a;
    float offset_b;
    float offset_c;

    float gain_a;
    float gain_b;
    float gain_c;
    float shunt_resistor;
    float gain;
};

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
    struct LowsideCurrentSense *lowside_current_sense;
};

void foc_init();
struct Motor* new_foc_motor(gpio_num_t pin_pwm1, gpio_num_t pin_pwm2, gpio_num_t pin_pwm3,
                        ledc_channel_t channel1,
                        ledc_channel_t channel2,
                        ledc_channel_t channel3,
                        ledc_timer_t timer,
                        int pp);

void motor_run(struct Motor* motor);
void motor_align(struct Motor* motor);

// INA181A2ID
/*
GPIO 引脚	ADC 单元	ADC 通道	注意事项
GPIO 1	ADC1	CH0	
GPIO 2	ADC1	CH1	
GPIO 3	ADC1	CH2	
GPIO 4	ADC1	CH3	
GPIO 5	ADC1	CH4	
GPIO 6	ADC1	CH5	
GPIO 7	ADC1	CH6	
GPIO 8	ADC1	CH7	
GPIO 9	ADC1	CH8	
GPIO 10	ADC1	CH9
GPIO 11	ADC2	CH0	可能与Wi-Fi冲突
GPIO 12	ADC2	CH1	可能与Wi-Fi冲突
GPIO 13	ADC2	CH2	可能与Wi-Fi冲突
GPIO 14	ADC2	CH3	可能与Wi-Fi冲突
*/
adc_channel_t get_channel_by_pin(int pin);
adc_oneshot_unit_handle_t new_lowside_current_sense_adc_unit();
struct LowsideCurrentSense* new_lowside_current_sense(adc_oneshot_unit_handle_t adc_handle, float shunt_resistor, float gain, int pin_a, int pin_b, int pin_c);
void lowside_current_sense_init(struct LowsideCurrentSense *lcs);
void lowside_current_sense_read_voltage(struct LowsideCurrentSense *lcs);
void lowside_current_sense_read_current(struct LowsideCurrentSense *lcs);
float lowside_current_sense_get_iq(struct LowsideCurrentSense *lcs, float angle);

// foc
#define _constrain(amt,low,high) ((amt)<(low)?(low):((amt)>(high)?(high):(amt)))
void motor_set_pwm(struct Motor* motor, float ua, float ub, float uc);
float _normalizeAngle(float angle);
float _electricalAngle(struct Motor* motor);
void setPhaseVoltage(struct Motor* motor, float Uq,float Ud, float angle_el);
void setTorque(struct Motor* motor, float Uq, float angle_el);
void setTorqueSVPWM(struct Motor* motor, float Uq, float angle_el);
float velocityOpenloop(struct Motor* motor, float target_velocity);
float positionClosedloop(struct Motor* motor, float target_angle);
float velocityClosedloop(struct Motor* motor, float target_velocity);

#endif