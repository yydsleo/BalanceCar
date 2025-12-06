#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "motor_foc.h"
#include "esp_log.h"
#include "esp_timer.h"
#include <driver/ledc.h>
#include <driver/i2c_master.h>
#include <string.h>
#include <math.h>
#include <driver/pulse_cnt.h>
#include <driver/pcnt_types_legacy.h>

#define MOTOR_PWM_FREQ 20000


// SVPWM
#define _SQRT3 1.73205080757f
#define _SQRT3_2 0.86602540378f
#define _1_SQRT3 0.57735026919f
#define _2_SQRT3 1.15470053838f
#define _PI 3.14159265359f
#define _PI_2 1.57079632679f
#define _PI_3 1.0471975512f
#define _3PI_2 4.71238898038f
#define _2PI 6.28318530718f


static const char *TAG = "motor";

void motor_align(struct Motor* motor);
    
void motor_init(struct Motor* motor) {
    motor_align(motor);
}

void motor_run(struct Motor* motor) { 
    velocityClosedloop(motor, motor->target_velocity);
}

void foc_init() {
    // init pwm
    ledc_timer_config_t pwm_timer = {
        .speed_mode = LEDC_LOW_SPEED_MODE,
        .duty_resolution = LEDC_TIMER_10_BIT,
        .timer_num = LEDC_TIMER_0,
        .freq_hz = MOTOR_PWM_FREQ,
        .clk_cfg = LEDC_AUTO_CLK
    };
    esp_err_t ret = ledc_timer_config(&pwm_timer);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to initialize PWM timer");
        return;
    }
    // init pwm
    ledc_timer_config_t pwm_timer1 = {
        .speed_mode = LEDC_LOW_SPEED_MODE,
        .duty_resolution = LEDC_TIMER_10_BIT,
        .timer_num = LEDC_TIMER_1,
        .freq_hz = MOTOR_PWM_FREQ,
        .clk_cfg = LEDC_AUTO_CLK
    };
    ret = ledc_timer_config(&pwm_timer1);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to initialize PWM timer");
        return;
    }
}

struct Motor* new_foc_motor_ledc(gpio_num_t pin_in1, gpio_num_t pin_in2, gpio_num_t pin_in3,
                            ledc_channel_t channel1,
                            ledc_channel_t channel2,
                            ledc_channel_t channel3,
                            ledc_timer_t timer,
                            int pp) {
    struct Motor* motor = emalloc(sizeof(struct Motor));
    if (motor == NULL) {
        ESP_LOGE(TAG, "Failed to allocate memory for motor");
        return NULL;
    }
    memset(motor, 0, sizeof(struct Motor));
    motor->pin_pwm_1 = pin_in1;
    motor->pin_pwm_2 = pin_in2;
    motor->pin_pwm_3 = pin_in3;

    gpio_set_direction(pin_in1, GPIO_MODE_OUTPUT);
    gpio_set_direction(pin_in2, GPIO_MODE_OUTPUT);
    gpio_set_direction(pin_in3, GPIO_MODE_OUTPUT);

    gpio_set_level(pin_in1, 0);
    gpio_set_level(pin_in2, 0);
    gpio_set_level(pin_in3, 0);

    // 配置两个PWM通道(对应H桥的两个输入)
    ledc_channel_config_t pwm_channel1 = {
        .gpio_num = pin_in1,
        .speed_mode = LEDC_LOW_SPEED_MODE,
        .channel = channel1,
        .timer_sel = timer,
        .duty = 0,
        .hpoint = 0
    };
    esp_err_t ret = ledc_channel_config(&pwm_channel1);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to configure PWM channel");
        return NULL;
    }
    ledc_channel_config_t pwm_channel2 = {
        .gpio_num = pin_in2,
        .speed_mode = LEDC_LOW_SPEED_MODE,
        .channel = channel2,
        .timer_sel = timer,
        .duty = 0,
        .hpoint = 0
    };
    ret = ledc_channel_config(&pwm_channel2);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to configure PWM channel");
        return NULL;
    }
    ledc_channel_config_t pwm_channel3 = {
        .gpio_num = pin_in3,
        .speed_mode = LEDC_LOW_SPEED_MODE,
        .channel = channel3,
        .timer_sel = timer,
        .duty = 0,
        .hpoint = 0
    };
    ret = ledc_channel_config(&pwm_channel3);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to configure PWM channel");
        return NULL;
    }

    motor->channel1 = channel1;
    motor->channel2 = channel2;
    motor->channel3 = channel3;

    // init foc param
    motor->voltage_power_supply = 8.2;
    motor->voltage_limit = 1.5;
    motor->start_ts = esp_timer_get_time();
    motor->direction = MOTOR_DIRECTION_CW;

    // set pole pair number
    motor->pp = pp;

    // init motor param
    motor->integral = 0.0;
    motor->prev_dvelocity = 0.0;
    motor->target_velocity = 0.0;

    motor->init = motor_init;
    motor->run = motor_run;

    return motor;
}

#define _PWM_FREQUENCY 25000
#define _PWM_TIMEBASE_RESOLUTION_HZ 160e6f
#define PERIOD_TICKS (uint32_t)(_PWM_TIMEBASE_RESOLUTION_HZ / _PWM_FREQUENCY)
struct Motor* new_foc_motor(gpio_num_t pin_in1, gpio_num_t pin_in2, gpio_num_t pin_in3,
                            int mcpwm_unit,
                            int pp) {
    struct Motor* motor = emalloc(sizeof(struct Motor));
    if (motor == NULL) {
        ESP_LOGE(TAG, "Failed to allocate memory for motor");
        return NULL;
    }
    memset(motor, 0, sizeof(struct Motor));
    motor->mcpwm_unit = mcpwm_unit;
    motor->pin_pwm_1 = pin_in1;
    motor->pin_pwm_2 = pin_in2;
    motor->pin_pwm_3 = pin_in3;

    gpio_set_direction(pin_in1, GPIO_MODE_OUTPUT);
    gpio_set_direction(pin_in2, GPIO_MODE_OUTPUT);
    gpio_set_direction(pin_in3, GPIO_MODE_OUTPUT);

    gpio_set_level(pin_in1, 0);
    gpio_set_level(pin_in2, 0);
    gpio_set_level(pin_in3, 0);

    // 创建定时器
    mcpwm_timer_config_t timer_config = {
        .group_id = mcpwm_unit,
        .clk_src = MCPWM_TIMER_CLK_SRC_DEFAULT,
        .resolution_hz = _PWM_TIMEBASE_RESOLUTION_HZ, // 定时器分辨率是1MHz
        .count_mode = MCPWM_TIMER_COUNT_MODE_UP_DOWN,
        // .count_mode = MCPWM_TIMER_COUNT_MODE_UP,
        .intr_priority = 0,
        .period_ticks = PERIOD_TICKS, // 周期为50 ticks->PWM频率=1MHz/50Hz=20kHz
    };
    printf("period_ticks: %d\n", (int)PERIOD_TICKS);
    mcpwm_new_timer(&timer_config, &motor->mcpwm_timer_1);
    mcpwm_new_timer(&timer_config, &motor->mcpwm_timer_2);
    mcpwm_new_timer(&timer_config, &motor->mcpwm_timer_3);
    // 安装操作器
    mcpwm_operator_config_t operator_config = {
        .group_id = mcpwm_unit,
        .intr_priority = 0,
        .flags.update_gen_action_on_tep = true,
        .flags.update_gen_action_on_tez = true,
    };
    mcpwm_new_operator(&operator_config, &motor->mcpwm_operator_1);
    mcpwm_operator_connect_timer(motor->mcpwm_operator_1, motor->mcpwm_timer_1);
    mcpwm_new_operator(&operator_config, &motor->mcpwm_operator_2);
    mcpwm_operator_connect_timer(motor->mcpwm_operator_2, motor->mcpwm_timer_2);
    mcpwm_new_operator(&operator_config, &motor->mcpwm_operator_3);
    mcpwm_operator_connect_timer(motor->mcpwm_operator_3, motor->mcpwm_timer_3);
    // 比较器
    mcpwm_comparator_config_t cmp_config = {
        .intr_priority = 0,
        .flags.update_cmp_on_tez = true,
    };
    mcpwm_new_comparator(motor->mcpwm_operator_1, &cmp_config, &motor->mcpwm_comparator_1);
    mcpwm_new_comparator(motor->mcpwm_operator_2, &cmp_config, &motor->mcpwm_comparator_2);
    mcpwm_new_comparator(motor->mcpwm_operator_3, &cmp_config, &motor->mcpwm_comparator_3);
    mcpwm_comparator_set_compare_value(motor->mcpwm_comparator_1, 0);
    mcpwm_comparator_set_compare_value(motor->mcpwm_comparator_2, 0);
    mcpwm_comparator_set_compare_value(motor->mcpwm_comparator_3, 0);
    // 生成器
    mcpwm_generator_config_t gen_config1 = { .gen_gpio_num = pin_in1, };
    mcpwm_generator_config_t gen_config2 = { .gen_gpio_num = pin_in2, };
    mcpwm_generator_config_t gen_config3 = { .gen_gpio_num = pin_in3, };
    mcpwm_new_generator(motor->mcpwm_operator_1, &gen_config1, &motor->mcpwm_generator_1);
    mcpwm_new_generator(motor->mcpwm_operator_2, &gen_config2, &motor->mcpwm_generator_2);
    mcpwm_new_generator(motor->mcpwm_operator_3, &gen_config3, &motor->mcpwm_generator_3);
    // 设置生成器动作。计数器为0时输出高电平，达到比较值时输出低电平

    mcpwm_generator_set_actions_on_compare_event(motor->mcpwm_generator_1,
                MCPWM_GEN_COMPARE_EVENT_ACTION(MCPWM_TIMER_DIRECTION_UP, motor->mcpwm_comparator_1, MCPWM_GEN_ACTION_LOW),
                MCPWM_GEN_COMPARE_EVENT_ACTION(MCPWM_TIMER_DIRECTION_DOWN, motor->mcpwm_comparator_1, MCPWM_GEN_ACTION_HIGH),
                MCPWM_GEN_COMPARE_EVENT_ACTION_END());
   
    mcpwm_generator_set_action_on_timer_event(motor->mcpwm_generator_1,
                MCPWM_GEN_TIMER_EVENT_ACTION(MCPWM_TIMER_DIRECTION_UP,
                                             MCPWM_TIMER_EVENT_EMPTY,
                                             MCPWM_GEN_ACTION_HIGH));
/*
    mcpwm_generator_set_action_on_compare_event(motor->mcpwm_generator_1,
                MCPWM_GEN_COMPARE_EVENT_ACTION(MCPWM_TIMER_DIRECTION_UP,
                                             motor->mcpwm_comparator_1,
                                             MCPWM_GEN_ACTION_LOW));
    */


    mcpwm_generator_set_actions_on_compare_event(motor->mcpwm_generator_2,
                MCPWM_GEN_COMPARE_EVENT_ACTION(MCPWM_TIMER_DIRECTION_UP, motor->mcpwm_comparator_2, MCPWM_GEN_ACTION_LOW),
                MCPWM_GEN_COMPARE_EVENT_ACTION(MCPWM_TIMER_DIRECTION_DOWN, motor->mcpwm_comparator_2, MCPWM_GEN_ACTION_HIGH),
                MCPWM_GEN_COMPARE_EVENT_ACTION_END());
    mcpwm_generator_set_action_on_timer_event(motor->mcpwm_generator_2,
                MCPWM_GEN_TIMER_EVENT_ACTION(MCPWM_TIMER_DIRECTION_UP,
                                             MCPWM_TIMER_EVENT_EMPTY,
                                             MCPWM_GEN_ACTION_HIGH));
    /*
    mcpwm_generator_set_action_on_compare_event(motor->mcpwm_generator_2,
                MCPWM_GEN_COMPARE_EVENT_ACTION(MCPWM_TIMER_DIRECTION_UP,
                                             motor->mcpwm_comparator_2,
                                             MCPWM_GEN_ACTION_LOW));
    */

    mcpwm_generator_set_actions_on_compare_event(motor->mcpwm_generator_3,
                MCPWM_GEN_COMPARE_EVENT_ACTION(MCPWM_TIMER_DIRECTION_UP, motor->mcpwm_comparator_3, MCPWM_GEN_ACTION_LOW),
                MCPWM_GEN_COMPARE_EVENT_ACTION(MCPWM_TIMER_DIRECTION_DOWN, motor->mcpwm_comparator_3, MCPWM_GEN_ACTION_HIGH),
                MCPWM_GEN_COMPARE_EVENT_ACTION_END());
    mcpwm_generator_set_action_on_timer_event(motor->mcpwm_generator_3,
                MCPWM_GEN_TIMER_EVENT_ACTION(MCPWM_TIMER_DIRECTION_UP,
                                             MCPWM_TIMER_EVENT_EMPTY,
                                             MCPWM_GEN_ACTION_HIGH));
    /*
    mcpwm_generator_set_action_on_compare_event(motor->mcpwm_generator_3,
                MCPWM_GEN_COMPARE_EVENT_ACTION(MCPWM_TIMER_DIRECTION_UP,
                                             motor->mcpwm_comparator_3,
                                             MCPWM_GEN_ACTION_LOW));
    */
    esp_err_t ret;
    // 启动定时器
    ret = mcpwm_timer_enable(motor->mcpwm_timer_1);
    ESP_ERROR_CHECK(ret);
    ret = mcpwm_timer_enable(motor->mcpwm_timer_2);
    ESP_ERROR_CHECK(ret);
    ret = mcpwm_timer_enable(motor->mcpwm_timer_3);
    ESP_ERROR_CHECK(ret);
    // ret = mcpwm_timer_start_stop(motor->mcpwm_timer_1, MCPWM_TIMER_START_NO_STOP);
    ret = mcpwm_timer_start_stop(motor->mcpwm_timer_1, MCPWM_TIMER_START_NO_STOP);
    ESP_ERROR_CHECK(ret);
    ret = mcpwm_timer_start_stop(motor->mcpwm_timer_2, MCPWM_TIMER_START_NO_STOP);
    ESP_ERROR_CHECK(ret);
    ret = mcpwm_timer_start_stop(motor->mcpwm_timer_3, MCPWM_TIMER_START_NO_STOP);
    ESP_ERROR_CHECK(ret);

    // init foc param
    motor->voltage_power_supply = 8.2;
    motor->voltage_limit = 1.5;
    motor->start_ts = esp_timer_get_time();
    motor->direction = MOTOR_DIRECTION_CW;

    // set pole pair number
    motor->pp = pp;

    // init motor param
    motor->integral = 0.0;
    motor->prev_dvelocity = 0.0;
    motor->target_velocity = 0.0;

    motor->init = motor_init;
    motor->run = motor_run;

    return motor;
}

void motor_align(struct Motor* motor) { 
    for (int i = 0; i <= 500; i++) {
        float angle = _3PI_2 + _2PI * i / 500.0f;
        setTorqueSVPWM(motor, motor->voltage_limit, angle);
        motor->sensor->update(motor->sensor);
        vTaskDelay(pdMS_TO_TICKS(2));
    }
    float mid_angle = motor->sensor->read_angle(motor->sensor);
    printf("mid_angle: %f\n", mid_angle);
    for (int i = 500; i >= 0; i--) {
        float angle = _3PI_2 + _2PI * i / 500.0f;
        setTorqueSVPWM(motor, motor->voltage_limit, angle);
        motor->sensor->update(motor->sensor);
        vTaskDelay(pdMS_TO_TICKS(2));
    }

    motor->sensor->update(motor->sensor);
    float end_angle = motor->sensor->read_angle(motor->sensor);
    printf("end_angle: %f\n", end_angle);
    float moved = fabs(mid_angle - end_angle);

    setTorqueSVPWM(motor, 0, 0);
    vTaskDelay(pdMS_TO_TICKS(200));
    printf("moved: %f\n", moved);
    if (moved < _2PI / 101.0f) {
        motor->direction = MOTOR_DIRECTION_CW;
        ESP_LOGE(TAG, "motor align failed");
        // return;
    } else if (mid_angle < end_angle) {
        motor->direction = MOTOR_DIRECTION_CCW;
    } else {
        motor->direction = MOTOR_DIRECTION_CW;
    }
    printf("motor direction: %d\n", motor->direction);

    setTorqueSVPWM(motor, 0, _3PI_2);
    vTaskDelay(pdMS_TO_TICKS(700));
    motor->sensor->update(motor->sensor);
    motor->sensor->read_angle(motor->sensor);
    motor->zero_electric_angle = 0;
    motor->zero_electric_angle = _electricalAngle(motor);
    vTaskDelay(pdMS_TO_TICKS(20));
    setTorqueSVPWM(motor, 0, 0);
    vTaskDelay(pdMS_TO_TICKS(200));

    // 把这个zero_electric_angle设置成-0.2转速会更大，可以到1800rpm
    // motor->zero_electric_angle = 0;
    printf("zero angle: %f\n", motor->zero_electric_angle);
}

// foc
void motor_set_pwm(struct Motor* motor, float ua, float ub, float uc) { 
    // printf("%f %f %f\n", ua, ub, uc);
    motor->dc_a = _constrain(ua / motor->voltage_power_supply, 0, 1);
    motor->dc_b = _constrain(ub / motor->voltage_power_supply, 0, 1);
    motor->dc_c = _constrain(uc / motor->voltage_power_supply, 0, 1);

    uint32_t output_a = (uint32_t)(motor->dc_a * PERIOD_TICKS / 2);
    uint32_t output_b = (uint32_t)(motor->dc_b * PERIOD_TICKS / 2);
    uint32_t output_c = (uint32_t)(motor->dc_c * PERIOD_TICKS / 2);
    mcpwm_comparator_set_compare_value(motor->mcpwm_comparator_1, output_a);
    mcpwm_comparator_set_compare_value(motor->mcpwm_comparator_2, output_b);
    mcpwm_comparator_set_compare_value(motor->mcpwm_comparator_3, output_c);

    /*
    mcpwm_set_duty(motor->mcpwm_unit, motor->mcpwm_timer_1, MCPWM_OPR_A, motor->dc_a * 100);
    mcpwm_set_duty_type(motor->mcpwm_unit, motor->mcpwm_timer_1, MCPWM_OPR_A, MCPWM_DUTY_MODE_0);

    mcpwm_set_duty(motor->mcpwm_unit, motor->mcpwm_timer_2, MCPWM_OPR_A, motor->dc_b * 100);
    mcpwm_set_duty_type(motor->mcpwm_unit, motor->mcpwm_timer_2, MCPWM_OPR_A, MCPWM_DUTY_MODE_0);

    mcpwm_set_duty(motor->mcpwm_unit, motor->mcpwm_timer_3, MCPWM_OPR_A, motor->dc_c * 100);
    mcpwm_set_duty_type(motor->mcpwm_unit, motor->mcpwm_timer_3, MCPWM_OPR_A, MCPWM_DUTY_MODE_0);
    */
    /*
    ledc_set_duty(LEDC_LOW_SPEED_MODE, motor->channel1, motor->dc_a * 1023);
    ledc_update_duty(LEDC_LOW_SPEED_MODE, motor->channel1);
    
    ledc_set_duty(LEDC_LOW_SPEED_MODE, motor->channel2, motor->dc_b * 1023);
    ledc_update_duty(LEDC_LOW_SPEED_MODE, motor->channel2);

    ledc_set_duty(LEDC_LOW_SPEED_MODE, motor->channel3, motor->dc_c * 1023);
    ledc_update_duty(LEDC_LOW_SPEED_MODE, motor->channel3);
    */
}

float _normalizeAngle(float angle){
  float a = fmod(angle, 2*M_PI);
  return a >= 0 ? a : (a + 2*M_PI);  
}

float _electricalAngle(struct Motor* motor) {
    float angle = motor->sensor->read_angle(motor->sensor);
    return _normalizeAngle((motor->direction * motor->pp) * angle) - motor->zero_electric_angle;
}

void setPhaseVoltage(struct Motor* motor, float Uq,float Ud, float angle_el) {
    angle_el = _normalizeAngle(angle_el + motor->zero_electric_angle);
    // 帕克逆变换
    motor->Ualpha =  -Uq*sin(angle_el); 
    motor->Ubeta =   Uq*cos(angle_el); 

    // 克拉克逆变换
    motor->Ua = motor->Ualpha + motor->voltage_power_supply/2;
    motor->Ub = (sqrt(3)*motor->Ubeta-motor->Ualpha)/2 + motor->voltage_power_supply/2;
    motor->Uc = (-motor->Ualpha-sqrt(3)*motor->Ubeta)/2 + motor->voltage_power_supply/2;
    motor_set_pwm(motor, motor->Ua, motor->Ub, motor->Uc);
}
void setTorque(struct Motor* motor, float Uq, float angle_el) {
    setPhaseVoltage(motor, Uq, 0, angle_el);
}


void setTorqueSVPWM(struct Motor* motor, float Uq, float angle_el) {
    if (Uq < 0)
        angle_el += _PI;
    Uq = fabs(Uq);

    // angle_el = _normalizeAngle(angle_el + _PI_2);
    angle_el = _normalizeAngle(angle_el);
    int sector = floor(angle_el / _PI_3) + 1;
    // calculate the duty cycles
    float T1 = _SQRT3 * sin(sector * _PI_3 - angle_el) * Uq / motor->voltage_power_supply;
    float T2 = _SQRT3 * sin(angle_el - (sector - 1.0) * _PI_3) * Uq / motor->voltage_power_supply;
    float T0 = 1 - T1 - T2;

    float Ta, Tb, Tc;
    switch (sector)
    {
    case 1:
        Ta = T1 + T2 + T0 / 2;
        Tb = T2 + T0 / 2;
        Tc = T0 / 2;
        break;
    case 2:
        Ta = T1 + T0 / 2;
        Tb = T1 + T2 + T0 / 2;
        Tc = T0 / 2;
        break;
    case 3:
        Ta = T0 / 2;
        Tb = T1 + T2 + T0 / 2;
        Tc = T2 + T0 / 2;
        break;
    case 4:
        Ta = T0 / 2;
        Tb = T1 + T0 / 2;
        Tc = T1 + T2 + T0 / 2;
        break;
    case 5:
        Ta = T2 + T0 / 2;
        Tb = T0 / 2;
        Tc = T1 + T2 + T0 / 2;
        break;
    case 6:
        Ta = T1 + T2 + T0 / 2;
        Tb = T0 / 2;
        Tc = T1 + T0 / 2;
        break;
    default:
        Ta = 0;
        Tb = 0;
        Tc = 0;
    }

    motor->Ua = Ta * motor->voltage_power_supply;
    motor->Ub = Tb * motor->voltage_power_supply;
    motor->Uc = Tc * motor->voltage_power_supply;

    motor_set_pwm(motor, motor->Ua, motor->Ub, motor->Uc);
}

// 开环控制
float velocityOpenloop(struct Motor* motor, float target_velocity){
    int64_t now_us = esp_timer_get_time();
    float ts = (now_us - motor->start_ts) * 1e-6f;
    motor->shaft_angle = _normalizeAngle(motor->shaft_angle + target_velocity*ts);
    float Uq = motor->voltage_power_supply/3;
    float eletricasl_angle = _electricalAngle(motor);
    // printf("Up: %f, electrical angle: %f\n", Uq, eletricasl_angle);
    setPhaseVoltage(motor, Uq,  0, eletricasl_angle);
    motor->start_ts = now_us;  //用于计算下一个时间间隔
    return Uq;
}

// 闭环控制
float positionClosedloop(struct Motor* motor, float target_angle) { 
    float angle = motor->sensor->read_angle(motor->sensor);

    // show speed
    static int64_t prev_us = 0;
    static float prev_angle = 0;
    static float speed = 0;
    static int cnt = 0;
    int64_t cur_us = esp_timer_get_time();
    float cur_speed = (angle - prev_angle) / ((float)(cur_us - prev_us) * 1e-6);
    speed = speed * 0.85 + cur_speed * 0.15;
    cnt++;
    if (cur_us - prev_us > 1000000) {
        // printf("speed: %f, cnt: %d, angle: %f, target angle: %f\n", cur_speed, cnt, angle, target_angle);
        prev_us = cur_us;
        prev_angle = angle;
        cnt = 0;
    }

    int DIR = 1;
    float kp = 0.133;
    float Uq = _constrain(kp * DIR * (target_angle - DIR * angle) * 180 / M_PI, -6, 6);
    float eletricasl_angle = DIR * _electricalAngle(motor);
    setPhaseVoltage(motor, Uq, 0, eletricasl_angle);

    return Uq;
}

#include "lowside_current_sense.h"

float velocityClosedloop(struct Motor* motor, float target_velocity) {
    motor->sensor->update(motor->sensor);
    float velocity = motor->sensor->read_velocity(motor->sensor);
    float angle = motor->sensor->read_angle(motor->sensor);
    float iq = motor->current_sense->get_foc_current(motor->current_sense, _electricalAngle(motor));

    // 计算电流

    if (motor->cnt >= 1000) {
        // struct PhaseCurrent current = motor->current_sense->read_current(motor->current_sense);
        struct LowsideCurrentSense *lcs = motor->current_sense->current_sense;
        /*
        printf("name: %s, gan_a: %f, b: %f, c: %f\n", motor->name, lcs->gain_a, lcs->gain_b, lcs->gain_c);
        printf("name: %s, a: %f, b: %f, c: %f\n", motor->name, lcs->current_a, lcs->current_b, lcs->current_c);
        // float iq = motor->current_sense->get_foc_current(motor->current_sense, _electricalAngle(motor));
        printf("name: %s, velocity: %f, target velocity: %f, angle: %f, electricalAngle: %f\n",
                motor->name, velocity, target_velocity, angle, _electricalAngle(motor));
        */
        // printf("name :%s, iq: %f\n", motor->name, iq);

        motor->cnt = 0;
        /*
        printf("name: %s, a: %f, b: %f, c: %f\n",
                motor->name,
                motor->lowside_current_sense->current_a,
                motor->lowside_current_sense->current_b,
                motor->lowside_current_sense->current_c);
                */
    }
    motor->cnt++;

    // PI控制
    int64_t cur_us = esp_timer_get_time();
    float dt = (float)(cur_us - motor->start_ts) * 1e-6;
    motor->start_ts = cur_us;
    float dvelocity = target_velocity - velocity;
    float kp = 0.003, ki = 0.002;
    motor->integral += (dvelocity + motor->prev_dvelocity) * 0.5 * dt;
    if (fabs(motor->integral) > 100) {
        motor->integral = motor->integral > 0 ? 100 : -100;
    }
    motor->prev_dvelocity = dvelocity;
    float pi = kp * dvelocity + ki * motor->integral;
    float Up = _constrain(pi * 180 / M_PI, -motor->voltage_limit, motor->voltage_limit);

    setTorqueSVPWM(motor, Up, _electricalAngle(motor));
    return 0;
}

