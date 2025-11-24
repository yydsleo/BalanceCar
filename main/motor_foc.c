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

struct Motor *motor_left, *motor_right;
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
    /*
    motor_left = foc_init_motor(MOTOR_PWM1, MOTOR_PWM2, MOTOR_PWM3, MOTOR_EN,
        LEDC_CHANNEL_0, LEDC_CHANNEL_1, LEDC_CHANNEL_2,
        LEDC_TIMER_0);

    motor_left->i2c_dev_handle = foc_motor_i2c_init();
    */
}

struct Motor* new_foc_motor(gpio_num_t pin_in1, gpio_num_t pin_in2, gpio_num_t pin_in3,
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

    return motor;
}

void motor_run(struct Motor* motor) {
    // velocityOpenloop(motor, 70);
    // velocityOpenloop(motor, 70);
    /*
    static float target_angle = 0.0;
    target_angle += 0.020;
    positionClosedloop(motor, target_angle);s
    */

    velocityClosedloop(motor, motor->target_velocity);
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


adc_channel_t get_channel_by_pin(int pin) {
    ESP_ERROR_CHECK(!(pin >= 1 && pin <= 10));
    adc_channel_t table[11] = {0, ADC_CHANNEL_0, ADC_CHANNEL_1, ADC_CHANNEL_2, ADC_CHANNEL_3, ADC_CHANNEL_4, ADC_CHANNEL_5, ADC_CHANNEL_6, ADC_CHANNEL_7, ADC_CHANNEL_8, ADC_CHANNEL_9};
    return table[pin];
}

adc_oneshot_unit_handle_t new_lowside_current_sense_adc_unit() {
    adc_oneshot_unit_handle_t adc_handle;
    adc_oneshot_unit_init_cfg_t init_config1 = {
        .unit_id = ADC_UNIT_1,          // 选择ADC单元，例如ADC_UNIT_1
        .ulp_mode = ADC_ULP_MODE_DISABLE,
    };
    // 创建ADC单元句柄
    esp_err_t ret = adc_oneshot_new_unit(&init_config1, &adc_handle);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "create adc failed!");
        return NULL;
    }

    return adc_handle;
}

struct LowsideCurrentSense* new_lowside_current_sense(adc_oneshot_unit_handle_t adc_handle, float shunt_resistor, float gain, int pin_a, int pin_b, int pin_c) {
    struct LowsideCurrentSense *lowside_current_sense = malloc(sizeof(struct LowsideCurrentSense));
    if (lowside_current_sense == NULL) {
        ESP_LOGE(TAG, "malloc memory failed!");
        return NULL;
    }

    adc_oneshot_chan_cfg_t config = {
        .atten = ADC_ATTEN_DB_12,   // 设置衰减，以调整测量电压范围
        .bitwidth = ADC_BITWIDTH_DEFAULT, // 设置输出位宽
    };
    lowside_current_sense->adc_handle = adc_handle;

    if (pin_a != 0) {
        ESP_ERROR_CHECK(adc_oneshot_config_channel(adc_handle, get_channel_by_pin(pin_a), &config)); // 此处以配置通道2为例
        lowside_current_sense->channel_a = get_channel_by_pin(pin_a);
    }
    if (pin_b != 0) {
        ESP_ERROR_CHECK(adc_oneshot_config_channel(adc_handle, get_channel_by_pin(pin_b), &config)); // 此处以配置通道2为例
        lowside_current_sense->channel_b = get_channel_by_pin(pin_b);
    }
    if (pin_c != 0) {
        ESP_ERROR_CHECK(adc_oneshot_config_channel(adc_handle, get_channel_by_pin(pin_c), &config)); // 此处以配置通道2为例
        lowside_current_sense->channel_c = get_channel_by_pin(pin_c);
    }

    // gain是电压放大倍数，也就是增益系数
    // shut_resistor是采样电阻的阻值
    // 得到电流的原理是：采样的电压/电阻
    lowside_current_sense->shunt_resistor = shunt_resistor;
    lowside_current_sense->gain = gain;

    lowside_current_sense->gain_a = 1.0f / shunt_resistor / gain * -1;
    lowside_current_sense->gain_b = 1.0f / shunt_resistor / gain * -1;
    lowside_current_sense->gain_c = 1.0f / shunt_resistor / gain * -1;

    // other param
    lowside_current_sense->lowpass_filter = 0.05f;
    lowside_current_sense->iq = 0;

    return lowside_current_sense;
}


void lowside_current_sense_init(struct LowsideCurrentSense *lcs) {
    int cnt = 1000;
    float offset_ia = 0;
    float offset_ib = 0;
    float offset_ic = 0;
    for (int i = 0; i < cnt; i++) {
        lowside_current_sense_read_voltage(lcs);
        offset_ia += lcs->voltage_a;
        offset_ib += lcs->voltage_b;
        offset_ic += lcs->voltage_c;
        vTaskDelay(pdMS_TO_TICKS(1));
    }
    lcs->offset_a = offset_ia / cnt;
    lcs->offset_b = offset_ib / cnt;
    lcs->offset_c = offset_ic / cnt;
}

void lowside_current_sense_read_voltage(struct LowsideCurrentSense *lcs) {
    int adc_raw_a, adc_raw_b;
    ESP_ERROR_CHECK(adc_oneshot_read(lcs->adc_handle, lcs->channel_a, &adc_raw_a));
    ESP_ERROR_CHECK(adc_oneshot_read(lcs->adc_handle, lcs->channel_b, &adc_raw_b));

    lcs->adc_value_a = adc_raw_a;
    lcs->adc_value_b = adc_raw_b;

    lcs->voltage_a = (lcs->adc_value_a * 3.3) / 4095.0 - 1.65;
    lcs->voltage_b = (lcs->adc_value_b * 3.3) / 4095.0 - 1.65;
    lcs->voltage_c = -(lcs->voltage_a + lcs->voltage_b);
}

void lowside_current_sense_read_current(struct LowsideCurrentSense *lcs) {
    lowside_current_sense_read_voltage(lcs);
 
    lcs->current_a = (lcs->voltage_a - lcs->offset_a) * lcs->gain_a;
    lcs->current_b = (lcs->voltage_b - lcs->offset_b) * lcs->gain_b;
    lcs->current_c = (lcs->voltage_c - lcs->offset_c) * lcs->gain_c;
}

float lowside_current_sense_get_iq(struct LowsideCurrentSense *lcs, float angle) {
    float current_a = lcs->current_a;
    float current_b = lcs->current_b;
    float I_alpha = current_a;
    float I_beta = _1_SQRT3 * current_a + _2_SQRT3 * current_b;
    float ct = cos(angle);
    float st = sin(angle);
    float I_q = I_beta * ct - I_alpha * st;

    // 低通滤波
    if (lcs->lowpass_filter < 0.001f) {
        lcs->iq = I_q;
    } else {
        lcs->iq = lcs->iq * (1 - lcs->lowpass_filter) + I_q * lcs->lowpass_filter;
    }
    return lcs->iq;
}
// foc

void motor_set_pwm(struct Motor* motor, float ua, float ub, float uc) { 
    // printf("%f %f %f\n", ua, ub, uc);
    motor->dc_a = _constrain(ua / motor->voltage_power_supply, 0, 1);
    motor->dc_b = _constrain(ub / motor->voltage_power_supply, 0, 1);
    motor->dc_c = _constrain(uc / motor->voltage_power_supply, 0, 1);

    // printf("%f %f %f\n", motor->dc_a, motor->dc_b, motor->dc_c);
    ledc_set_duty(LEDC_LOW_SPEED_MODE, motor->channel1, motor->dc_a * 1023);
    ledc_update_duty(LEDC_LOW_SPEED_MODE, motor->channel1);
    
    ledc_set_duty(LEDC_LOW_SPEED_MODE, motor->channel2, motor->dc_b * 1023);
    ledc_update_duty(LEDC_LOW_SPEED_MODE, motor->channel2);

    ledc_set_duty(LEDC_LOW_SPEED_MODE, motor->channel3, motor->dc_c * 1023);
    ledc_update_duty(LEDC_LOW_SPEED_MODE, motor->channel3);
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

void setTorque(struct Motor* motor, float Uq, float angle_el) {
    Uq = _constrain(Uq, -motor->voltage_power_supply/2, motor->voltage_power_supply/2);
    // float Ud = 0;
    angle_el = _normalizeAngle(angle_el);
    // 帕克逆变换
    motor->Ualpha = -Uq*sin(angle_el); 
    motor->Ubeta = Uq*cos(angle_el); 

    // 克拉克逆变换
    motor->Ua = motor->Ualpha + motor->voltage_power_supply/2;
    motor->Ub = (sqrt(3)*motor->Ubeta-motor->Ualpha)/2 + motor->voltage_power_supply/2;
    motor->Uc = (-motor->Ualpha-sqrt(3)*motor->Ubeta)/2 + motor->voltage_power_supply/2;
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

float velocityClosedloop(struct Motor* motor, float target_velocity) {
    motor->sensor->update(motor->sensor);
    float velocity = motor->sensor->read_velocity(motor->sensor);
    float angle = motor->sensor->read_angle(motor->sensor);

    // 计算电流
    lowside_current_sense_read_current(motor->lowside_current_sense);
    float iq = lowside_current_sense_get_iq(motor->lowside_current_sense, _electricalAngle(motor));

    if (motor->cnt >= 4000) {
        printf("name: %s, velocity: %f, target velocity: %f, angle: %f\n",
                motor->name, velocity, target_velocity, angle);
        motor->cnt = 0;
        /*
        printf("name: %s, a: %f, b: %f, c: %f\n",
                motor->name,
                motor->lowside_current_sense->current_a,
                motor->lowside_current_sense->current_b,
                motor->lowside_current_sense->current_c);
                */
        printf("name :%s, iq: %f\n", motor->name, iq);
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

