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

#define AS5600_ADDRESS 0x36
// 原始角度值
#define AS5600_REG_RAW_ANGLE 0x0C
// 滤波后的角度值
#define AS5600_REG_ANGLE 0x0E
// 状态寄存器
#define AS5600_REG_STATUS 0x0B
// 配置寄存器
#define AS5600_REG_CONFIG 0x09

// SVPWM
#define _SQRT3 1.73205080757f
#define _SQRT3_2 0.86602540378f
#define _1_SQRT3 0.57735026919f
#define _2_SQRT3 1.15470053838f
#define _PI 3.14159265359f
#define _PI_2 1.57079632679f
#define _PI_3 1.0471975512f
#define _3PI_2 4.71238898038f

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

    // set pole pair number
    motor->pp = pp;

    // init motor param
    motor->velocity = 0.0;
    motor->integral = 0.0;
    motor->prev_dvelocity = 0.0;
    motor->target_velocity = 0.0;

    // encoder param
    motor->angle_without_track = 0.0;
    motor->prev_angle = 0.0;
    motor->full_ratations = 0.0;

    // speed param
    motor->prev_us = 0.0;
    motor->prev_angle = 0.0;

    return motor;
}

i2c_master_dev_handle_t foc_motor_i2c_init(gpio_num_t pin_sda, gpio_num_t pin_scl, i2c_port_t port) {
    i2c_master_dev_handle_t i2c_dev_handle;
    i2c_master_bus_config_t bus_cfg = {
        .clk_source = I2C_CLK_SRC_DEFAULT,
        .i2c_port = port,
        .scl_io_num = pin_scl,
        .sda_io_num = pin_sda,
        .glitch_ignore_cnt = 7,
        .flags.enable_internal_pullup = true,
    };
    i2c_master_bus_handle_t bus_handle;
    ESP_ERROR_CHECK(i2c_new_master_bus(&bus_cfg, &bus_handle));
    i2c_device_config_t dev_cfg = {
        .dev_addr_length = I2C_ADDR_BIT_LEN_7,
        .device_address = AS5600_ADDRESS,
        .scl_speed_hz = 800*1000,
    };
    ESP_ERROR_CHECK(i2c_master_bus_add_device(bus_handle, &dev_cfg, &i2c_dev_handle));
    gpio_set_level((gpio_num_t)pin_sda, 1);
    gpio_set_level((gpio_num_t)pin_scl, 1);

    uint8_t status = 0;
    ESP_ERROR_CHECK(as5600_i2c_read_reg(i2c_dev_handle, AS5600_REG_STATUS, &status));

    ESP_LOGI(TAG, "I2C bus initialized");
    return i2c_dev_handle;
    /*
    for (int i = 0; i < 128; i += 16) {
        for (int j = 0; j < 16; j++) {
            int addr = i + j;
            if (addr <= 0x03 || addr > 0x77) {
                continue;
            }
            i2c_device_config_t dev_cfg = {
                .dev_addr_length = I2C_ADDR_BIT_LEN_7,
                .device_address = addr,
                .scl_speed_hz = 400*1000,
            };
            i2c_master_dev_handle_t dev_handle;
            esp_err_t ret = i2c_master_bus_add_device(bus_handle, &dev_cfg, &dev_handle);
            if (ret == ESP_OK) {
                uint8_t status = 0;
                ret = as5600_i2c_read_reg(AS5600_REG_STATUS, &status);
                if (ret == ESP_OK) {
                    printf("%02x: %02x\n", addr, status);
                }
            }
            i2c_master_bus_rm_device(dev_handle);
        }
        printf("\n");
    }
    */
}

esp_err_t as5600_i2c_read_bytes(i2c_master_dev_handle_t i2c_dev_handle, uint8_t reg, uint8_t *data, size_t len)
{
    esp_err_t ret = i2c_master_transmit(i2c_dev_handle, &reg, 1, -1);
    if (ret != ESP_OK) {
        return ret;
    }
    return i2c_master_receive(i2c_dev_handle, data, len, -1);
}

esp_err_t as5600_i2c_read_reg(i2c_master_dev_handle_t i2c_dev_handle, uint8_t reg, uint8_t *data)
{
    esp_err_t ret = i2c_master_transmit(i2c_dev_handle, &reg, 1, -1);
    if (ret != ESP_OK) {
        return ret;
    }
    return i2c_master_receive(i2c_dev_handle, data, 1, -1);
}

esp_err_t as5600_i2c_write_bytes(i2c_master_dev_handle_t i2c_dev_handle, uint8_t reg, uint8_t *data, size_t length)
{
    esp_err_t ret = i2c_master_transmit(i2c_dev_handle, &reg, 1, -1);
    if (ret != ESP_OK) {
        return ret;
    }
    ret = i2c_master_transmit(i2c_dev_handle, data, length, -1);
    
    return ret;
}

esp_err_t as5600_i2c_write_reg(i2c_master_dev_handle_t i2c_dev_handle, uint8_t reg, uint8_t value)
{
    uint8_t cmd[2] = {reg, value};
    return i2c_master_transmit(i2c_dev_handle, cmd, 2, -1);
}

esp_err_t as5600_read_raw_angle(struct Motor* motor, uint16_t *angle) {
    uint8_t data[2] = {0};
    esp_err_t ret = as5600_i2c_read_bytes(motor->i2c_dev_handle, AS5600_REG_RAW_ANGLE, data, 2);
    if (ret != ESP_OK) {
        return ret;
    }
    *angle = ((data[0] << 8) | data[1]) & 0x0FFF;;
    return ESP_OK;
}

esp_err_t as5600_read_angle_without_track(struct Motor* motor, float *angle) {
    uint16_t raw_angle = 0;
    esp_err_t ret = as5600_read_raw_angle(motor, &raw_angle);
    if (ret != ESP_OK) {
        return ret;
    }
    // 0.087890625其实就是360/4096(as5600编码器的角度范围是0-4096，所以要先转换成角度值，然后再转换成弧度)
    *angle = (float)raw_angle * 0.087890625 * M_PI / 180;
    return ESP_OK;
}

esp_err_t as5600_read_angle(struct Motor* motor, float *angle) {
    esp_err_t ret = as5600_read_angle_without_track(motor, &motor->angle_without_track);
    if (ret != ESP_OK) {
        return ret;
    }
    float d_angle = motor->angle_without_track - motor->prev_angle_without_track;
    // 判断圈数是否超过80%的一圈
    if (fabs(d_angle) > (0.8f * 2 * M_PI)) {
        motor->full_ratations += (d_angle > 0 ? -1 : 1);
    }
    motor->prev_angle_without_track = motor->angle_without_track;
    *angle = (float) motor->full_ratations * 2 * M_PI + motor->angle_without_track;
    return ESP_OK;
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
    // setTorque(motor, 3, _3PI_2);
    setTorqueSVPWM(motor, 2.0, _3PI_2);
    vTaskDelay(pdMS_TO_TICKS(1000));
    float angle = 0;
    as5600_read_angle(motor, &angle);
    // setTorqueSVPWM(motor, 0.0, _3PI_2);
    setTorqueSVPWM(motor, 0.0, 0.0);
    vTaskDelay(pdMS_TO_TICKS(500));
    motor->zero_electric_angle = 0;
    motor->zero_electric_angle = _electricalAngle(motor);

    // 把这个zero_electric_angle设置成-0.2转速会更大，可以到1800rpm
    // motor->zero_electric_angle = 0;
    printf("zero angle: %f\n", motor->zero_electric_angle);
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

struct LowsideCurrentSense* new_lowside_current_sense(adc_oneshot_unit_handle_t adc_handle, adc_channel_t channel1, adc_channel_t channel2) {
    struct LowsideCurrentSense *lowside_current_sense = malloc(sizeof(struct LowsideCurrentSense));
    if (lowside_current_sense == NULL) {
        ESP_LOGE(TAG, "malloc memory failed!");
        return NULL;
    }

    adc_oneshot_chan_cfg_t config = {
        .atten = ADC_ATTEN_DB_12,   // 设置衰减，以调整测量电压范围
        .bitwidth = ADC_BITWIDTH_DEFAULT, // 设置输出位宽
    };
    ESP_ERROR_CHECK(adc_oneshot_config_channel(adc_handle, channel1, &config)); // 此处以配置通道2为例
    ESP_ERROR_CHECK(adc_oneshot_config_channel(adc_handle, channel2, &config)); // 此处以配置通道2为例
    lowside_current_sense->adc_handle = adc_handle;
    lowside_current_sense->channel_a = channel1;
    lowside_current_sense->channel_b = channel2;

    return lowside_current_sense;
}

void lowside_current_sense_read(struct LowsideCurrentSense *lcs) {
    int adc_raw_a, adc_raw_b;
    ESP_ERROR_CHECK(adc_oneshot_read(lcs->adc_handle, lcs->channel_a, &adc_raw_a));
    ESP_ERROR_CHECK(adc_oneshot_read(lcs->adc_handle, lcs->channel_b, &adc_raw_b));
    lcs->adc_value_a = (adc_raw_a * 3.3) / 4095.0 - 1.65;
    lcs->adc_value_b = (adc_raw_b * 3.3) / 4095.0 - 1.65;
    lcs->adc_value_c = -(lcs->adc_value_a + lcs->adc_value_b);
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
    float angle = motor->angle_without_track;
    return _normalizeAngle(motor->pp * angle) - motor->zero_electric_angle;
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
    float angle = 0;
    as5600_read_angle(motor, &angle);
    motor->shaft_angle = motor->angle_without_track;

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
    float angle = 0;
    as5600_read_angle(motor, &angle);
    motor->shaft_angle = motor->angle_without_track;

    // 获取速度
    int64_t cur_us = esp_timer_get_time();
    float dt = (float)(cur_us - motor->prev_us) * 1e-6;
    float dangle = (angle - motor->prev_angle);
    float cur_velocity = dangle / dt;
    motor->prev_us = cur_us;
    motor->prev_angle = angle;

    // 使用低通滤波算法获取速度
    float TF = 0.009;
    float alpha = TF / (TF + dt);
    motor->velocity = motor->velocity * alpha + cur_velocity * (1 - alpha);

    if (motor->cnt >= 4000) {
        // printf("name: %s, velocity: %f, target velocity: %f\n",
        //        motor->name, motor->velocity, target_velocity);
        motor->cnt = 0;
        lowside_current_sense_read(motor->lowside_current_sense);
        /*
        printf("name: %s, a: %f, b: %f, c: %f\n",
                motor->name,
                motor->lowside_current_sense->adc_value_a,
                motor->lowside_current_sense->adc_value_b,
                motor->lowside_current_sense->adc_value_c);
        */
    }
    motor->cnt++;

    // PI控制
    float dvelocity = target_velocity - motor->velocity;
    float kp = 0.003, ki = 0.002;
    // static float integral = 0;
    // static float prev_dvelocity = 0;
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
