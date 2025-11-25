#include "current_sense.h"
#include "lowside_current_sense.h"

#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <esp_log.h>
#include <esp_timer.h>
#include <math.h>

static char *TAG = "LCS";

#define _1_SQRT3 0.57735026919f
#define _2_SQRT3 1.15470053838f

adc_channel_t get_channel_by_pin(int pin);
adc_oneshot_unit_handle_t new_lowside_current_sense_adc_unit();
void lowside_current_sense_read_voltage(struct LowsideCurrentSense *lcs);

// export function
struct PhaseCurrent read_current(struct CurrentSense *lcs);
void init(struct CurrentSense *current_sense);
float get_foc_current(struct CurrentSense *current_sense, float angle);

adc_channel_t get_channel_by_pin(int pin) {
    ESP_ERROR_CHECK(!(pin >= 1 && pin <= 10));
    adc_channel_t table[11] = {0, ADC_CHANNEL_0, ADC_CHANNEL_1, ADC_CHANNEL_2, ADC_CHANNEL_3, ADC_CHANNEL_4, ADC_CHANNEL_5, ADC_CHANNEL_6, ADC_CHANNEL_7, ADC_CHANNEL_8, ADC_CHANNEL_9};
    return table[pin];
}

adc_oneshot_unit_handle_t new_lowside_current_sense_adc_unit() {
    static adc_oneshot_unit_handle_t adc_handle;
    static int init_flag = 0;
    if (init_flag) {
        return adc_handle;
    }
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

    init_flag = 1;
    return adc_handle;
}

struct CurrentSense* new_lowside_current_sense(float shunt_resistor, float gain, int pin_a, int pin_b, int pin_c) {
    struct LowsideCurrentSense *lowside_current_sense = emalloc(sizeof(struct LowsideCurrentSense));
    if (lowside_current_sense == NULL) {
        ESP_LOGE(TAG, "malloc memory failed!");
        return NULL;
    }

    adc_oneshot_chan_cfg_t config = {
        .atten = ADC_ATTEN_DB_12,   // 设置衰减，以调整测量电压范围
        .bitwidth = ADC_BITWIDTH_DEFAULT, // 设置输出位宽
    };
    adc_oneshot_unit_handle_t adc_handle = new_lowside_current_sense_adc_unit();
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
    // 低通滤波
    lowside_current_sense->foc_current = 0.0f;
    lowside_current_sense->foc_lowpass_filter = 0.002f;

    struct CurrentSense *current_sense = emalloc(sizeof(struct CurrentSense));
    current_sense->current_sense = lowside_current_sense;
    current_sense->read_current = read_current;
    current_sense->init = init;
    current_sense->get_foc_current = get_foc_current;
    return current_sense;
}

void lowside_current_sense_read_voltage(struct LowsideCurrentSense *lcs) {
    int adc_raw_a, adc_raw_b;
    ESP_ERROR_CHECK(adc_oneshot_read(lcs->adc_handle, lcs->channel_a, &adc_raw_a));
    ESP_ERROR_CHECK(adc_oneshot_read(lcs->adc_handle, lcs->channel_b, &adc_raw_b));

    lcs->adc_value_a = adc_raw_a;
    lcs->adc_value_b = adc_raw_b;

    // lcs->voltage_a = (lcs->adc_value_a * 3.3) / 4095.0 - 1.65;
    // lcs->voltage_b = (lcs->adc_value_b * 3.3) / 4095.0 - 1.65;
    lcs->voltage_a = (lcs->adc_value_a * 3.3) / 4095.0;
    lcs->voltage_b = (lcs->adc_value_b * 3.3) / 4095.0;

    lcs->voltage_c = -(lcs->voltage_a + lcs->voltage_b);
}

struct PhaseCurrent read_current(struct CurrentSense *current_sense) {
    lowside_current_sense_read_voltage(current_sense->current_sense);
    struct LowsideCurrentSense *lcs = current_sense->current_sense;

    lcs->current_a = (lcs->voltage_a - lcs->offset_a) * lcs->gain_a;
    lcs->current_b = (lcs->voltage_b - lcs->offset_b) * lcs->gain_b;
    lcs->current_c = (lcs->voltage_c - lcs->offset_c) * lcs->gain_c;

    struct PhaseCurrent current;
    current.a = lcs->current_a;
    current.b = lcs->current_b;
    current.c = lcs->current_c;
    return current;
}

void init(struct CurrentSense *current_sense) {
    struct LowsideCurrentSense *lcs = current_sense->current_sense;
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

float get_foc_current(struct CurrentSense *current_sense, float angle) {
    struct PhaseCurrent current = current_sense->read_current(current_sense);
    float I_alpha = current.a;
    float I_beta = _1_SQRT3 * current.a + _2_SQRT3 * current.b;
    float ct = cos(angle);
    float st = sin(angle);
    float I_q = I_beta * ct - I_alpha * st;

    // 低通滤波
    if (((struct LowsideCurrentSense*)current_sense->current_sense)->foc_lowpass_filter < 0.001f) {
        ((struct LowsideCurrentSense*)current_sense->current_sense)->foc_current = I_q;
    } else {
        float old_foc_current = ((struct LowsideCurrentSense*)current_sense->current_sense)->foc_current;
        float foc_lowpass_filter = ((struct LowsideCurrentSense*)current_sense->current_sense)->foc_lowpass_filter;
        ((struct LowsideCurrentSense*)current_sense->current_sense)->foc_current = old_foc_current * (1 - foc_lowpass_filter) + I_q * foc_lowpass_filter;
    }
    return ((struct LowsideCurrentSense*)current_sense->current_sense)->foc_current;
}