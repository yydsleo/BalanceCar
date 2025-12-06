#include "current_sense.h"
#include "lowside_current_sense.h"

#include <soc/soc.h>
#include <register/soc/sens_reg.h>
#include <../src/mcpwm_private.h>
#include <driver/mcpwm_prelude.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <esp_log.h>
#include <esp_timer.h>
#include <math.h>
#include <string.h>

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

float IRAM_ATTR adcRead2(uint8_t channel) {
    uint16_t value = 0;

    return (float)value;
}

float IRAM_ATTR adcRead(uint8_t channel)
{
    CLEAR_PERI_REG_MASK(SENS_SAR_MEAS1_CTRL2_REG, SENS_MEAS1_START_SAR_M);
    SET_PERI_REG_BITS(SENS_SAR_MEAS1_CTRL2_REG, SENS_SAR1_EN_PAD, (1 << channel), SENS_SAR1_EN_PAD_S);
    SET_PERI_REG_MASK(SENS_SAR_MEAS1_CTRL2_REG, SENS_MEAS1_START_SAR_M);
    uint16_t value = 0;

    //wait for conversion
    while (GET_PERI_REG_MASK(SENS_SAR_MEAS1_CTRL2_REG, SENS_MEAS1_DONE_SAR) == 0);
    // read teh value
    value = GET_PERI_REG_BITS2(SENS_SAR_MEAS1_CTRL2_REG, SENS_MEAS1_DATA_SAR, SENS_MEAS1_DATA_SAR_S);
    return (float)value;
}

uint64_t prev_us = 0;
uint64_t cnt = 0;
// 电流采样回调函数（中断上下文内，需遵循IRAM_ATTR等规则）
static bool IRAM_ATTR on_timer_event_cb(mcpwm_timer_handle_t timer, const mcpwm_timer_event_data_t *edata, void *user_data) {
    // 检查是否是定时器达到峰值（TEP）的事件
    struct LowsideCurrentSense *current_sense = (struct LowsideCurrentSense *)user_data;

    cnt++;
    if (++current_sense->adc_task_cnt < 25000) {
        // return true;
    }
    current_sense->adc_task_cnt = 0;

    current_sense->adc_value_a = adcRead(3);
    current_sense->adc_value_b = adcRead(4);
    /*
    BaseType_t xHigherPriorityTaskWoken = pdFALSE;
    vTaskNotifyGiveFromISR(current_sense->adc_task_handle, &xHigherPriorityTaskWoken);
    if (xHigherPriorityTaskWoken == pdTRUE) {
        portYIELD_FROM_ISR();
    }
    */
    return true; // 返回值含义需查阅文档，通常false表示未占用更高优先级任务
}

void adc_task(void *params) {
    struct LowsideCurrentSense *current_sense = (struct LowsideCurrentSense *)params;
    while (true) {
        ulTaskNotifyTake(pdTRUE, portMAX_DELAY);
        lowside_current_sense_read_voltage(current_sense);
        printf("cnt: %lld\n", cnt);
        cnt = 0;
        // vTaskDelay(pdMS_TO_TICKS(1));
    }
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

struct CurrentSense* new_lowside_current_sense(float shunt_resistor, float gain, int pin_a, int pin_b, int pin_c, mcpwm_timer_handle_t timer) {
    struct LowsideCurrentSense *lowside_current_sense = emalloc(sizeof(struct LowsideCurrentSense));
    if (lowside_current_sense == NULL) {
        ESP_LOGE(TAG, "malloc memory failed!");
        return NULL;
    }
    memset(lowside_current_sense, 0, sizeof(struct LowsideCurrentSense));

    adc_oneshot_chan_cfg_t config = {
        .atten = ADC_ATTEN_DB_11,   // 设置衰减，以调整测量电压范围
        .bitwidth = ADC_BITWIDTH_12, // 设置输出位宽
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
    lowside_current_sense->foc_lowpass_filter = 0.0f;

    lowside_current_sense->adc_task_cnt = 0;
    // 创建电流采样的task
    char buf[32];
    snprintf(buf, sizeof(buf), "adc_task_%d", rand() % 100);
    xTaskCreatePinnedToCore(&adc_task, buf, 4 * 1024, lowside_current_sense, 5, &lowside_current_sense->adc_task_handle, 1);
    // 注册电流传感器的回调
    mcpwm_timer_event_callbacks_t cbs = {
        .on_full = on_timer_event_cb,
        .on_empty = NULL,
        .on_stop = NULL,
    };
    struct mcpwm_timer_t *t = (struct mcpwm_timer_t*)timer;
    t->fsm = MCPWM_TIMER_FSM_INIT;
    mcpwm_timer_register_event_callbacks(t, &cbs, lowside_current_sense);
    t->fsm = MCPWM_TIMER_FSM_ENABLE;
    esp_intr_enable(t->intr);


    struct CurrentSense *current_sense = emalloc(sizeof(struct CurrentSense));
    current_sense->current_sense = lowside_current_sense;
    current_sense->read_current = read_current;
    current_sense->init = init;
    current_sense->get_foc_current = get_foc_current;
    vTaskDelay(pdMS_TO_TICKS(100));
    return current_sense;
}

void lowside_current_sense_read_voltage(struct LowsideCurrentSense *lcs) {
    int adc_raw_a = 0, adc_raw_b = 0;
    // ESP_ERROR_CHECK(adc_oneshot_read(lcs->adc_handle, lcs->channel_a, &adc_raw_a));
    // ESP_ERROR_CHECK(adc_oneshot_read(lcs->adc_handle, lcs->channel_b, &adc_raw_b));
    esp_err_t ret = adc_oneshot_read(lcs->adc_handle, lcs->channel_a, &adc_raw_a);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "adc_oneshot_read a failed!");
        return;
    }
    ret = adc_oneshot_read(lcs->adc_handle, lcs->channel_b, &adc_raw_b);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "adc_oneshot_read b failed!");
        return;
    }
    lcs->adc_value_a = adc_raw_a;
    lcs->adc_value_b = adc_raw_b;
}

struct PhaseCurrent read_current(struct CurrentSense *current_sense) {
    struct LowsideCurrentSense *lcs = current_sense->current_sense;
    // lowside_current_sense_read_voltage(current_sense->current_sense);
    lcs->voltage_a = (lcs->adc_value_a * 3.3) / 4095.0;
    lcs->voltage_b = (lcs->adc_value_b * 3.3) / 4095.0;
    // lcs->voltage_c = -(lcs->voltage_a + lcs->voltage_b);
    lcs->voltage_c = 0;

    lcs->current_a = (lcs->voltage_a - lcs->offset_a) * lcs->gain_a * -1;
    lcs->current_b = (lcs->voltage_b - lcs->offset_b) * lcs->gain_b * -1;
    lcs->current_c = (lcs->voltage_c - lcs->offset_c) * lcs->gain_c * -1;

    struct PhaseCurrent current;
    current.a = lcs->current_a;
    current.b = lcs->current_b;
    current.c = lcs->current_c;
    return current;
}

void init(struct CurrentSense *current_sense) {
    struct LowsideCurrentSense *lcs = current_sense->current_sense;
    int cnt = 2000;
    float offset_ia = 0;
    float offset_ib = 0;
    float offset_ic = 0;
    for (int i = 0; i < cnt; i++) {
        // lowside_current_sense_read_voltage(lcs);
        offset_ia += lcs->voltage_a;
        offset_ib += lcs->voltage_b;
        offset_ic += lcs->voltage_c;
        vTaskDelay(pdMS_TO_TICKS(1));
    }
    lcs->offset_a = offset_ia / cnt;
    lcs->offset_b = offset_ib / cnt;
    lcs->offset_c = offset_ic / cnt;
    printf("lcs offset_a: %f, offset_b: %f, offset_c: %f\n", lcs->offset_a, lcs->offset_b, lcs->offset_c);
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