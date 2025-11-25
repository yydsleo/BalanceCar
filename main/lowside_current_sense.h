#ifndef _LOWSIDE_CURRENT_SENSE_H_
#define _LOWSIDE_CURRENT_SENSE_H_ 

#include "esp_adc/adc_oneshot.h"
#include "esp_adc/adc_cali.h"
#include "esp_adc/adc_cali_scheme.h"

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

    float offset_a;
    float offset_b;
    float offset_c;

    float gain_a;
    float gain_b;
    float gain_c;
    float shunt_resistor;
    float gain;

    float foc_current;
    float foc_lowpass_filter;

};
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
struct CurrentSense* new_lowside_current_sense(float shunt_resistor, float gain, int pin_a, int pin_b, int pin_c);

#endif