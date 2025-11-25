#ifndef _CURRENT_SENSE_H_
#define _CURRENT_SENSE_H_ 

#include "esp_adc/adc_oneshot.h"
#include "esp_adc/adc_cali.h"
#include "esp_adc/adc_cali_scheme.h"

#define emalloc(size) malloc(size)
struct PhaseCurrent {
    float a;
    float b;
    float c;
};

struct CurrentSense {
    void *current_sense;

    void (*init)(struct CurrentSense *current_sense);
    struct PhaseCurrent (*read_current)(struct CurrentSense *current_sense);
    float (*get_foc_current)(struct CurrentSense *current_sense, float angle);
};

#endif