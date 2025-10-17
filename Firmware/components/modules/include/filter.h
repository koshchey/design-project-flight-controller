#ifndef FILTER_H_
#define FILTER_H_

#include "filter.h"
#include <math.h>

typedef struct {
    float b0, b1, b2, a1, a2, delayed1, delayed2; 
} lpf_data;

void init_lpf(lpf_data *lpf_data, float fs, float fr);
float apply_lpf(lpf_data *lpf_data, float sample);

#endif