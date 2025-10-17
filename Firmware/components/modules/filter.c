#include "filter.h"
#include <math.h>

/* Initiliase low-pass filter values */
void init_lpf(lpf_data *lpf_data, float fs, float fc) {
    /* Ref: CrazyFlieCore, Filter.c */
    /* Ref: https://www.earlevel.com/main/2003/03/02/the-bilinear-z-transform/ */
    // ohm = tan(pi*Fc/Fs) = K
    // c = K^2 + K/Q + 1, Q = 0.707 (second order butterworth)
    // i.e. 1/Q = 1.414

    float fr = fs/fc; // sampling frequency/cutoff frequency
    float ohm = tanf((float)M_PI/fr);
    float c = 1.0f+2.0f*cosf((float)M_PI/4.0f)*ohm+ohm*ohm; 
    lpf_data->b0 = ohm*ohm/c;
    lpf_data->b1 = 2.0f*lpf_data->b0;
    lpf_data->b2 = lpf_data->b0;
    lpf_data->a1 = 2.0f*(ohm*ohm-1.0f)/c;
    lpf_data->a2 = (1.0f-2.0f*cosf((float)M_PI/4.0f)*ohm+ohm*ohm)/c;
    lpf_data->delayed1 = .0f;
    lpf_data->delayed2 = .0f;
}

/* Apply low-pass filter */
float apply_lpf(lpf_data *lpf_data, float sample) {
    /* Ref: https://en.wikipedia.org/wiki/Digital_biquad_filter */

    //        b0 + b1 z^-1 + b2 z^-2 
    // H(z) = ---------------------- 
    //        1 + a1 z^-1 + a2 z^-2  

    /* Direct form 2 implementation: */
    // w(n) = x(n) - a1 w(n-1) - a2 w(n-2) 
    // y(n) = b0 w(n) + b1 w(n-1) + b3 w(n-2) 
    
    float delayed0 = sample - (lpf_data->delayed1 * lpf_data->a1) - (lpf_data->delayed2 * lpf_data->a2);
    
    if (!isfinite(delayed0)) {
        delayed0 = sample;
    }
        
    float output = (delayed0 * lpf_data->b0) + (lpf_data->delayed1 * lpf_data->b1) + (lpf_data->delayed2 * lpf_data->b2);

    lpf_data->delayed2 = lpf_data->delayed1;
    lpf_data->delayed1 = delayed0;

    return output;
}
