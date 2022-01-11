#ifndef FFT_H
#define FFT_H

#include <math.h>
#include <stdint.h>

#define TWO_PI          6.28318531

#define SAMPLES         1024
#define SAMPLING_FREQ   41666
#define MATRIX_LENGTH   8
#define MAX_LED 		64

uint16_t __power_of_two(uint16_t number);

void __pre_fft(float* points_array);
void __compute_fft(float* real_values, float* imag_values);

float __calc_slice_peak(float* magnitudes, const uint16_t start, const uint16_t end);
float __calc_min(float* array);

void __compute_magnitudes(float* real_values, float* imag_values);
void __interpret_magnitudes(float* magnitudes, float* peaks);
void __transform_for_diods(float* values, uint8_t leds[MAX_LED]);

void perform_fft(float* real_values, uint8_t leds[MAX_LED]);

#endif
