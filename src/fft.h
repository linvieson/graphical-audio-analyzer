#ifndef FFT_H
#define FFT_H

#include <math.h>
#include <stdint.h>


#define TWO_PI 6.28318531

#define SAMPLES         1024
#define SAMPLING_FREQ   40000
#define MATRIX_LENGTH   8

#define STEP            (uint8_t) (ceil((float) SAMPLING_FREQ / SAMPLES))
#define PEAKS_LENGTH    (uint8_t) (ceil((float) SAMPLES / STEP))

uint16_t __power_of_two(uint16_t number);

void __pre_fft(float* points_array);
void __compute_fft(float* real_values, float* imag_values);

void __compute_magnitude(float* real_values, float* imag_values);
float __calc_slice_peak(float* magnitudes, const uint16_t start, const uint16_t end);
float __calc_slice_mean(float* magnitudes, const uint16_t start, const uint16_t end);
void __interpret_magnitudes(float* magnitudes, float* peaks);

void get_result(float* real_values, float* imag_values, float* peaks);

float __calc_min(float* array);
void transform_for_diods(float* values, uint8_t matrix[MATRIX_LENGTH][MATRIX_LENGTH]);


#endif