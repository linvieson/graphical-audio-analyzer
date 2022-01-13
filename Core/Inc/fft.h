#ifndef FFT_H
#define FFT_H

#include <math.h>
#include <stdint.h>

#define TWO_PI          6.28318531

#define SAMPLES         2048
#define SAMPLING_FREQ   36290
#define MATRIX_LENGTH   8
#define MAX_LED 		64

uint16_t __power_of_two(uint16_t number);

void __pre_fft(float real_values[SAMPLES]);
void __compute_fft(float real_values[SAMPLES], float imag_values[SAMPLES]);

float __calc_slice_peak(float* magnitudes, const uint16_t start, const uint16_t end);
float __calc_min(float* array);

void __compute_magnitudes(float real_values[SAMPLES], float imag_values[SAMPLES]);
void __interpret_magnitudes(float magnitudes[SAMPLES], float peaks[MATRIX_LENGTH]);
void __transform_for_diods(float values[MATRIX_LENGTH], uint8_t leds[MAX_LED]);

void perform_fft(float real_values[SAMPLES], uint8_t leds[MAX_LED]);

#endif
