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

uint16_t power_of_two(uint16_t number);

void pre_fft(float* points_array);
void compute_fft(float* real_values, float* imag_values);

void compute_magnitude(float* real_values, float* imag_values);
void find_peaks(float* magnitudes, float* peaks);

void normalize_and_transform(float* peaks, uint8_t* transformed);

void get_result(float* real_values, float* imag_values, uint8_t* results);


#endif