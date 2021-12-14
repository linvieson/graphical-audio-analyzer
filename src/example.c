#include <stdio.h>
#include <stdint.h>
#include "operate_data.h"
#include "fft.h"

#define SAMPLES         1024
#define SAMPLING_FREQ   40000
#define MATRIX_LENGTH   8


int main() {
    float time[SAMPLES] = {};
    float real_values[SAMPLES] = {};
    read_data(time, real_values, SAMPLES);              // Save values of points to the array of real values


    float imag_values[SAMPLES] = {};                    // Initialize array for imaginary numbers

    float peaks[MATRIX_LENGTH] = {};
    float means[MATRIX_LENGTH] = {};
    get_result(real_values, imag_values, peaks, means);      // Save results to the array

    printf("\nPeaks\n");
    for (int i = 0; i < MATRIX_LENGTH; ++i)
    {
        printf("%f ", peaks[i]);
    }

    printf("\nMeans\n");

    for (int i = 0; i < MATRIX_LENGTH; ++i)
    {
        printf("%f ", means[i]);
    }


    uint8_t res_peaks[MATRIX_LENGTH];
    transform_for_diods(peaks, res_peaks);

    uint8_t res_means[MATRIX_LENGTH];
    transform_for_diods(means, res_means);

    printf("\nPeaks\n");

    for (int i = 0; i < MATRIX_LENGTH; ++i)
    {
        printf("%d ", res_peaks[i]);
    }

    printf("\nMeans\n");

    for (int i = 0; i < MATRIX_LENGTH; ++i)
    {
        printf("%d ", res_means[i]);
    }

    return 0;
}

