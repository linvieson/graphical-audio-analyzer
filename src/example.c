#include <stdio.h>
#include <stdint.h>
#include "operate_data.h"
#include "fft.h"
#include "leds_matrix.h"

#define SAMPLES         1024
#define SAMPLING_FREQ   40000
#define MATRIX_LENGTH   8


int main() {
    float time[SAMPLES] = {};
    float real_values[SAMPLES] = {};
    read_data(time, real_values, SAMPLES);              // Save values of points to the array of real values

    float imag_values[SAMPLES] = {};                    // Initialize array for imaginary numbers

    float peaks[MATRIX_LENGTH] = {};
    get_result(real_values, imag_values, peaks);      // Save results to the array

    uint8_t res_peaks[MATRIX_LENGTH][MATRIX_LENGTH];
    transform_for_diods(peaks, res_peaks);

    printf("\nPeaks\n");

    for (int i = 0; i < MATRIX_LENGTH; ++i) {
        for (int j = 0; j < MATRIX_LENGTH; ++j) {
            {
                printf("%d ", res_peaks[i][j]);
            }
        }
        printf("\n");
    }

//    printf("\n\n");

//    uint8_t matrix[MATRIX_LENGTH][MATRIX_LENGTH][3];
//    transform_to_matrix(res_peaks, matrix);


    return 0;
}

