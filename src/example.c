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
    uint8_t results[MATRIX_LENGTH] = {};                // Initialize array for future results

    get_result(real_values, imag_values, results);      // Save results to the array

    for (int i = 0; i < MATRIX_LENGTH; ++i)
    {
        printf("%d ", results[i]);
    }

    return 0;
}

