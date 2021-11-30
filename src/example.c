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
    read_data(time, real_values, SAMPLES);


    float imag_values[SAMPLES] = {};
    uint8_t results[MATRIX_LENGTH] = {};

    get_result(real_values, imag_values, results);

    for (int i = 0; i < MATRIX_LENGTH; ++i)
    {
        printf("%d ", results[i]);
    }

    return 0;
}

