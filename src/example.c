#include <stddef.h>
#include <complex.h>
#include <math.h>
#include <stdio.h>
#include "fft.h"
#include "read_data.h"

#define NUM_ROWS 2000   // only 1024 will be used


int main() {
    double time[NUM_ROWS] = {};
    double values[NUM_ROWS] = {};
    read_data(time, values, NUM_ROWS);
    
    size_t length = pow(2, floor(log(NUM_ROWS)/log(2)));
    float complex vector[length];

    for(size_t index = 0; index < length; index++) {
        vector[index] = values[index];
    }

    printf("\nin time domain:\n");
    for(size_t index = 0; index < length; index++) {
        printf("%f + %f i\n", creal(vector[index]), cimag(vector[index]));
    }

    fft(vector, length);

    printf("\nin frequency domain:\n");
    for(size_t index = 0; index < length; index++) {
        printf("%f + %f i\n", creal(vector[index]), cimag(vector[index]));
    }

    return 0;
}
