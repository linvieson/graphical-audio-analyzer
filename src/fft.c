#include "fft.h"


void __pre_fft(float* real_values)
// Scale (window) the initial values of the points to reduce the spectral leakage.
// Use Hamming Window with scaling factor (coherent gain) = 0.54
{
    // Values are symmetric, thus iterate only on the first half of the array
    for (uint16_t ind = 0; ind < (SAMPLES >> 1); ++ind) {
        float ratio = (float) ind / (SAMPLES - 1);
        float weighing_factor = 0.54 - (0.46 * cos( TWO_PI * ratio));

        real_values[ind] *= weighing_factor;
        real_values[SAMPLES - ind - 1] *= weighing_factor;      // symmetrical to the real_values[ind]
    }
}

uint16_t __power_of_two(uint16_t number)
// Get the power of 2, which is a bottom bound of the given number.
{
    return (uint16_t) floor(log(number) / log(2));
}

void __swap(float* val_1, float* val_2)
// Swap two points values given their addresses.
{
    float temp = *val_1;
    *val_1 = *val_2;
    *val_2 = temp;
}

void __compute_fft(float* real_values, float* imag_values)
// Compute the in-place Fast Furrier transform.
{
    uint16_t j = 0;
    for (uint16_t i = 0; i < (SAMPLES - 1); ++i)
    {
        if (i < j)
        {
            __swap(&real_values[i], &real_values[j]);
        }

        uint16_t half = SAMPLES >> 1;
        while (half <= j) {
            j -= half;
            half >>= 1;
        }
        j += half;
    }

    float c1 = -1.0;
    float c2 = 0.0;
    uint16_t l2 = 1;
    uint16_t power = __power_of_two(SAMPLES);

    for (uint8_t l = 0; l < power; l++)
    {
        uint16_t l1 = l2;
        l2 <<= 1;
        float u1 = 1.0;
        float u2 = 0.0;

        for (j = 0; j < l1; j++)
        {
            for (uint16_t i = j; i < SAMPLES; i += l2)
            {
                uint16_t i1 = i + l1;
                float t1 = u1 * real_values[i1] - u2 * imag_values[i1];
                float t2 = u1 * imag_values[i1] + u2 * real_values[i1];

                real_values[i1] = real_values[i] - t1;
                imag_values[i1] = imag_values[i] - t2;
                real_values[i] += t1;
                imag_values[i] += t2;
            }
            float z = ((u1 * c1) - (u2 * c2));
            u2 = ((u1 * c2) + (u2 * c1));
            u1 = z;
        }
        c2 = - pow((1.0 - c1) / 2.0, 0.5);
        c1 = pow((1.0 + c1) / 2.0, 0.5);
    }
}

void __compute_magnitude(float* real_values, float* imag_values)
// Compute in-place the absolute values of the received numbers from the FFT.
// They are stored in the array real_values.
{
    for (uint16_t ind = 0; ind < SAMPLES; ++ind)
    {
        // |z| = sqrt(a^2 + b^2)
        real_values[ind] = pow((pow(real_values[ind], 2) + pow(imag_values[ind], 2)), 0.5);
    }
}

void __find_peaks(float* magnitudes, float* peaks)
// Find the maximum value in each magnitude array slice with a STEP numbers.
// STEP = SAMPLING_FREQ / SAMPLES.
// Save the peaks into array peaks with length PEAKS_LENGTH.
// PEAKS_LENGTH = SAMPLES / STEP.
{
    // Ignore the first number in fft.
    magnitudes[0] = 0;

    for (uint8_t peak_ind = 0; peak_ind < PEAKS_LENGTH; ++peak_ind)
    {
        float peak = 0;
        for (uint16_t point_ind = peak_ind * STEP; point_ind < (peak_ind + 1) * STEP; ++point_ind)
        {
            if (magnitudes[point_ind] > peak)
            {
                peak = magnitudes[point_ind];
            }
        }
        peaks[peak_ind] = peak;
    }
}

void __normalize_and_transform(float* peaks, uint8_t* transformed)
// Reduce the peak`s magnitude to the relative numbers, count the number of their occurrences and
// save them to the array transformed.

// Represent each peak`s magnitude as power of 2, which bounds it in the bottom.
// Then save the number of their occurrences in the peaks array according to the following representation:

// power    index
// 0        : 0
// 1-2      : 1
// 3-4      : 2
// 5-6      : 3
// 7-8      : 4
// 9-10     : 5
// 11-12    : 6
// 13+      : 7

{
    for (uint8_t ind = 0; ind < PEAKS_LENGTH; ++ind)
    {
        uint8_t power = __power_of_two(peaks[ind]);
        uint8_t new_ind = ceil(power >> 1);

        if (new_ind >= MATRIX_LENGTH)
        {
            new_ind = MATRIX_LENGTH - 1;
        }

        transformed[new_ind] += 1;
    }
}

void get_result(float* real_values, float* imag_values, uint8_t* results)
// Driver function to get the array of relative magnitudes.
{
    __pre_fft(real_values);
    __compute_fft(real_values, imag_values);
    __compute_magnitude(real_values, imag_values);

    float peaks[PEAKS_LENGTH];
    __find_peaks(real_values, peaks);

    __normalize_and_transform(peaks, results);
}
