#include "fft.h"


void pre_fft(float* real_values)
{
    for (uint16_t ind = 0; ind < (SAMPLES >> 1); ++ind) {
        float ratio = (float) ind / (SAMPLES - 1);
        float weighing_factor = 0.54 - (0.46 * cos( TWO_PI * ratio));

        real_values[ind] *= weighing_factor;
        real_values[SAMPLES - ind - 1] *= weighing_factor;
    }
}

uint16_t power_of_two(uint16_t number)
{
    return (uint16_t) floor(log(number) / log(2));
}

void swap(float* val_1, float* val_2)
{
    float temp = *val_1;
    *val_1 = *val_2;
    *val_2 = temp;
}

void compute_fft(float* real_values, float* imag_values)
{
    uint16_t j = 0;
    for (uint16_t i = 0; i < (SAMPLES - 1); ++i)
    {
        if (i < j)
        {
            swap(&real_values[i], &real_values[j]);
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
    uint16_t power = power_of_two(SAMPLES);

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

void compute_magnitude(float* real_values, float* imag_values)
{
    for (uint16_t ind = 0; ind < SAMPLES; ++ind)
    {
        real_values[ind] = pow((pow(real_values[ind], 2) + pow(imag_values[ind], 2)), 0.5);
    }
}

void find_peaks(float* magnitudes, float* peaks)
{
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

void normalize_and_transform(float* peaks, uint8_t* transformed)
{
    for (uint8_t ind = 0; ind < PEAKS_LENGTH; ++ind)
    {
        uint8_t power = power_of_two(peaks[ind]);
        uint8_t new_ind = ceil(power / 2);

        if (new_ind >= MATRIX_LENGTH)
        {
            new_ind = MATRIX_LENGTH - 1;
        }

        transformed[new_ind] += 1;
    }
}

void get_result(float* real_values, float* imag_values, uint8_t* results)
{
    pre_fft(real_values);
    compute_fft(real_values, imag_values);
    compute_magnitude(real_values, imag_values);

    float peaks[PEAKS_LENGTH];
    find_peaks(real_values, peaks);

    normalize_and_transform(peaks, results);
}

// power    index
// 0        : 0
// 1-2      : 1
// 3-4      : 2
// 5-6      : 3
// 7-8      : 4
// 9-10     : 5
// 11-12    : 6
// 13+      : 7
