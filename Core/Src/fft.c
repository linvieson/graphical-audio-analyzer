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

void __interpret_magnitudes(float* magnitudes, float* peaks)
// STEP = SAMPLING_FREQ / SAMPLES.
// Divide the fist half of the array of magnitudes on the MATRIX_LENGTH parts,
// with each next slice being larger than the previous one.
// For each slice calculate the peak and the average value, and store them into
// arrays peaks and means respectively.

// The slice boundaries are the number 2 raised to a power, starting from 1 (2 ^ 1 = 2), and ending with
// MATRIX_LENGTH (2 ^ 8 = 512). Thus, the slices boundaries are: (2, 4); (4, 8); (8, 16); ...; (256, 512).
{
    uint16_t slices[] = {8, 15, 25, 75, 125, 200, 300, 512};

    uint16_t upper_bound;
    uint16_t lower_bound = 3;

    for (uint8_t ind = 0; ind < MATRIX_LENGTH; ++ind) {
        upper_bound = slices[ind];
        peaks[ind] = __calc_slice_peak(magnitudes, lower_bound, upper_bound);
        lower_bound = upper_bound;
    }
}

float __calc_slice_peak(float* magnitudes, const uint16_t start, const uint16_t end)
// Find the max value of the array slice with the given indices.
{
    float max_val = 0;
    for (uint16_t ind = start; ind < end; ++ind)
    {
        if (magnitudes[ind] > max_val)
        {
            max_val = magnitudes[ind];
        }
    }
    return max_val;
}

float __calc_slice_mean(float* magnitudes, const uint16_t start, const uint16_t end)
// Find the average of the array slice with the given indices.
{
    float mean = 0;
    for (uint16_t ind = start; ind < end; ++ind)
    {
        mean += magnitudes[ind];
    }
    return mean / (end - start);
}

float __calc_min(float* array)
// Find the min value of the array.
{
    float min_val = array[0];
    for (uint16_t ind = 1; ind < MATRIX_LENGTH; ++ind)
    {
        if (array[ind] < min_val)
        {
            min_val = array[ind];
        }
    }
    return min_val;
}

void get_result(float* real_values, uint8_t matrix[MATRIX_LENGTH][MATRIX_LENGTH])
// Driver function to compute the peaks and the average values, which will be stored in the arrays peaks and means respectively.
{
    __pre_fft(real_values);

    float imag_values[SAMPLES] = {};
    float peaks[MATRIX_LENGTH] = {};

    __compute_fft(real_values, imag_values);
    __compute_magnitude(real_values, imag_values);

    __interpret_magnitudes(real_values, peaks);

    transform_for_diods(peaks, matrix);

}

void transform_for_diods(float* values, uint8_t matrix[MATRIX_LENGTH][MATRIX_LENGTH])
{
    float max = __calc_slice_peak(values, 0, MATRIX_LENGTH);
    float min = __calc_min(values);
    float step = (max - min) / (MATRIX_LENGTH - 1);

    for (uint8_t col = 0; col < MATRIX_LENGTH; ++col)
    {
        uint8_t scaled = floor(values[col] / step);
//        if (scaled == 0)
//        {
//            continue;
//        }

        for (uint8_t row = 0; row <= scaled; ++row)
        {
            matrix[MATRIX_LENGTH - row - 1][col] = 1;
        }
    }
}
