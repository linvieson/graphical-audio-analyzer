#include "fft.h"

void __pre_fft(float real_values[SAMPLES])
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

void __compute_fft(float real_values[SAMPLES], float imag_values[SAMPLES])
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
        c1 =   pow((1.0 + c1) / 2.0, 0.5);
    }
}

void __compute_magnitudes(float real_values[SAMPLES], float imag_values[SAMPLES])
// Compute in-place the absolute values of the received numbers from the FFT.
// They are stored in the array real_values.
{
    for (uint16_t ind = 0; ind < SAMPLES >> 1; ++ind)
    {
        // |z| = sqrt(a^2 + b^2)
        real_values[ind] = sqrt(real_values[ind] * real_values[ind] + imag_values[ind] * imag_values[ind]);
    }
}

void __interpret_magnitudes(float magnitudes[SAMPLES], float peaks[MATRIX_LENGTH])
// step = SAMPLING_FREQ / SAMPLES.
// Divide the fist half of the array of magnitudes on the MATRIX_LENGTH parts,
// with each next slice being larger than the previous one.
// For each slice calculate the peak, and store them in the peaks array.

// STEP = 36
// peaks[0] = 108   - 287   Hz
// peaks[1] = 288   - 539   Hz
// peaks[2] = 540   - 899   Hz
// peaks[3] = 900   - 2699  Hz
// peaks[4] = 2700  - 4499  Hz
// peaks[5] = 4500  - 7199  Hz
// peaks[6] = 7200  - 10799 Hz
// peaks[7] = 10800 - 18432 Hz
{
    uint16_t slices[] = {20, 50, 100, 150, 250, 350, 670, 1024};

    uint16_t upper_bound;
    uint16_t lower_bound = 10;

    for (uint8_t ind = 0; ind < MATRIX_LENGTH; ++ind)
    {
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

void __transform_for_diods(float values[MATRIX_LENGTH], uint8_t leds[MAX_LED])
// Fill the array leds consisting of MAX_LEDS elements of 0 or 1, denoting whether to
// lighten the led or not, by taking the related magnitudes of the frequencies.
{
//	values[0] /= 1.25;
    float max_peak = __calc_slice_peak(values, 0, MATRIX_LENGTH);
    float max_emp  = 250000.0;

    float max = (max_peak > max_emp) ? max_peak : max_emp;

//    float min = __calc_min(values);
    float min = 10000.0;
    float step = (max - min) / (MATRIX_LENGTH - 1);

    for (uint8_t col = 0; col < MATRIX_LENGTH; ++col)
    {
        uint8_t scaled_peak = floor(values[col] / step);

        uint8_t led_ind = (MATRIX_LENGTH - 1) * MATRIX_LENGTH + col;

        if (scaled_peak >= 8) {
        	scaled_peak = 7;
        }

        for (uint8_t row = 0; row <= scaled_peak; ++row)
        {
            leds[led_ind] = 1;
            led_ind -= MATRIX_LENGTH;
        }
    }
}

void perform_fft(float real_values[SAMPLES], uint8_t leds[MAX_LED])
// Driver function to compute the peaks and the average values, which will be stored in the arrays peaks and means respectively.
{
    __pre_fft(real_values);

    float imag_values[SAMPLES] = {};
    float peaks[MATRIX_LENGTH] = {};

    __compute_fft(real_values, imag_values);
    __compute_magnitudes(real_values, imag_values);

    for (uint8_t i = 0; i < 50; ++i)
    {
    	real_values[i] /= 80.0 / (30.0 + i);
    }

    __interpret_magnitudes(real_values, peaks);

    __transform_for_diods(peaks, leds);
}
