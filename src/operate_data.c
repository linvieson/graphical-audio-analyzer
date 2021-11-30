#include "operate_data.h"


void __save_to_arrays(FILE* file, float *time, float *values, const uint16_t length)
{
    char buffer[length];
    size_t num = 0;

    while (num < length && fgets (buffer, length, file))
    {
        if (sscanf(buffer, "%f %f", &time[num], &values[num]) == 2)
        {
            ++num;
        }
    }
}

int read_data (float* time, float* values, const uint16_t length)
{
    FILE* file = fopen("./data/data1.txt", "r");
    if (!file)
    {
        return 1;
    }
    __save_to_arrays(file, time, values, length);

    fclose(file);
    return 0;
}

int write_data(float* values, const uint16_t length)
{
    FILE *file = fopen("./data/magnitudes1.txt", "wb");
    if (!file)
    {
        return 1;
    }

    for (size_t index = 0; index < length; ++index)
    {
        fprintf(file, "%f\n", fabsf(values[index]));
    }
    fclose(file);
    return 0;
}