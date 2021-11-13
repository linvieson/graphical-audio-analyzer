#include "operate_data.h"


void save_to_arrays(FILE* file, double *time, double *values, size_t length) {
    char buffer[length];                                                           // buffer for reading a line
    size_t num = 0;

    while (num < length && fgets (buffer, length, file)) {
        if (sscanf(buffer, "%lf %lf", &time[num], &values[num]) == 2) {     // if line contains 2 doubles
            ++num;
        }
    }
}

int read_data (double* time, double* values, size_t length) {
    FILE* file = fopen("./data/data1.txt", "r");
    if (!file) {
        return 1;
    }
    save_to_arrays(file, time, values, length);

    fclose(file);
    return 0;
}

int write_abs_data(float complex* vector, size_t length) {
    FILE *file = fopen("./data/results1.txt", "wb");
    for (size_t index = 0; index < length; ++index) {
        fprintf(file, "%f\n", fabsf(vector[index]));
    }
    fclose(file);
    return 0;
}
