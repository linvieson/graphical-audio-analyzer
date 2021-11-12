#include "read_data.h"


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
    char filename[PATH_MAX];
    if (!getcwd(filename, sizeof(filename))) {
        return 1;
    }

    char data_path[] = "/data/data1.txt";
    strcat(filename, data_path);

    FILE* file = fopen(filename, "r");
    if (!file) {
        return 1;
    }

    save_to_arrays(file, time, values, length);

    fclose(file);
    return 0;
}
