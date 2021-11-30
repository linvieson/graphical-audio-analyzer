#ifndef OPERATE_DATA_H
#define OPERATE_DATA_H

#include <math.h>
#include <stdint.h>
#include <stdio.h>
#include <string.h>


void __save_to_arrays(FILE* file, float *time, float *values, const uint16_t length);
int read_data (float* time, float* values, const uint16_t length);

int write_data(float* values, const uint16_t length);


#endif