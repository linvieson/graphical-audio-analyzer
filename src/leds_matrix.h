#ifndef LEDS_MATRIX_H
#define LEDS_MATRIX_H

#include <stdint.h>

#define MATRIX_LENGTH   8

void transform_to_matrix(uint8_t* peaks, uint8_t matrix[MATRIX_LENGTH][MATRIX_LENGTH][3]);

#endif
