#include "leds_matrix.h"

#include <stdio.h>

void transform_to_matrix(uint8_t* peaks, uint8_t matrix[MATRIX_LENGTH][MATRIX_LENGTH][3])
{
//    uint8_t COLUMN_1[3] = {60, 60, 60};
//    uint8_t COLUMN_2[3] = {60, 60, 60};
//    uint8_t COLUMN_3[3] = {60, 60, 60};
//    uint8_t COLUMN_4[3] = {60, 60, 60};
//    uint8_t COLUMN_5[3] = {60, 60, 60};
//    uint8_t COLUMN_6[3] = {60, 60, 60};
//    uint8_t COLUMN_7[3] = {60, 60, 60};
//    uint8_t COLUMN_8[3] = {60, 60, 60};
    uint8_t COLORS[MATRIX_LENGTH][3] = {
            {60, 60, 60},
            {60, 60, 60},
            {60, 60, 60},
            {60, 60, 60},
            {60, 60, 60},
            {60, 60, 60},
            {60, 60, 60},
            {60, 60, 60}
    };
//     = {COLUMN_1, COLUMN_2, COLUMN_3, COLUMN_4, COLUMN_5, COLUMN_6, COLUMN_7, COLUMN_8};

    uint8_t PEAK_COL[3] = {100, 100, 100};
    uint8_t WEAK_COL[3] = {30, 30, 30};

//    for (uint8_t col = 0; col < MATRIX_LENGTH; ++col)
//    {
//        uint8_t peak = peaks[col];
////        if (peak == 0)
////        {
////            matrix[0][col] = WEAK_COL;
////            continue;
////        } else {
////            matrix[(MATRIX_LENGTH - peak - 1) * col] = PEAK_COL;
////        }
//
//        for (uint8_t row = 0; row < peak; ++row)
//        {
//            matrix[(MATRIX_LENGTH - row - 1) * col] = COLORS[col];
//        }
//    }

    for (uint8_t ind = 0; ind < MATRIX_LENGTH * MATRIX_LENGTH; ++ind)
    {
//        uint8_t peak_ind = (nd) % MATRIX_LENGTH;
//        uint8_t peak = peaks[peak_ind];
//        printf("%d ", peak);
    }

    for (uint8_t peak_ind = 0; peak_ind < MATRIX_LENGTH; ++peak_ind)
    {
        matrix[peak_ind][peaks[peak_ind]] = PEAK_COL;
    }
}