/*
 * SPDX-FileCopyrightText: 2025 Espressif Systems (Shanghai) CO LTD
 *
 * SPDX-License-Identifier: Apache-2.0
 */
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <math.h>
#include <sys/param.h>
#include <esp_log.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#include "esp_radar.h"

#define ITRS_MAX 30
#define PRECISION 0.0001

#define DOUBLE_CALCULATE

static const char *TAG = "algorithm";

esp_err_t compute_cov(uint32_t row, uint32_t column,
                      const float matrix[row][column],
                      float matrix_cov[column][column])
{
    uint32_t zoom_out = row * column;

    for (int i = 0; i < column; i++) {
        for (int j = 0; j < i + 1; j++) {
            for (int k = 0; k < row; k++) {
                matrix_cov[i][j] += matrix[k][i] * matrix[k][j];
            }

            matrix_cov[i][j] /= zoom_out;

            if (i != j) {
                matrix_cov[j][i] = matrix_cov[i][j];
            }
        }
    }

    return ESP_OK;
}

// #define PYTHON
// #define MATRIX_PRINT
#ifdef MATRIX_PRINT

static void matrix_print(int row, int column, const float data[row][column])
{
    printf("===  matrix, row: %d, column: %d\n", row, column);

#ifdef PYTHON
    printf("data = [\n");

    for (int i = 0; i < row; i++) {
        printf("\t[");

        for (int j = 0; j < column; j++) {
            printf("\t%.2f,", data[i][j]);
        }

        printf("\b],\n");
    }

    printf("]\n");

    printf("const float matrix_0[%d][%d] = {\n", row, column);

    for (int i = 0; i < row; i++) {
        printf("\t{");

        for (int j = 0; j < column; j++) {
            printf("\t%.2f,", data[i][j]);
        }

        printf("\b},\n");
    }

    printf("};\n");
#else

#endif
}
#endif

static esp_err_t matrix_power_method_eigen(uint32_t row, uint32_t column, const float matrix[row][column],
                                           uint32_t itrs_max, float delta_min, float *eigenvector)
{
    uint32_t iterate = 0;
#ifdef DOUBLE_CALCULATE
    double eigenvalue = 1.0;
    double eigenvalue_last = 0;
    double *eigenvalue_list = RADAR_MALLOC_RETRY(column * sizeof(double));
#else
    float eigenvalue = 1.0;
    float eigenvalue_last = 0;
    float *eigenvalue_list = RADAR_MALLOC_RETRY(column * sizeof(float));
#endif

    for (int i = 0; i < column; i++) {
        eigenvector[i] = 1;
    }

    for (iterate = 0; fabs(eigenvalue - eigenvalue_last) > delta_min && iterate < itrs_max;
            iterate++) {
        eigenvalue_last = eigenvalue;
        eigenvalue      = 0;

        for (int i = 0; i < row; i++) {
            eigenvalue_list[i] = 0;

            for (int j = 0; j < column; j++) {
                eigenvalue_list[i] += matrix[i][j] * eigenvector[j];
            }

            if (eigenvalue_list[i] > eigenvalue) {
                eigenvalue =  eigenvalue_list[i];
            }
        }

        for (int i = 0; i < row; i++) {
            eigenvector[i] = eigenvalue_list[i] / eigenvalue;
        }
    }

    RADAR_FREE(eigenvalue_list);

    ESP_LOGD(TAG, "iterate: %ld, itrs_max: %ld", iterate, itrs_max);
    return (iterate == itrs_max) ? ESP_FAIL : ESP_OK;
}

esp_err_t pca(uint32_t cols,
              uint32_t row_0, const float data_0[row_0][cols],
              uint32_t row_1, const float data_1[row_1][cols],
              float output[cols])
{
    esp_err_t ret   = ESP_OK;
    uint32_t row    = cols;
    uint32_t column = row_0 + row_1;
    // int64_t start_time = esp_timer_get_time();

    // printf("cols: %d, row_0: %d, row_1: %d\n", cols, row_0, row_1);

    float (*matrix)[column] = RADAR_MALLOC_RETRY(row * column * sizeof(float));

    for (int i = 0; i < row; i++) {
        for (int j = 0; j < row_0; j++) {
            matrix[i][j] = data_0[j][i];
        }

        for (int j = 0; j < row_1; j++) {
            matrix[i][j + row_0] = data_1[j][i];
        }
    }

    float (*matrix_cov)[column] = RADAR_MALLOC_RETRY(column * column * sizeof(float));
    memset(matrix_cov, 0, column * column * sizeof(float));

    compute_cov(row, column, matrix, matrix_cov);

    float *eigenvector = RADAR_MALLOC_RETRY(column * sizeof(float));
    ret = matrix_power_method_eigen(column, column, matrix_cov,
                                    ITRS_MAX, PRECISION, eigenvector);

    if (ret != ESP_OK) {
        // ESP_LOGW(TAG, "[%d] fail", __LINE__);
        // matrix_print(row_0, cols, data_0);
        // matrix_print(row_1, cols, data_1);

        goto EXIT;
    }

    for (int i = 0; i < row; i++) {
        output[i] = 0;

        for (int j = 0; j < column; j++) {
            output[i] +=  matrix[i][j] * eigenvector[j];
        }

        output[i] /= column;
    }

    // printf("pca = [");
    // for (int i = 0; i < row; i++) {
    //     printf("\t%.2f,", output[i]);
    // }
    // printf("\b]\n");

EXIT:
    RADAR_FREE(matrix);
    RADAR_FREE(matrix_cov);
    RADAR_FREE(eigenvector);

    // ESP_LOGD(TAG, "[%s, %d] spend time: %lld", __FUNCTION__, __LINE__, esp_timer_get_time() - start_time);
    return ret;
}
