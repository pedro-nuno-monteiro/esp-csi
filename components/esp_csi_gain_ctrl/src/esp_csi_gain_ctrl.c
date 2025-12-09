/*
 * SPDX-FileCopyrightText: 2025 Espressif Systems (Shanghai) CO LTD
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include <math.h>

#include "esp_system.h"
#include "esp_log.h"
#include "esp_csi_gain_ctrl.h"

#if WIFI_CSI_PHY_GAIN_ENABLE

#define FIX_GAIN_BUFF_SIZE          50
#define FIX_GAIN_OUTLIER_THRESHOLD  8

typedef struct {
    uint8_t reserved[22];
    uint8_t fft_gain;
    uint8_t agc_gain;
} __attribute__((packed)) wifi_pkt_rx_ctrl_gain_info_t;

typedef struct {
    bool force_en;

    uint32_t count;
    uint32_t baseline_count;

    uint8_t agc_gain_buff[FIX_GAIN_BUFF_SIZE];
    int8_t fft_gain_buff[FIX_GAIN_BUFF_SIZE];
} rx_gain_record_t;

extern void phy_fft_scale_force(bool force_en, uint8_t force_value);
extern void phy_force_rx_gain(bool force_en, int force_value);

static const char *TAG = "wifi_rx_gain";

static rx_gain_record_t g_rx_gain_record = {0};
static uint8_t g_agc_gain_baseline = 0;
static int8_t g_fft_gain_baseline  = 0;
esp_err_t esp_csi_gain_ctrl_get_rx_gain_baseline(uint8_t* agc_gain, int8_t *fft_gain)
{
    if (!agc_gain || !fft_gain) {
        return ESP_ERR_INVALID_ARG;
    }
    *agc_gain = g_agc_gain_baseline;
    *fft_gain = g_fft_gain_baseline;
    return ESP_OK;
}

rx_gain_status_t esp_csi_gain_ctrl_get_gain_status(void)
{
    if (g_rx_gain_record.force_en) {
        return RX_GAIN_FORCE;
    } else if (g_rx_gain_record.baseline_count >= FIX_GAIN_BUFF_SIZE) {
        return RX_GAIN_READY;
    } else {
        return RX_GAIN_COLLECT;
    }
}

typedef struct {
    uint8_t agc;
    int8_t fft;
} gain_pair_t;

static int cmp_gain_pair_by_agc(const void *a, const void *b)
{
    const gain_pair_t *pa = (const gain_pair_t *)a;
    const gain_pair_t *pb = (const gain_pair_t *)b;
    return (int)pa->agc - (int)pb->agc;
}

static inline int16_t clamp_int16(int32_t v)
{
    if (v > INT16_MAX) {
        return INT16_MAX;
    }
    if (v < INT16_MIN) {
        return INT16_MIN;
    }
    return (int16_t)v;
}

static inline int8_t clamp_int8(int32_t v)
{
    if (v > INT8_MAX) {
        return INT8_MAX;
    }
    if (v < INT8_MIN) {
        return INT8_MIN;
    }
    return (int8_t)v;
}

static esp_err_t esp_csi_gain_ctrl_calculate_gain_baseline(uint8_t* agc_gain, int8_t *fft_gain)
{
    if (!agc_gain || !fft_gain) {
        return ESP_ERR_INVALID_ARG;
    }

    if (g_rx_gain_record->baseline_count < FIX_GAIN_BUFF_SIZE) {
        return ESP_ERR_INVALID_STATE;
    }

    gain_pair_t *tmp = (gain_pair_t *)malloc(sizeof(gain_pair_t) * FIX_GAIN_BUFF_SIZE);
    if (tmp == NULL) {
        return ESP_ERR_NO_MEM;
    }
    for (uint32_t i = 0; i < FIX_GAIN_BUFF_SIZE; i++) {
        tmp[i].agc = g_rx_gain_record->agc_gain_buff[i];
        tmp[i].fft = g_rx_gain_record->fft_gain_buff[i];
    }

    qsort(tmp, FIX_GAIN_BUFF_SIZE, sizeof(gain_pair_t), cmp_gain_pair_by_agc);

    uint32_t mid = FIX_GAIN_BUFF_SIZE / 2;
    *agc_gain = tmp[mid].agc;
    *fft_gain = tmp[mid].fft;

    ESP_LOGI(TAG, "agc_gain(median): %u, fft_gain@median: %d", *agc_gain, *fft_gain);
    free(tmp);
    return ESP_OK;
}

esp_err_t esp_csi_gain_ctrl_record_rx_gain(uint8_t agc_gain, int8_t fft_gain)
{
    uint32_t index = g_rx_gain_record->count % FIX_GAIN_BUFF_SIZE;
    g_rx_gain_record->agc_gain_buff[index] = agc_gain;
    g_rx_gain_record->fft_gain_buff[index] = fft_gain;

    g_rx_gain_record->count++;
    if (g_rx_gain_record->baseline_count < FIX_GAIN_BUFF_SIZE) {
        g_rx_gain_record->baseline_count++;
        if (g_rx_gain_record->baseline_count == FIX_GAIN_BUFF_SIZE) {
            esp_csi_gain_ctrl_calculate_gain_baseline(&g_agc_gain_baseline, &g_fft_gain_baseline);
        }
    }
    return ESP_OK;
}

esp_err_t esp_csi_gain_ctrl_set_rx_force_gain(uint8_t agc_gain, int8_t fft_gain)
{
    if (agc_gain == 0 && fft_gain == 0) {
        phy_force_rx_gain(false, 0);
        phy_fft_scale_force(false, 0);

        g_rx_gain_record->force_en = false;
    } else {
        if (agc_gain <= 25) {
            ESP_LOGE(TAG, "Fixed rx gain failed, 'rx_gain <= 25' will prevent wifi packets from being sent out properly");
            return ESP_ERR_INVALID_STATE;
        }

        g_rx_gain_record->force_en = true;
        phy_force_rx_gain(true, agc_gain);
        phy_fft_scale_force(true, (uint8_t)fft_gain);
    }

    return ESP_OK;
}

void esp_csi_gain_ctrl_reset_rx_gain_baseline(void)
{
    g_agc_gain_baseline = 0;
    g_fft_gain_baseline = 0;
    g_rx_gain_record->baseline_count = 0;
}

esp_err_t esp_csi_gain_ctrl_get_gain_compensation(float *compensate_gain, uint8_t agc_gain, int8_t fft_gain)
{

    if (!g_rx_gain_record) {
        return ESP_ERR_INVALID_STATE;
    }

    if (g_rx_gain_record->baseline_count < FIX_GAIN_BUFF_SIZE) {
        g_agc_gain_baseline = 0;
        g_fft_gain_baseline = 0;
        return ESP_ERR_INVALID_STATE;
    }

    if (g_agc_gain_baseline == 0 && g_fft_gain_baseline == 0) {
        if (esp_csi_gain_ctrl_calculate_gain_baseline(&g_agc_gain_baseline, &g_fft_gain_baseline) != ESP_OK) {
            return ESP_ERR_INVALID_STATE;
        }
    }

    float compensate_factor = powf(10.0, ((agc_gain - g_agc_gain_baseline) + (fft_gain - g_fft_gain_baseline) / 4.0) / -20.0);
    *compensate_gain = compensate_factor;

    return ESP_OK;
}

esp_err_t esp_csi_gain_ctrl_compensate_rx_gain(void *data, uint16_t size, bool samples_are_16bit,
                                               float *compensate_gain, uint8_t agc_gain, int8_t fft_gain)
{
    float compensate_factor = 0;
    esp_err_t err = esp_csi_gain_ctrl_get_gain_compensation(&compensate_factor, agc_gain, fft_gain);
    if (err != ESP_OK) {
        return err;
    }

    *compensate_gain = compensate_factor;

    if (samples_are_16bit) {
        int16_t *ptr = (int16_t *)data;
        uint16_t sample_count = size / sizeof(int16_t);
        for (uint16_t i = 0; i < sample_count; i++) {
            int32_t scaled = (int32_t)roundf(ptr[i] * compensate_factor);
            ptr[i] = clamp_int16(scaled);
        }
    } else {
        int8_t *ptr = (int8_t *)data;
        for (uint16_t i = 0; i < size; i++) {
            int32_t scaled = (int32_t)roundf(ptr[i] * compensate_factor);
            ptr[i] = clamp_int8(scaled);
        }
    }
    return ESP_OK;
}

void esp_csi_gain_ctrl_get_rx_gain(const void *rx_ctrl, uint8_t* agc_gain, int8_t* fft_gain)
{
    if (!rx_ctrl || !agc_gain || !fft_gain) {
        ESP_LOGE(TAG, "invalid args to get_rx_gain");
        return;
    }
    *agc_gain = ((wifi_pkt_rx_ctrl_gain_info_t *)rx_ctrl)->agc_gain;
    *fft_gain = (int8_t)((wifi_pkt_rx_ctrl_gain_info_t *)rx_ctrl)->fft_gain;
}

#endif
