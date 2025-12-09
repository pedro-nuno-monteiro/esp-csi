/*
 * SPDX-FileCopyrightText: 2025 Espressif Systems (Shanghai) CO LTD
 *
 * SPDX-License-Identifier: Apache-2.0
 */
/* Get Start Example

   This example code is in the Public Domain (or CC0 licensed, at your option.)

   Unless required by applicable law or agreed to in writing, this
   software is distributed on an "AS IS" BASIS, WITHOUT WARRANTIES OR
   CONDITIONS OF ANY KIND, either express or implied.
*/

#include <stdio.h>
#include <string.h>
#include <stdlib.h>

#include "nvs_flash.h"

#include "esp_mac.h"
#include "rom/ets_sys.h"
#include "esp_log.h"
#include "esp_wifi.h"
#include "esp_netif.h"
#include "esp_now.h"
#include "esp_radar_t.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_heap_caps.h"

#define CONFIG_LESS_INTERFERENCE_CHANNEL   11
#if CONFIG_IDF_TARGET_ESP32C5 || CONFIG_IDF_TARGET_ESP32C6
#define CONFIG_WIFI_BAND_MODE   WIFI_BAND_MODE_2G_ONLY
#define CONFIG_WIFI_2G_BANDWIDTHS           WIFI_BW_HT20
#define CONFIG_WIFI_5G_BANDWIDTHS           WIFI_BW_HT20
#define CONFIG_WIFI_2G_PROTOCOL             WIFI_PROTOCOL_11N
#define CONFIG_WIFI_5G_PROTOCOL             WIFI_PROTOCOL_11N
#define CONFIG_ESP_NOW_PHYMODE           WIFI_PHY_MODE_HT20
#else
#define CONFIG_WIFI_BANDWIDTH           WIFI_BW_HT40
#endif
#define CONFIG_ESP_NOW_RATE             WIFI_PHY_RATE_MCS0_LGI
#define CONFIG_FORCE_GAIN                   1
#if CONFIG_IDF_TARGET_ESP32C5
#define CSI_FORCE_LLTF                      0
#endif
#if CONFIG_IDF_TARGET_ESP32S3 || CONFIG_IDF_TARGET_ESP32C3 || CONFIG_IDF_TARGET_ESP32C5 || CONFIG_IDF_TARGET_ESP32C6
#define CONFIG_GAIN_CONTROL                 1
#endif

static const uint8_t CONFIG_CSI_SEND_MAC[] = {0x2a, 0x00, 0x00, 0x00, 0x00, 0x00};
static const char *TAG = "csi_recv";
typedef struct {
    unsigned : 32; /**< reserved */
    unsigned : 32; /**< reserved */
    unsigned : 32; /**< reserved */
    unsigned : 32; /**< reserved */
    unsigned : 32; /**< reserved */
#if CONFIG_IDF_TARGET_ESP32S2
    unsigned : 32; /**< reserved */
#elif CONFIG_IDF_TARGET_ESP32S3 || CONFIG_IDF_TARGET_ESP32C3 || CONFIG_IDF_TARGET_ESP32C5 ||CONFIG_IDF_TARGET_ESP32C6
    unsigned : 16; /**< reserved */
    unsigned fft_gain : 8;
    unsigned agc_gain : 8;
    unsigned : 32; /**< reserved */
#endif
    unsigned : 32; /**< reserved */
#if CONFIG_IDF_TARGET_ESP32S2
    signed : 8;  /**< reserved */
    unsigned : 24; /**< reserved */
#elif CONFIG_IDF_TARGET_ESP32S3 || CONFIG_IDF_TARGET_ESP32C3 || CONFIG_IDF_TARGET_ESP32C5
    unsigned : 32; /**< reserved */
    unsigned : 32; /**< reserved */
    unsigned : 32; /**< reserved */
#endif
    unsigned : 32; /**< reserved */
} wifi_pkt_rx_ctrl_phy_t;

#if CONFIG_FORCE_GAIN
/**
 * @brief Enable/disable automatic fft gain control and set its value
 * @param[in] force_en true to disable automatic fft gain control
 * @param[in] force_value forced fft gain value
 */
extern void phy_fft_scale_force(bool force_en, uint8_t force_value);

/**
 * @brief Enable/disable automatic gain control and set its value
 * @param[in] force_en true to disable automatic gain control
 * @param[in] force_value forced gain value
 */
extern void phy_force_rx_gain(int force_en, int force_value);
#endif
static void wifi_init()
{
    // esp_wifi_enable_rx_stbc(1);
    ESP_ERROR_CHECK(esp_event_loop_create_default());

    ESP_ERROR_CHECK(esp_netif_init());
    wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
    ESP_ERROR_CHECK(esp_wifi_init(&cfg));

    ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_STA));
    ESP_ERROR_CHECK(esp_wifi_set_storage(WIFI_STORAGE_RAM));

#if CONFIG_IDF_TARGET_ESP32C5 || CONFIG_IDF_TARGET_ESP32C6
    ESP_ERROR_CHECK(esp_wifi_start());

    esp_wifi_set_band_mode(CONFIG_WIFI_BAND_MODE);
    wifi_protocols_t protocols = {
        .ghz_2g = CONFIG_WIFI_2G_PROTOCOL,
        .ghz_5g = CONFIG_WIFI_5G_PROTOCOL
    };
    ESP_ERROR_CHECK(esp_wifi_set_protocols(ESP_IF_WIFI_STA, &protocols));

    wifi_bandwidths_t bandwidth = {
        .ghz_2g = CONFIG_WIFI_2G_BANDWIDTHS,
        .ghz_5g = CONFIG_WIFI_5G_BANDWIDTHS
    };
    ESP_ERROR_CHECK(esp_wifi_set_bandwidths(ESP_IF_WIFI_STA, &bandwidth));

#else
    ESP_ERROR_CHECK(esp_wifi_set_bandwidth(ESP_IF_WIFI_STA, CONFIG_WIFI_BANDWIDTH));
    ESP_ERROR_CHECK(esp_wifi_start());
#endif

#if CONFIG_IDF_TARGET_ESP32 || CONFIG_IDF_TARGET_ESP32C3 || CONFIG_IDF_TARGET_ESP32S3
    ESP_ERROR_CHECK(esp_wifi_config_espnow_rate(ESP_IF_WIFI_STA, CONFIG_ESP_NOW_RATE));
#endif
    ESP_ERROR_CHECK(esp_wifi_set_ps(WIFI_PS_NONE));
#if CONFIG_IDF_TARGET_ESP32C5 || CONFIG_IDF_TARGET_ESP32C6
    if ((CONFIG_WIFI_BAND_MODE == WIFI_BAND_MODE_2G_ONLY && CONFIG_WIFI_2G_BANDWIDTHS == WIFI_BW_HT20)
            || (CONFIG_WIFI_BAND_MODE == WIFI_BAND_MODE_5G_ONLY && CONFIG_WIFI_5G_BANDWIDTHS == WIFI_BW_HT20)) {
        ESP_ERROR_CHECK(esp_wifi_set_channel(CONFIG_LESS_INTERFERENCE_CHANNEL, WIFI_SECOND_CHAN_NONE));
    } else {
        ESP_ERROR_CHECK(esp_wifi_set_channel(CONFIG_LESS_INTERFERENCE_CHANNEL, WIFI_SECOND_CHAN_BELOW));
    }
#else
    if (CONFIG_WIFI_BANDWIDTH == WIFI_BW_HT20) {
        ESP_ERROR_CHECK(esp_wifi_set_channel(CONFIG_LESS_INTERFERENCE_CHANNEL, WIFI_SECOND_CHAN_NONE));
    } else {
        ESP_ERROR_CHECK(esp_wifi_set_channel(CONFIG_LESS_INTERFERENCE_CHANNEL, WIFI_SECOND_CHAN_BELOW));
    }
#endif

    ESP_ERROR_CHECK(esp_wifi_set_mac(WIFI_IF_STA, CONFIG_CSI_SEND_MAC));
}
#if CONFIG_IDF_TARGET_ESP32C5
static void wifi_esp_now_init(esp_now_peer_info_t peer)
{
    ESP_ERROR_CHECK(esp_now_init());
    ESP_ERROR_CHECK(esp_now_set_pmk((uint8_t *)"pmk1234567890123"));
    esp_now_rate_config_t rate_config = {
        .phymode = CONFIG_ESP_NOW_PHYMODE,
        .rate = CONFIG_ESP_NOW_RATE,//  WIFI_PHY_RATE_MCS0_LGI,
        .ersu = false,
        .dcm = false
    };
    ESP_ERROR_CHECK(esp_now_add_peer(&peer));
    ESP_ERROR_CHECK(esp_now_set_peer_rate_config(peer.peer_addr, &rate_config));

}
#endif
static void radar_rx_cb(void *ctx, const wifi_radar_info_t *info)
{
    ESP_LOGI(TAG, "waveform_jitter %lf, waveform_wander %lf", info->waveform_jitter, info->waveform_wander);
}
static void wifi_csi_rx_cb_1(void *ctx, const wifi_csi_filtered_info_t *filtered_info)
{
    if (filtered_info->data_type == WIFI_CSI_DATA_TYPE_INT16) {
        const int16_t *csi = filtered_info->valid_data_i16;
        int sample_cnt = filtered_info->valid_len / sizeof(int16_t);
        ets_printf("CSI_DATA," MACSTR ",%d,%d,\"[%d", MAC2STR(filtered_info->mac), sample_cnt, 0, csi[0]);
        for (int k = 1; k < sample_cnt; ++k) {
            ets_printf(",%d", csi[k]);
        }
        ets_printf("]\"\n");

        // const int8_t *csi1 = filtered_info->raw_data;
        // ets_printf(",%d,%d,\"[%d", filtered_info->raw_len, 0, (int16_t)(((int16_t)csi1[1]) << 12)>>4 | (uint8_t)csi1[0]);
        // for (int i = 2; i < filtered_info->raw_len; i+=2) {
        //     ets_printf(",%d", (int16_t)(((int16_t)csi1[i+1]) << 12)>>4 | (uint8_t)csi1[i]);
        // }
        // ets_printf("]\"\n");

    } else {
        const int8_t *csi = filtered_info->valid_data;
        int sample_cnt = filtered_info->valid_len;
        printf("rx_gain_compensation: %lf\n", filtered_info->rx_gain_compensation);
        ets_printf("CSI_DATA," MACSTR ",%d,%d,\"[%d", MAC2STR(filtered_info->mac), sample_cnt, 0, csi[0]);
        for (int k = 1; k < sample_cnt; ++k) {
            ets_printf(",%d", csi[k]);
        }
        ets_printf("]\"\n");

        // const int8_t *csi1 = filtered_info->raw_data;
        // ets_printf(",%d,%d,\"[%d", filtered_info->raw_len, 0, csi1[0]);
        // for (int i = 1; i < filtered_info->raw_len; i++) {
        //     ets_printf(",%d", csi1[i]);
        // }
        // ets_printf("]\"\n");
    }
}
static void wifi_csi_rx_cb(void *ctx, wifi_csi_info_t *info)
{
    // ESP_LOGI(TAG, "<%s>", __func__);
    if (!info || !info->buf) {
        ESP_LOGW(TAG, "<%s> wifi_csi_cb", esp_err_to_name(ESP_ERR_INVALID_ARG));
        return;
    }

    if (memcmp(info->mac, ctx, 6)) {
        return;
    }

    wifi_pkt_rx_ctrl_phy_t *phy_info = (wifi_pkt_rx_ctrl_phy_t *)info;
    static int s_count = 0;
#if CONFIG_GAIN_CONTROL
    static uint16_t agc_gain_sum = 0;
    static uint16_t fft_gain_sum = 0;
    static uint8_t agc_gain_force_value = 0;
    static uint8_t fft_gain_force_value = 0;
    if (s_count < 100) {
        agc_gain_sum += phy_info->agc_gain;
        fft_gain_sum += phy_info->fft_gain;
    } else if (s_count == 100) {
        agc_gain_force_value = agc_gain_sum / 100;
        fft_gain_force_value = fft_gain_sum / 100;
#if CONFIG_FORCE_GAIN
        phy_fft_scale_force(1, fft_gain_force_value);
        phy_force_rx_gain(1, agc_gain_force_value);
#endif
        ESP_LOGI(TAG, "fft_force %d, agc_force %d", fft_gain_force_value, agc_gain_force_value);
    }
#endif
    uint32_t rx_id = *(uint32_t *)(info->payload + 15);
    const wifi_pkt_rx_ctrl_t *rx_ctrl = &info->rx_ctrl;
#if CONFIG_IDF_TARGET_ESP32C5 || CONFIG_IDF_TARGET_ESP32C6
    if (!s_count) {
        ESP_LOGI(TAG, "================ CSI RECV ================");
        ets_printf("type,seq,mac,rssi,rate,noise_floor,fft_gain,agc_gain,channel,local_timestamp,sig_len,rx_state,len,first_word,data\n");
    }

    ets_printf("CSI_DATA,%d," MACSTR ",%d,%d,%d,%d,%d,%d,%d,%d,%d",
               rx_id, MAC2STR(info->mac), rx_ctrl->rssi, rx_ctrl->rate,
               rx_ctrl->noise_floor, phy_info->fft_gain, phy_info->agc_gain, rx_ctrl->channel,
               rx_ctrl->timestamp, rx_ctrl->sig_len, rx_ctrl->rx_state);
#else
    if (!s_count) {
        ESP_LOGI(TAG, "================ CSI RECV ================");
        ets_printf("type,id,mac,rssi,rate,sig_mode,mcs,bandwidth,smoothing,not_sounding,aggregation,stbc,fec_coding,sgi,noise_floor,ampdu_cnt,channel,secondary_channel,local_timestamp,ant,sig_len,rx_state,len,first_word,data\n");
    }

    ets_printf("CSI_DATA,%d," MACSTR ",%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d",
               rx_id, MAC2STR(info->mac), rx_ctrl->rssi, rx_ctrl->rate, rx_ctrl->sig_mode,
               rx_ctrl->mcs, rx_ctrl->cwb, rx_ctrl->smoothing, rx_ctrl->not_sounding,
               rx_ctrl->aggregation, rx_ctrl->stbc, rx_ctrl->fec_coding, rx_ctrl->sgi,
               rx_ctrl->noise_floor, rx_ctrl->ampdu_cnt, rx_ctrl->channel, rx_ctrl->secondary_channel,
               rx_ctrl->timestamp, rx_ctrl->ant, rx_ctrl->sig_len, rx_ctrl->rx_state);

#endif
#if CONFIG_IDF_TARGET_ESP32C5 && CSI_FORCE_LLTF
    ets_printf(",%d,%d,\"[%d", (info->len - 2) / 2, info->first_word_invalid, (int16_t)(((int16_t)info->buf[1]) << 12) >> 4 | (uint8_t)info->buf[0]);
    for (int i = 2; i < (info->len - 2); i += 2) {
        ets_printf(",%d", (int16_t)(((int16_t)info->buf[i + 1]) << 12) >> 4 | (uint8_t)info->buf[i]);
    }
#else
    ets_printf(",%d,%d,\"[%d", info->len, info->first_word_invalid, info->buf[0]);
    for (int i = 1; i < info->len; i++) {
        ets_printf(",%d", info->buf[i]);
    }
#endif
    ets_printf("]\"\n");
    s_count++;
}

static void wifi_csi_init()
{
    ESP_ERROR_CHECK(esp_wifi_set_promiscuous(true));

    /**< default config */
#if CONFIG_IDF_TARGET_ESP32C5
    wifi_csi_config_t csi_config = {
        .enable                   = true,
        .acquire_csi_legacy       = false,
        .acquire_csi_force_lltf   = CSI_FORCE_LLTF,
        .acquire_csi_ht20         = true,
        .acquire_csi_ht40         = true,
        .acquire_csi_vht          = false,
        .acquire_csi_su           = false,
        .acquire_csi_mu           = false,
        .acquire_csi_dcm          = false,
        .acquire_csi_beamformed   = false,
        .acquire_csi_he_stbc_mode = 2,
        .val_scale_cfg            = 0,
        .dump_ack_en              = false,
        .reserved                 = false
    };
#elif CONFIG_IDF_TARGET_ESP32C6
    wifi_csi_config_t csi_config = {
        .enable                 = true,
        .acquire_csi_legacy     = false,
        .acquire_csi_ht20       = true,
        .acquire_csi_ht40       = true,
        .acquire_csi_su         = false,
        .acquire_csi_mu         = false,
        .acquire_csi_dcm        = false,
        .acquire_csi_beamformed = false,
        .acquire_csi_he_stbc    = 2,
        .val_scale_cfg          = false,
        .dump_ack_en            = false,
        .reserved               = false
    };
#else
    wifi_csi_config_t csi_config = {
        .lltf_en           = true,
        .htltf_en          = true,
        .stbc_htltf2_en    = true,
        .ltf_merge_en      = true,
        .channel_filter_en = true,
        .manu_scale        = false,
        .shift             = false,
    };
#endif
    ESP_ERROR_CHECK(esp_wifi_set_csi_config(&csi_config));
    ESP_ERROR_CHECK(esp_wifi_set_csi_rx_cb(wifi_csi_rx_cb, NULL));
    ESP_ERROR_CHECK(esp_wifi_set_csi(true));
}

// 内存监控任务
static void memory_monitor_task(void *arg)
{
    const char *TAG_MEM = "memory_monitor";
    uint32_t free_heap_size = 0;
    uint32_t min_free_heap_size = 0;
    uint32_t last_free_heap_size = 0;

    uint32_t free_psram_size = 0;
    uint32_t last_free_psram_size = 0;
    uint32_t free_internal_size = 0;
    uint32_t last_free_internal_size = 0;

    ESP_LOGI(TAG_MEM, "Memory monitor task started");

    while (1) {
        free_heap_size = esp_get_free_heap_size();
        min_free_heap_size = esp_get_minimum_free_heap_size();
        free_internal_size = heap_caps_get_free_size(MALLOC_CAP_INTERNAL);
        free_psram_size = heap_caps_get_free_size(MALLOC_CAP_SPIRAM);

        // 打印详细的内存信息
        ESP_LOGI(TAG_MEM, "=== Memory Status ===");
        ESP_LOGI(TAG_MEM, "Total free heap: %lu bytes (%.2f KB)",
                 free_heap_size, free_heap_size / 1024.0f);
        ESP_LOGI(TAG_MEM, "Min free heap: %lu bytes (%.2f KB)",
                 min_free_heap_size, min_free_heap_size / 1024.0f);

        // 打印内存分配详情（按能力）
        ESP_LOGI(TAG_MEM, "Internal RAM free: %lu bytes (%.2f KB)",
                 free_internal_size, free_internal_size / 1024.0f);
        ESP_LOGI(TAG_MEM, "PSRAM free: %lu bytes (%.2f KB)",
                 free_psram_size, free_psram_size / 1024.0f);
        ESP_LOGI(TAG_MEM, "DMA capable free: %lu bytes (%.2f KB)",
                 heap_caps_get_free_size(MALLOC_CAP_DMA),
                 heap_caps_get_free_size(MALLOC_CAP_DMA) / 1024.0f);

        // 打印最大可分配块
        ESP_LOGI(TAG_MEM, "Largest free block (Internal): %lu bytes",
                 heap_caps_get_largest_free_block(MALLOC_CAP_INTERNAL));
        ESP_LOGI(TAG_MEM, "Largest free block (PSRAM): %lu bytes",
                 heap_caps_get_largest_free_block(MALLOC_CAP_SPIRAM));

        // 检测总内存泄漏
        if (last_free_heap_size > 0 && free_heap_size < last_free_heap_size) {
            int32_t leaked = last_free_heap_size - free_heap_size;
            ESP_LOGW(TAG_MEM, "Total memory decreased by %ld bytes!", leaked);
        }

        // 检测 Internal RAM 泄漏
        if (last_free_internal_size > 0 && free_internal_size < last_free_internal_size) {
            int32_t leaked = last_free_internal_size - free_internal_size;
            ESP_LOGW(TAG_MEM, "Internal RAM decreased by %ld bytes!", leaked);
        }

        // 检测 PSRAM 泄漏
        if (last_free_psram_size > 0 && free_psram_size < last_free_psram_size) {
            int32_t leaked = last_free_psram_size - free_psram_size;
            ESP_LOGW(TAG_MEM, "PSRAM decreased by %ld bytes!", leaked);
        }

        last_free_heap_size = free_heap_size;
        last_free_internal_size = free_internal_size;
        last_free_psram_size = free_psram_size;

        // 每5秒打印一次
        vTaskDelay(pdMS_TO_TICKS(5000));
    }
}

void app_main()
{
    /**
     * @brief Initialize NVS
     */

    esp_err_t ret = nvs_flash_init();
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        ESP_ERROR_CHECK(nvs_flash_erase());
        ret = nvs_flash_init();
    }
    ESP_ERROR_CHECK(ret);

    esp_radar_csi_config_t csi_config = ESP_RADAR_CSI_CONFIG_DEFAULT();
    memcpy(csi_config.filter_mac, CONFIG_CSI_SEND_MAC, 6);
    // csi_config.csi_filtered_cb = wifi_csi_rx_cb_1;
    // csi_config.acquire_csi_lltf = false;
    // csi_config.csi_compensate_en = false;
    // csi_config.csi_filtered_cb_ctx = CONFIG_CSI_SEND_MAC;
    // csi_config.csi_recv_interval = 50;
    esp_radar_wifi_config_t wifi_config = ESP_RADAR_WIFI_CONFIG_DEFAULT();
    esp_radar_espnow_config_t espnow_config = ESP_RADAR_ESPNOW_CONFIG_DEFAULT();
    esp_radar_config_t radar_config = ESP_RADAR_CONFIG_DEFAULT();
    radar_config.wifi_radar_cb = radar_rx_cb;
    ESP_ERROR_CHECK(esp_radar_wifi_init(&wifi_config, &espnow_config));
    ESP_ERROR_CHECK(esp_radar_set_csi_config(&csi_config));
    ESP_ERROR_CHECK(esp_radar_init(&radar_config));
    esp_radar_start();
    float wander_threshold = 0;
    float jitter_threshold = 0;
    vTaskDelay(pdMS_TO_TICKS(3000));
    esp_radar_train_start();
    vTaskDelay(pdMS_TO_TICKS(5000));
    esp_radar_train_stop(&wander_threshold, &jitter_threshold);
    ESP_LOGI(TAG, "wander_threshold: %f, jitter_threshold: %f", wander_threshold, jitter_threshold);
    // 启动内存监控任务
    // xTaskCreate(memory_monitor_task, "mem_monitor", 4096, NULL, 5, NULL);
    // ESP_LOGI(TAG, "Memory monitor task created");

    /**
     * @brief Initialize Wi-Fi
     */
//     wifi_init();

//     /**
//      * @brief Initialize ESP-NOW
//      *        ESP-NOW protocol see: https://docs.espressif.com/projects/esp-idf/en/latest/esp32/api-reference/network/esp_now.html
//      */
// #if CONFIG_IDF_TARGET_ESP32C5
//     esp_now_peer_info_t peer = {
//         .channel   = CONFIG_LESS_INTERFERENCE_CHANNEL,
//         .ifidx     = WIFI_IF_STA,
//         .encrypt   = false,
//         .peer_addr = {0xff, 0xff, 0xff, 0xff, 0xff, 0xff},
//     };

//     wifi_esp_now_init(peer);
// #endif
//     wifi_csi_init();
}
