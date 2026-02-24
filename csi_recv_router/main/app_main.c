/*
 * SPDX-FileCopyrightText: 2026 Espressif Systems (Shanghai) CO LTD
 *
 * SPDX-License-Identifier: Apache-2.0
 */
/* Get recv router csi

   This example code is in the Public Domain (or CC0 licensed, at your option.)

   Unless required by applicable law or agreed to in writing, this
   software is distributed on an "AS IS" BASIS, WITHOUT WARRANTIES OR
   CONDITIONS OF ANY KIND, either express or implied.
*/
#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include <errno.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/event_groups.h"

#include "nvs_flash.h"

#include "esp_mac.h"
#include "rom/ets_sys.h"
#include "esp_log.h"
#include "esp_wifi.h"
#include "esp_netif.h"
#include "esp_now.h"

#include "lwip/inet.h"
#include "lwip/netdb.h"
#include "lwip/sockets.h"

#include "ping/ping_sock.h"
#include "esp_timer.h"

#include "protocol_examples_common.h"
#include "esp_csi_gain_ctrl.h"

/* ============= USER CONFIG ============= */

#define ESP_WIFI_SSID      "NOS_JCAS"
#define ESP_WIFI_PASS      "q1w2e3r4"

#define server_ip          "10.95.100.235"
#define SERVER_UDP_PORT    5001

#define CSI_QUEUE_SIZE     64
#define MAX_CSI_LEN        256

#define CONFIG_SEND_FREQUENCY 50

/* ======================================== */


/* ============= CSI CONFIG  ============== */

typedef struct {
    uint8_t type;
    uint32_t seq;
    uint8_t mac[6];
    int8_t rssi;
    uint8_t rate;
    int8_t noise_floor;
    int8_t fft_gain;
    uint8_t agc_gain;
    uint8_t channel;
    uint32_t local_timestamp;
    uint16_t sig_len;
    uint16_t rx_state;
    uint16_t len;
    uint8_t first_word_invalid;
    uint16_t csi_len;
    int16_t csi_data[MAX_CSI_LEN];
} csi_packet;

/* ======================================== */

static QueueHandle_t csi_queue;
static int udp_sock = -1;
static struct sockaddr_in server_addr;

#if CONFIG_IDF_TARGET_ESP32C5 || CONFIG_IDF_TARGET_ESP32C61
#define CSI_FORCE_LLTF                      0
#endif
#define CONFIG_FORCE_GAIN                   0

#if CONFIG_IDF_TARGET_ESP32S3 || CONFIG_IDF_TARGET_ESP32C3 || CONFIG_IDF_TARGET_ESP32C5 || CONFIG_IDF_TARGET_ESP32C6 || CONFIG_IDF_TARGET_ESP32C61
#define CONFIG_GAIN_CONTROL                 1
#endif

#if ESP_IDF_VERSION >= ESP_IDF_VERSION_VAL(6, 0, 0)
#define ESP_IF_WIFI_STA ESP_MAC_WIFI_STA
#endif

static const char *TAG = "CSI Recv Router Mode";

static void udp_init(void) {
    udp_sock = socket(AF_INET, SOCK_DGRAM, IPPROTO_IP);
    if (udp_sock < 0) {
        ESP_LOGE(TAG, "Unable to create UDP socket");
        return;
    }

    memset(&server_addr, 0, sizeof(server_addr));
    server_addr.sin_family = AF_INET;
    server_addr.sin_port = htons(SERVER_UDP_PORT);
    server_addr.sin_addr.s_addr = inet_addr(server_ip);

    int sndbuf = 16384;
    setsockopt(udp_sock, SOL_SOCKET, SO_SNDBUF, &sndbuf, sizeof(sndbuf));

    ESP_LOGI(TAG, "UDP socket initialized");
}

/* ===================== UDP TASK ======================== */

static void csi_udp_task(void *pvParameters)
{
    csi_packet packet;

    while (1) {
        if (xQueueReceive(csi_queue, &packet, portMAX_DELAY)) {

            int send_len = offsetof(csi_packet, csi_data) + packet.csi_len;
            int err;
            do {
                err = sendto(udp_sock,
                             &packet,
                             send_len,
                             0,
                             (struct sockaddr *)&server_addr,
                             sizeof(server_addr));
                if (err < 0 && errno == ENOMEM) {
                    vTaskDelay(pdMS_TO_TICKS(20)); /* yield so lwIP can free pbufs */
                }
            } while (err < 0 && errno == ENOMEM);

            if (err < 0) {
                ESP_LOGE(TAG, "UDP send error: errno %d", errno);
            }
        }
    }
}

static void wifi_csi_rx_cb(void *ctx, wifi_csi_info_t *info) {

    if (!info || !info->buf) {
        ESP_LOGW(TAG, "<%s> wifi_csi_cb", esp_err_to_name(ESP_ERR_INVALID_ARG));
        return;
    }

    if (memcmp(info->mac, ctx, 6)) {
        return;
    }

    static uint32_t s_count = 0;
    if (++s_count % 5 != 0) return;

    csi_packet packet;
    memset(&packet, 0, sizeof(packet));

    packet.local_timestamp = (uint32_t)(esp_timer_get_time() & 0xFFFFFFFF);
    memcpy(packet.mac, info->mac, 6);
    packet.rssi            = info->rx_ctrl.rssi;
    packet.rate            = info->rx_ctrl.rate;
    packet.noise_floor     = info->rx_ctrl.noise_floor;
    packet.channel         = info->rx_ctrl.channel;
    packet.sig_len         = info->rx_ctrl.sig_len;
    packet.rx_state        = info->rx_ctrl.rx_state;
    packet.len             = info->len;
    packet.first_word_invalid = info->first_word_invalid;
    packet.csi_len         = info->len < MAX_CSI_LEN * (int)sizeof(int16_t)
                            ? info->len
                            : MAX_CSI_LEN * (int)sizeof(int16_t);
    memcpy(packet.csi_data, info->buf, packet.csi_len);

    BaseType_t xHigherPriorityTaskWoken = pdFALSE;
    xQueueSendFromISR(csi_queue, &packet, &xHigherPriorityTaskWoken);
    if (xHigherPriorityTaskWoken) {
        portYIELD_FROM_ISR();
    }

/*
#if CONFIG_GAIN_CONTROL
    static uint8_t agc_gain_baseline = 0;
    static int8_t fft_gain_baseline = 0;
    esp_csi_gain_ctrl_get_rx_gain(rx_ctrl, &agc_gain, &fft_gain);
    if (s_count < 100) {
        esp_csi_gain_ctrl_record_rx_gain(agc_gain, fft_gain);
    } else if (s_count == 100) {
        esp_csi_gain_ctrl_get_rx_gain_baseline(&agc_gain_baseline, &fft_gain_baseline);
#if CONFIG_FORCE_GAIN
        esp_csi_gain_ctrl_set_rx_force_gain(agc_gain_baseline, fft_gain_baseline);
        ESP_LOGI(TAG, "fft_force %d, agc_force %d", fft_gain_baseline, agc_gain_baseline);
#endif
    }
    esp_csi_gain_ctrl_get_gain_compensation(&compensate_gain, agc_gain, fft_gain);
    ESP_LOGD(TAG, "compensate_gain %f, agc_gain %d, fft_gain %d", compensate_gain, agc_gain, fft_gain);
#endif

    char buf[CSI_BUF_SIZE];
    int offset = 0;

    // Send CSV header once on first packet 
#if CONFIG_IDF_TARGET_ESP32C5 || CONFIG_IDF_TARGET_ESP32C6 || CONFIG_IDF_TARGET_ESP32C61
    if (!s_count) {
        ESP_LOGI(TAG, "================ CSI RECV ================");
        const char *header = "type,seq,mac,rssi,rate,noise_floor,fft_gain,agc_gain,channel,local_timestamp,sig_len,rx_state,len,first_word,data\n";
        send_csi_udp(header, strlen(header));
    }
    offset += snprintf(buf + offset, sizeof(buf) - offset,
                    "CSI_DATA,%d," MACSTR ",%d,%d,%d,%d,%d,%d,%d,%d,%d",
                    s_count, MAC2STR(info->mac), rx_ctrl->rssi, rx_ctrl->rate,
                    rx_ctrl->noise_floor, fft_gain, agc_gain, rx_ctrl->channel,
                    rx_ctrl->timestamp, rx_ctrl->sig_len, rx_ctrl->rx_state);
#else
    if (!s_count) {
        ESP_LOGI(TAG, "================ CSI RECV ================");
        const char *header = "type,id,mac,rssi,rate,sig_mode,mcs,bandwidth,smoothing,not_sounding,aggregation,stbc,fec_coding,sgi,noise_floor,ampdu_cnt,channel,secondary_channel,local_timestamp,ant,sig_len,rx_state,len,first_word,data\n";
        send_csi_udp(header, strlen(header));
    }
    offset += snprintf(buf + offset, sizeof(buf) - offset,
        "CSI_DATA,%d," MACSTR ",%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d",
        s_count, MAC2STR(info->mac), rx_ctrl->rssi, rx_ctrl->rate, rx_ctrl->sig_mode,
        rx_ctrl->mcs, rx_ctrl->cwb, rx_ctrl->smoothing, rx_ctrl->not_sounding,
        rx_ctrl->aggregation, rx_ctrl->stbc, rx_ctrl->fec_coding, rx_ctrl->sgi,
        rx_ctrl->noise_floor, rx_ctrl->ampdu_cnt, rx_ctrl->channel, rx_ctrl->secondary_channel,
        rx_ctrl->timestamp, rx_ctrl->ant, rx_ctrl->sig_len, rx_ctrl->rx_state);
#endif

    // Append CSI data array 
#if (CONFIG_IDF_TARGET_ESP32C5 || CONFIG_IDF_TARGET_ESP32C61) && CSI_FORCE_LLTF
    int16_t csi = ((int16_t)(((((uint16_t)info->buf[1]) << 8) | info->buf[0]) << 4) >> 4);
    offset += snprintf(buf + offset, sizeof(buf) - offset,
                       ",%d,%d,\"[%d", (info->len - 2) / 2, info->first_word_invalid,
                       (int16_t)(compensate_gain * csi));
    for (int i = 2; i < (info->len - 2) && offset < (int)(sizeof(buf) - 8); i += 2) {
        csi = ((int16_t)(((((uint16_t)info->buf[i + 1]) << 8) | info->buf[i]) << 4) >> 4);
        offset += snprintf(buf + offset, sizeof(buf) - offset, ",%d", (int16_t)(compensate_gain * csi));
    }
#else
    offset += snprintf(buf + offset, sizeof(buf) - offset,
            ",%d,%d,\"[%d", info->len, info->first_word_invalid,
            (int16_t)(compensate_gain * info->buf[0]));
    for (int i = 1; i < info->len && offset < (int)(sizeof(buf) - 8); i++) {
        offset += snprintf(buf + offset, sizeof(buf) - offset, ",%d", (int16_t)(compensate_gain * info->buf[i]));
    }
#endif

    offset += snprintf(buf + offset, sizeof(buf) - offset, "]\"\n");

    // Send complete CSI line via UDP 
    send_csi_udp(buf, offset);

    s_count++;
*/
}

static void wifi_csi_init() {
    /**
     * @brief In order to ensure the compatibility of routers, only LLTF sub-carriers are selected.
     */
#if CONFIG_IDF_TARGET_ESP32C5 || CONFIG_IDF_TARGET_ESP32C61
    wifi_csi_config_t csi_config = {
        .enable                   = true,
        .acquire_csi_legacy       = true,
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
        .acquire_csi_legacy     = true,
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
        .htltf_en          = false,
        .stbc_htltf2_en    = false,
        .ltf_merge_en      = true,
        .channel_filter_en = true,
        .manu_scale        = true,
        .shift             = true,
    };
#endif
    static wifi_ap_record_t s_ap_info = {0};
    ESP_ERROR_CHECK(esp_wifi_sta_get_ap_info(&s_ap_info));
    ESP_ERROR_CHECK(esp_wifi_set_csi_config(&csi_config));
    ESP_ERROR_CHECK(esp_wifi_set_csi_rx_cb(wifi_csi_rx_cb, s_ap_info.bssid));
    ESP_ERROR_CHECK(esp_wifi_set_csi(true));
}

static esp_err_t wifi_ping_router_start() {
    static esp_ping_handle_t ping_handle = NULL;

    esp_ping_config_t ping_config = ESP_PING_DEFAULT_CONFIG();
    ping_config.count             = 0;
    ping_config.interval_ms       = 1000 / CONFIG_SEND_FREQUENCY;
    ping_config.task_stack_size   = 3072;
    ping_config.data_size         = 1;

    esp_netif_ip_info_t local_ip;
    esp_netif_get_ip_info(esp_netif_get_handle_from_ifkey("WIFI_STA_DEF"), &local_ip);
    ESP_LOGI(TAG, "got ip:" IPSTR ", gw: " IPSTR, IP2STR(&local_ip.ip), IP2STR(&local_ip.gw));
    ping_config.target_addr.u_addr.ip4.addr = ip4_addr_get_u32(&local_ip.gw);
    ping_config.target_addr.type = ESP_IPADDR_TYPE_V4;

    esp_ping_callbacks_t cbs = { 0 };
    esp_ping_new_session(&ping_config, &cbs, &ping_handle);
    esp_ping_start(ping_handle);

    return ESP_OK;
}


void app_main() {

    ESP_ERROR_CHECK(nvs_flash_init());
    ESP_ERROR_CHECK(esp_netif_init());
    ESP_ERROR_CHECK(esp_event_loop_create_default());

    /**
     * @brief This helper function configures Wi-Fi, as selected in menuconfig.
     *        Read "Establishing Wi-Fi Connection" section in esp-idf/examples/protocols/README.md
     *        for more information about this function.
     */
    ESP_ERROR_CHECK(example_connect());

    esp_wifi_set_ps(WIFI_PS_NONE);

    /* Create queue */
    csi_queue = xQueueCreate(CSI_QUEUE_SIZE, sizeof(csi_packet));
    if (!csi_queue) {
        ESP_LOGE(TAG, "Failed to create CSI queue");
        return;
    }

    /* Init UDP */
    udp_init();

    printf("Connected to Wi-Fi network \"%s\", starting CSI collection...\n", ESP_WIFI_SSID);

    /* Start UDP task */
    xTaskCreatePinnedToCore(
        csi_udp_task,
        "csi_udp_task",
        4096,
        NULL,
        5,
        NULL,
        tskNO_AFFINITY
    );

    /* Init CSI */
    wifi_csi_init();

    /* Start pinging the router to generate CSI frames */
    ESP_ERROR_CHECK(wifi_ping_router_start());

    ESP_LOGI(TAG, "System ready. Collecting CSI...");
}
