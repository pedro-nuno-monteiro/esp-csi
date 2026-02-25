#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include <errno.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/event_groups.h"
#include "freertos/queue.h"

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

#include "protocol_examples_common.h"
#include "esp_csi_gain_ctrl.h"

#define CONFIG_SEND_FREQUENCY     1    /* CSI/ping rate in Hz */
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

static const char *TAG = "csi_recv_router";

#define UDP_SERVER_IP           "193.136.94.101"   /* Change to your UDP server IP */
#define UDP_SERVER_PORT         5001              /* Change to your UDP server port */
#define UDP_MAX_CSI_PACKET_SIZE 1024

typedef struct {
    size_t len;
    char   data[UDP_MAX_CSI_PACKET_SIZE];
} csi_udp_msg_t;

static QueueHandle_t s_csi_udp_queue = NULL;
static int s_udp_sock = -1;
static struct sockaddr_in s_udp_dest_addr;
static uint8_t s_sta_mac[6] = {0};

static void csi_udp_sender_task(void *arg)
{
    for (;;) {
        csi_udp_msg_t msg;
        if (xQueueReceive(s_csi_udp_queue, &msg, portMAX_DELAY) == pdTRUE) {
            if (s_udp_sock < 0) {
                s_udp_sock = socket(AF_INET, SOCK_DGRAM, IPPROTO_IP);
                if (s_udp_sock < 0) {
                    ESP_LOGE(TAG, "Unable to create UDP socket: errno %d", errno);
                    vTaskDelay(pdMS_TO_TICKS(100));
                    continue;
                }
            }

            int retries = 3;
            int err = -1;
            while (retries-- > 0) {
                err = sendto(s_udp_sock, msg.data, msg.len, 0,
                             (struct sockaddr *)&s_udp_dest_addr,
                             sizeof(s_udp_dest_addr));
                if (err >= 0) {
                    ESP_LOGI(TAG, "CSI UDP packet sent (%d bytes)", err);
                    break;
                }

                if (errno == ENOMEM) {
                    /* Transient out-of-memory in UDP stack, back off briefly and retry */
                    vTaskDelay(pdMS_TO_TICKS(5));
                } else {
                    ESP_LOGE(TAG, "UDP send failed: errno %d", errno);
                    break;
                }
            }

            if (err < 0 && errno == ENOMEM) {
                static uint32_t s_udp_enomem_count = 0;
                s_udp_enomem_count++;
                if ((s_udp_enomem_count % 100) == 0) {
                    ESP_LOGW(TAG, "UDP ENOMEM occurred %u times (dropping packets)", (unsigned int)s_udp_enomem_count);
                }
            }
        }
    }
}

static void udp_sender_init(void)
{
    if (s_csi_udp_queue) {
        return;
    }

    s_csi_udp_queue = xQueueCreate(32, sizeof(csi_udp_msg_t));
    if (!s_csi_udp_queue) {
        ESP_LOGE(TAG, "Failed to create CSI UDP queue");
        return;
    }

    memset(&s_udp_dest_addr, 0, sizeof(s_udp_dest_addr));
    s_udp_dest_addr.sin_family      = AF_INET;
    s_udp_dest_addr.sin_port        = htons(UDP_SERVER_PORT);
    s_udp_dest_addr.sin_addr.s_addr = inet_addr(UDP_SERVER_IP);

    BaseType_t ok = xTaskCreatePinnedToCore(
        csi_udp_sender_task,
        "csi_udp_sender",
        4096,
        NULL,
        tskIDLE_PRIORITY + 3,
        NULL,
        tskNO_AFFINITY
    );

    if (ok != pdPASS) {
        ESP_LOGE(TAG, "Failed to create CSI UDP sender task");
        vQueueDelete(s_csi_udp_queue);
        s_csi_udp_queue = NULL;
    }
}

static void wifi_csi_rx_cb(void *ctx, wifi_csi_info_t *info)
{
    if (!info || !info->buf) {
        ESP_LOGW(TAG, "<%s> wifi_csi_cb", esp_err_to_name(ESP_ERR_INVALID_ARG));
        return;
    }

    if (memcmp(info->mac, ctx, 6)) {
        return;
    }

    const wifi_pkt_rx_ctrl_t *rx_ctrl = &info->rx_ctrl;
    static int s_count = 0;
    float compensate_gain = 1.0f;
    static uint8_t agc_gain = 0;
    static int8_t fft_gain = 0;
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

#if CONFIG_IDF_TARGET_ESP32C5 || CONFIG_IDF_TARGET_ESP32C6 || CONFIG_IDF_TARGET_ESP32C61
    if (!s_count) {
        ESP_LOGI(TAG, "================ CSI RECV ================");
        ets_printf("type,seq,mac,rssi,rate,noise_floor,fft_gain,agc_gain,channel,local_timestamp,sig_len,rx_state,len,first_word,data\n");
    }
    ets_printf("CSI_DATA,%d," MACSTR ",%d,%d,%d,%d,%d,%d,%d,%d,%d",
               s_count, MAC2STR(info->mac), rx_ctrl->rssi, rx_ctrl->rate,
               rx_ctrl->noise_floor, fft_gain, agc_gain, rx_ctrl->channel,
               rx_ctrl->timestamp, rx_ctrl->sig_len, rx_ctrl->rx_state);
#else
    if (!s_count) {
        ESP_LOGI(TAG, "================ CSI RECV ================");
        ets_printf("type,id,mac,rssi,rate,sig_mode,mcs,bandwidth,smoothing,not_sounding,aggregation,stbc,fec_coding,sgi,noise_floor,ampdu_cnt,channel,secondary_channel,local_timestamp,ant,sig_len,rx_state,len,first_word,data\n");
    }
    ets_printf("CSI_DATA,%d," MACSTR ",%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d",
               s_count, MAC2STR(info->mac), rx_ctrl->rssi, rx_ctrl->rate, rx_ctrl->sig_mode,
               rx_ctrl->mcs, rx_ctrl->cwb, rx_ctrl->smoothing, rx_ctrl->not_sounding,
               rx_ctrl->aggregation, rx_ctrl->stbc, rx_ctrl->fec_coding, rx_ctrl->sgi,
               rx_ctrl->noise_floor, rx_ctrl->ampdu_cnt, rx_ctrl->channel, rx_ctrl->secondary_channel,
               rx_ctrl->timestamp, rx_ctrl->ant, rx_ctrl->sig_len, rx_ctrl->rx_state);
#endif

#if (CONFIG_IDF_TARGET_ESP32C5 || CONFIG_IDF_TARGET_ESP32C61) && CSI_FORCE_LLTF

    int16_t csi = ((int16_t)(((((uint16_t)info->buf[1]) << 8) | info->buf[0]) << 4) >> 4);
    ets_printf(",%d,%d,\"[%d", (info->len - 2) / 2, info->first_word_invalid, (int16_t)(compensate_gain * csi));
    for (int i = 2; i < (info->len - 2); i += 2) {
        csi = ((int16_t)(((((uint16_t)info->buf[i + 1]) << 8) | info->buf[i]) << 4) >> 4);
        ets_printf(",%d", (int16_t)(compensate_gain * csi));
    }

#else
    ets_printf(",%d,%d,\"[%d", info->len, info->first_word_invalid, (int16_t)(compensate_gain * info->buf[0]));
    for (int i = 1; i < info->len; i++) {
        ets_printf(",%d", (int16_t)(compensate_gain * info->buf[i]));
    }
#endif
    ets_printf("]\"\n");

    /* Build the same CSV line into a buffer and enqueue for UDP send in a task */
    char udp_buf[UDP_MAX_CSI_PACKET_SIZE];
    int len = 0;

#if CONFIG_IDF_TARGET_ESP32C5 || CONFIG_IDF_TARGET_ESP32C6 || CONFIG_IDF_TARGET_ESP32C61
    len = snprintf(udp_buf, sizeof(udp_buf),
                   "%02X:%02X:%02X:%02X:%02X:%02X,CSI_DATA,%d," MACSTR ",%d,%d,%d,%d,%d,%d,%d,%d,%d",
                   s_sta_mac[0], s_sta_mac[1], s_sta_mac[2],
                   s_sta_mac[3], s_sta_mac[4], s_sta_mac[5],
                   s_count, MAC2STR(info->mac), rx_ctrl->rssi, rx_ctrl->rate,
                   rx_ctrl->noise_floor, fft_gain, agc_gain, rx_ctrl->channel,
                   rx_ctrl->timestamp, rx_ctrl->sig_len, rx_ctrl->rx_state);
#else
    len = snprintf(udp_buf, sizeof(udp_buf),
                   "%02X:%02X:%02X:%02X:%02X:%02X,CSI_DATA,%d," MACSTR ",%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d",
                   s_sta_mac[0], s_sta_mac[1], s_sta_mac[2],
                   s_sta_mac[3], s_sta_mac[4], s_sta_mac[5],
                   s_count, MAC2STR(info->mac), rx_ctrl->rssi, rx_ctrl->rate, rx_ctrl->sig_mode,
                   rx_ctrl->mcs, rx_ctrl->cwb, rx_ctrl->smoothing, rx_ctrl->not_sounding,
                   rx_ctrl->aggregation, rx_ctrl->stbc, rx_ctrl->fec_coding, rx_ctrl->sgi,
                   rx_ctrl->noise_floor, rx_ctrl->ampdu_cnt, rx_ctrl->channel, rx_ctrl->secondary_channel,
                   rx_ctrl->timestamp, rx_ctrl->ant, rx_ctrl->sig_len, rx_ctrl->rx_state);
#endif

    if (len > 0 && len < (int)sizeof(udp_buf)) {
#if (CONFIG_IDF_TARGET_ESP32C5 || CONFIG_IDF_TARGET_ESP32C61) && CSI_FORCE_LLTF
        int16_t csi_udp = ((int16_t)(((((uint16_t)info->buf[1]) << 8) | info->buf[0]) << 4) >> 4);
        len += snprintf(udp_buf + len, sizeof(udp_buf) - len, ",%d,%d,\"[%d",
                        (info->len - 2) / 2, info->first_word_invalid,
                        (int16_t)(compensate_gain * csi_udp));
        for (int i = 2; i < (info->len - 2) && len < (int)sizeof(udp_buf); i += 2) {
            csi_udp = ((int16_t)(((((uint16_t)info->buf[i + 1]) << 8) | info->buf[i]) << 4) >> 4);
            len += snprintf(udp_buf + len, sizeof(udp_buf) - len, ",%d",
                            (int16_t)(compensate_gain * csi_udp));
        }
#else
        len += snprintf(udp_buf + len, sizeof(udp_buf) - len, ",%d,%d,\"[%d",
                        info->len, info->first_word_invalid,
                        (int16_t)(compensate_gain * info->buf[0]));
        for (int i = 1; i < info->len && len < (int)sizeof(udp_buf); i++) {
            len += snprintf(udp_buf + len, sizeof(udp_buf) - len, ",%d",
                            (int16_t)(compensate_gain * info->buf[i]));
        }
#endif
        if (len < (int)sizeof(udp_buf)) {
            len += snprintf(udp_buf + len, sizeof(udp_buf) - len, "]\"\n");
        }

        if (len > 0 && s_csi_udp_queue) {
            csi_udp_msg_t msg = { 0 };
            msg.len = (size_t)len;
            if (msg.len >= UDP_MAX_CSI_PACKET_SIZE) {
                msg.len = UDP_MAX_CSI_PACKET_SIZE - 1;
            }
            memcpy(msg.data, udp_buf, msg.len);
            msg.data[msg.len] = '\0';
            if (xQueueSend(s_csi_udp_queue, &msg, 0) != pdPASS) {
                static uint32_t s_udp_drop_count = 0;
                s_udp_drop_count++;
                if ((s_udp_drop_count % 100) == 0) {
                    ESP_LOGW(TAG, "CSI UDP queue full, dropped %u messages", (unsigned int)s_udp_drop_count);
                }
            }
        }
    }

    s_count++;
}

static void wifi_csi_init()
{
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
        .acquire_csi_vht          = true,
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

static esp_err_t wifi_ping_router_start()
{
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

void app_main()
{
    ESP_ERROR_CHECK(nvs_flash_init());
    ESP_ERROR_CHECK(esp_netif_init());
    ESP_ERROR_CHECK(esp_event_loop_create_default());

    /**
     * @brief This helper function configures Wi-Fi, as selected in menuconfig.
     *        Read "Establishing Wi-Fi Connection" section in esp-idf/examples/protocols/README.md
     *        for more information about this function.
     */
    ESP_ERROR_CHECK(example_connect());

    ESP_ERROR_CHECK(esp_wifi_get_mac(WIFI_IF_STA, s_sta_mac));
    ESP_LOGI(TAG, "STA MAC: " MACSTR, MAC2STR(s_sta_mac));

    udp_sender_init();
    wifi_csi_init();
    wifi_ping_router_start();
}
