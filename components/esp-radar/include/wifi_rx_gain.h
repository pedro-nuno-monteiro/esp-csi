/*
 * SPDX-FileCopyrightText: 2025 Espressif Systems (Shanghai) CO LTD
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#pragma once

#include <stdint.h>
#include <stdbool.h>

#include "esp_err.h"
#include "esp_wifi.h"

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief 当前是否处于“自动 RX 增益被强制设置”的状态
 *
 * @return true  已经通过 esp_radar_set_rx_force_gain() 强制设置了增益
 * @return false 当前使用正常 AGC 自动增益
 */
bool esp_radar_auto_rx_gain_status(void);

/**
 * @brief 获取 RX 增益的“基线”值（中位数）
 *
 * 需要先通过多次调用 esp_radar_record_rx_gain() 采样。
 *
 * @param[out] agc_gain 基线 AGC 增益
 * @param[out] fft_gain 基线 FFT 增益
 *
 * @return
 *      - ESP_OK: 成功
 *      - ESP_ERR_INVALID_ARG: 参数非法
 *      - ESP_ERR_INVALID_STATE: 基线尚未准备好
 *      - ESP_ERR_NO_MEM: 内存不足
 */
esp_err_t esp_radar_get_rx_gain_baseline(uint8_t *agc_gain, int8_t *fft_gain);

/**
 * @brief 记录当前一次 RX 增益，用于后续计算基线
 *
 * @param agc_gain 当前 AGC 增益
 * @param fft_gain 当前 FFT 增益
 *
 * @return ESP_OK
 */
esp_err_t esp_radar_record_rx_gain(uint8_t agc_gain, int8_t fft_gain);

/**
 * @brief 强制设置接收增益（可能会导致丢包）
 *
 * 若 agc_gain 和 fft_gain 都为 0，则关闭强制增益，恢复自动增益。
 *
 * @param agc_gain AGC 增益
 * @param fft_gain FFT 增益
 *
 * @return
 *      - ESP_OK
 *      - ESP_ERR_INVALID_STATE: agc_gain 太小会影响数据收发
 */
esp_err_t esp_radar_set_rx_force_gain(uint8_t agc_gain, int8_t fft_gain);

/**
 * @brief 重置 RX 增益基线统计
 */
void esp_radar_reset_rx_gain_baseline(void);

/**
 * @brief 计算针对当前 RX 增益的补偿因子
 *
 * @param[out] compensate_gain 计算得到的补偿系数
 * @param agc_gain 当前 AGC 增益
 * @param fft_gain 当前 FFT 增益
 *
 * @return
 *      - ESP_OK
 *      - ESP_ERR_INVALID_STATE: 基线尚未准备好或其他状态异常
 */
esp_err_t esp_radar_get_gain_compensation(float *compensate_gain, uint8_t agc_gain, int8_t fft_gain);

/**
 * @brief Extended version of RX gain compensation supporting 8-bit and 16-bit CSI samples.
 *
 * @param data               Data buffer to be compensated (in-place).
 * @param size               Buffer length in bytes.
 * @param samples_are_16bit  true if every CSI component uses 16 bits, false for 8-bit data.
 * @param compensate_gain    Output compensation factor.
 * @param agc_gain           Current AGC gain.
 * @param fft_gain           Current FFT gain.
 */
esp_err_t esp_radar_compensate_rx_gain(void *data, uint16_t size, bool samples_are_16bit,
                                       float *compensate_gain, uint8_t agc_gain, int8_t fft_gain);

/**
 * @brief 从 CSI 包信息中解析出 RX 增益
 *
 * @param rx_ctrl      RX 控制信息指针（wifi_pkt_rx_ctrl_t）
 * @param[out] agc_gain 解析出的 AGC 增益
 * @param[out] fft_gain 解析出的 FFT 增益
 */
void esp_radar_get_rx_gain(const wifi_pkt_rx_ctrl_t *rx_ctrl, uint8_t *agc_gain, int8_t *fft_gain);

#ifdef __cplusplus
}
#endif
