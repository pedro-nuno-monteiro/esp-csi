# esp-radar Component

[![Component Registry](https://components.espressif.com/components/espressif/esp-radar/badge.svg)](https://components.espressif.com/components/espressif/esp-radar)

- [User Guide](https://github.com/espressif/esp-csi/tree/master/README.md)

WiFi CSI (Channel State Information) refers to the information obtained from analyzing changes in WiFi signals. This project provides examples of the ESP-CSI motion detection algorithm

### Add component to your project
Please use the component manager command `add-dependency` to add the `esp-radar` to your project's dependency, during the `CMake` step the component will be downloaded automatically.

```
idf.py add-dependency "espressif/esp-radar=*"
```

## Example
Please use the component manager command `create-project-from-example` to create the project from example template.

```
idf.py create-project-from-example "espressif/esp-radar=*:console_test"
```

Then the example will be downloaded in current folder, you can check into it for build and flash.

> You can use this command to download other examples. Or you can download examples from esp-radar repository:

 - [connect_rainmaker](https://github.com/espressif/esp-csi/tree/master/examples/esp-radar/connect_rainmaker): Adding Wi-Fi CSI Functionality in ESP RainMaker
 - [console_test](https://github.com/espressif/esp-csi/tree/master/examples/esp-radar/console_test): This example provides a test platform for Wi-Fi CSI, which includes functions such as data display, data acquisition and data analysis, which can help you quickly understand Wi-Fi CSI

### Q&A
Q1. I encountered the following problems when using the package manager

```
  HINT: Please check manifest file of the following component(s): main

  ERROR: Because project depends on esp-radar (2.*) which doesn't match any
  versions, version solving failed.
```

A1. For the examples downloaded by using this command, you need to comment out the override_path line in the main/idf_component.yml of each example.

Q2. I encountered the following problems when using the package manager

```
Executing action: create-project-from-example
CMakeLists.txt not found in project directory /home/username
```

A2. This is because an older version package manager was used, please run `pip install -U idf-component-manager` in ESP-IDF environment to update.


# Project Structure
功能规划：
## 1、不同的芯片会有不同的配置项
### if CONFIG_IDF_TARGET_ESP32C5
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
### elif CONFIG_IDF_TARGET_ESP32C6
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
### else
    wifi_csi_config_t csi_config = {
        .lltf_en           = true,
        .htltf_en          = true,
        .stbc_htltf2_en    = false,
        .ltf_merge_en      = false,
        .channel_filter_en = false,
        .manu_scale        = false,
        .shift             = false,
    };
#endif

## 2、不同的芯片回调函数的结构不同
### （这是S2，S3，C3,32）
```
/** @brief Received packet radio metadata header, this is the common header at the beginning of all promiscuous mode RX callback buffers */
typedef struct {
    signed rssi: 8;               /**< Received Signal Strength Indicator(RSSI) of packet. unit: dBm */
    unsigned rate: 5;             /**< PHY rate encoding of the packet. Only valid for non HT(11bg) packet */
    unsigned : 1;                 /**< reserved */
    unsigned sig_mode: 2;         /**< Protocol of the received packet, 0: non HT(11bg) packet; 1: HT(11n) packet; 3: VHT(11ac) packet */
    unsigned : 16;                /**< reserved */
    unsigned mcs: 7;              /**< Modulation Coding Scheme. If is HT(11n) packet, shows the modulation, range from 0 to 76(MSC0 ~ MCS76) */
    unsigned cwb: 1;              /**< Channel Bandwidth of the packet. 0: 20MHz; 1: 40MHz */
    unsigned : 16;                /**< reserved */
    unsigned smoothing: 1;         /**< Set to 1 indicates that channel estimate smoothing is recommended.
                                       Set to 0 indicates that only per-carrierindependent (unsmoothed) channel estimate is recommended. */
    unsigned not_sounding: 1;      /**< Set to 0 indicates that PPDU is a sounding PPDU. Set to 1indicates that the PPDU is not a sounding PPDU.
                                       sounding PPDU is used for channel estimation by the request receiver */
    unsigned : 1;                 /**< reserved */
    unsigned aggregation: 1;      /**< Aggregation. 0: MPDU packet; 1: AMPDU packet */
    unsigned stbc: 2;             /**< Space Time Block Code(STBC). 0: non STBC packet; 1: STBC packet */
    unsigned fec_coding: 1;       /**< Forward Error Correction(FEC). Flag is set for 11n packets which are LDPC */
    unsigned sgi: 1;              /**< Short Guide Interval(SGI). 0: Long GI; 1: Short GI */
#if CONFIG_IDF_TARGET_ESP32
    signed noise_floor: 8;        /**< noise floor of Radio Frequency Module(RF). unit: dBm*/
#elif CONFIG_IDF_TARGET_ESP32S2 || CONFIG_IDF_TARGET_ESP32S3 || CONFIG_IDF_TARGET_ESP32C3 || CONFIG_IDF_TARGET_ESP32C2
    unsigned : 8;                 /**< reserved */
#endif
    unsigned ampdu_cnt: 8;        /**< the number of subframes aggregated in AMPDU */
    unsigned channel: 4;          /**< primary channel on which this packet is received */
    unsigned secondary_channel: 4; /**< secondary channel on which this packet is received. 0: none; 1: above; 2: below */
    unsigned : 8;                 /**< reserved */
    unsigned timestamp: 32;       /**< timestamp. The local time when this packet is received. It is precise only if modem sleep or light sleep is not enabled. unit: microsecond */
    unsigned : 32;                /**< reserved */
#if CONFIG_IDF_TARGET_ESP32S2
    unsigned : 32;                /**< reserved */
#elif CONFIG_IDF_TARGET_ESP32S3 || CONFIG_IDF_TARGET_ESP32C3 || CONFIG_IDF_TARGET_ESP32C2
    signed noise_floor: 8;        /**< noise floor of Radio Frequency Module(RF). unit: dBm*/
    unsigned : 24;                /**< reserved */
    unsigned : 32;                /**< reserved */
#endif
    unsigned : 31;                /**< reserved */
    unsigned ant: 1;              /**< antenna number from which this packet is received. 0: WiFi antenna 0; 1: WiFi antenna 1 */
#if CONFIG_IDF_TARGET_ESP32S2
    signed noise_floor: 8;        /**< noise floor of Radio Frequency Module(RF). unit: dBm*/
    unsigned : 24;                /**< reserved */
#elif CONFIG_IDF_TARGET_ESP32S3 || CONFIG_IDF_TARGET_ESP32C3 || CONFIG_IDF_TARGET_ESP32C2
    unsigned : 32;                /**< reserved */
    unsigned : 32;                /**< reserved */
    unsigned : 32;                /**< reserved */
#endif
    unsigned sig_len: 12;         /**< length of packet including Frame Check Sequence(FCS) */
    unsigned : 12;                /**< reserved */
    unsigned rx_state: 8;         /**< state of the packet. 0: no error; others: error numbers which are not public */
} wifi_pkt_rx_ctrl_t;
#endif
```
### （这是C5）
```
typedef struct {
    signed rssi: 8;                               /**< the RSSI of the reception frame */
    unsigned rate: 5;                             /**< if cur_bb_format is RX_BB_FORMAT_11B, it's the transmission rate. otherwise it's Rate field of L-SIG */
    unsigned : 1;                                 /**< reserved */
    unsigned : 2;                                 /**< reserved */
    unsigned : 12;                                /**< reserved */
    unsigned rxmatch0: 1;                         /**< indicate whether the reception frame is from interface 0 */
    unsigned rxmatch1: 1;                         /**< indicate whether the reception frame is from interface 1 */
    unsigned rxmatch2: 1;                         /**< indicate whether the reception frame is from interface 2 */
    unsigned rxmatch3: 1;                         /**< indicate whether the reception frame is from interface 3 */
    uint32_t he_siga1;                            /**< HE-SIGA1 or HT-SIG or VHT-SIG */
    unsigned rxend_state: 8;                      /**< reception state, 0: successful, others: failure */
    uint16_t he_siga2;                            /**< HE-SIGA2 */
    unsigned : 7;                                 /**< reserved */
    unsigned is_group: 1;                         /**< indicate whether the reception is a group addressed frame */
    unsigned timestamp: 32;                       /**< timestamp. The local time when this packet is received. It is precise only if modem sleep or light sleep is not enabled. unit: microsecond */
    unsigned : 15;                                /**< reserved */
    unsigned : 15;                                /**< reserved */
    unsigned : 2;                                 /**< reserved */
    signed noise_floor: 8;                        /**< the noise floor of the reception frame */
    signed : 8;                                   /**< reserved */
    signed : 8;                                   /**< reserved */
    unsigned : 8;                                 /**< reserved */
    unsigned : 8;                                 /**< reserved */
    unsigned : 8;                                 /**< reserved */
    unsigned : 2;                                 /**< reserved */
    unsigned sigb_len: 10;                        /**< the sigb length */
    unsigned : 1;                                 /**< reserved */
    unsigned : 1;                                 /**< reserved */
    unsigned : 1;                                 /**< reserved */
    unsigned : 1;                                 /**< reserved */
    unsigned channel: 8;                          /**< the primary channel */
    unsigned second: 8;                           /**< the second channel if in HT40 */
    unsigned : 4;                                 /**< reserved */
    unsigned : 4;                                 /**< reserved */
    unsigned : 1;                                 /**< reserved */
    unsigned : 7;                                 /**< reserved */
    unsigned : 2;                                 /**< reserved */
    unsigned : 4;                                 /**< reserved */
    unsigned : 2;                                 /**< reserved */
    unsigned : 11;                                /**< reserved */
    unsigned : 1;                                 /**< reserved */
    unsigned : 12;                                /**< reserved */
    unsigned : 12;                                /**< reserved */
    unsigned cur_bb_format: 4;                    /**< the format of the reception frame */
    unsigned rx_channel_estimate_len: 10;         /**< the length of the channel information */
    unsigned rx_channel_estimate_info_vld: 1;     /**< indicate the channel information is valid */
    unsigned : 5;                                 /**< reserved */
    unsigned : 21;                                /**< reserved */
    unsigned : 10;                                /**< reserved */
    unsigned : 1;                                 /**< reserved */
    unsigned : 3;                                 /**< reserved */
    unsigned : 1;                                 /**< reserved */
    unsigned : 6;                                 /**< reserved */
    unsigned : 21;                                /**< reserved */
    unsigned : 1;                                 /**< reserved */
    unsigned : 32;                                /**< reserved */
    unsigned : 7;                                 /**< reserved */
    unsigned : 1;                                 /**< reserved */
    unsigned : 8;                                 /**< reserved */
    unsigned : 16;                                /**< reserved */
    unsigned sig_len: 14;                         /**< the length of the reception MPDU */
    unsigned : 2;                                 /**< reserved */
    unsigned dump_len: 14;                        /**< the length of the reception MPDU excluding the FCS */
    unsigned : 2;                                 /**< reserved */
    unsigned rx_state: 8;                         /**< reception state, 0: successful, others: failure */
    unsigned : 8;                                 /**< reserved */
    unsigned : 16;                                /**< reserved */
} __attribute__((packed)) esp_wifi_rxctrl_t;
```
### （这是C6）
```
typedef struct {
    signed rssi : 8;                              /**< the RSSI of the reception frame */
    unsigned rate : 5;                            /**< if cur_bb_format is RX_BB_FORMAT_11B, it's the transmission rate. otherwise it's Rate field of L-SIG */
    unsigned : 1;                                 /**< reserved */
    unsigned : 2;                                 /**< reserved */
    unsigned : 12;                                /**< reserved */
    unsigned rxmatch0 : 1;                        /**< indicate whether the reception frame is from interface 0 */
    unsigned rxmatch1 : 1;                        /**< indicate whether the reception frame is from interface 1 */
    unsigned rxmatch2 : 1;                        /**< indicate whether the reception frame is from interface 2 */
    unsigned rxmatch3 : 1;                        /**< indicate whether the reception frame is from interface 3 */
    uint32_t he_siga1;                            /**< HE-SIGA1 or HT-SIG */
    unsigned rxend_state : 8;                     /**< reception state, 0: successful, others: failure */
    uint16_t he_siga2;                            /**< HE-SIGA2 */
    unsigned : 7;                                 /**< reserved */
    unsigned is_group : 1;                        /**< indicate whether the reception is a group addressed frame */
    unsigned timestamp : 32;                      /**< timestamp. The local time when this packet is received. It is precise only if modem sleep or light sleep is not enabled. unit: microsecond */
    unsigned : 15;                                /**< reserved */
    unsigned : 15;                                /**< reserved */
    unsigned : 2;                                 /**< reserved */
    signed noise_floor : 8;                       /**< the noise floor of the reception frame */
    unsigned channel : 4;                         /**< the primary channel */
    unsigned second : 4;                          /**< the second channel if in HT40 */
    unsigned : 8;                                 /**< reserved */
    unsigned : 8;                                 /**< reserved */
    unsigned : 32;                                /**< reserved */
    unsigned : 32;                                /**< reserved */
    unsigned : 2;                                 /**< reserved */
    unsigned : 4;                                 /**< reserved */
    unsigned : 2;                                 /**< reserved */
    unsigned rx_channel_estimate_len : 10;        /**< the length of the channel information */
    unsigned rx_channel_estimate_info_vld : 1;    /**< indicate the channel information is valid */
    unsigned : 1;                                 /**< reserved */
    unsigned : 11;                                /**< reserved */
    unsigned : 1;                                 /**< reserved */
    unsigned : 24;                                /**< reserved */
    unsigned cur_bb_format : 4;                   /**< the format of the reception frame */
    unsigned cur_single_mpdu : 1;                 /**< indicate whether the reception MPDU is a S-MPDU */
    unsigned : 3;                                 /**< reserved */
    unsigned : 32;                                /**< reserved */
    unsigned : 32;                                /**< reserved */
    unsigned : 32;                                /**< reserved */
    unsigned : 32;                                /**< reserved */
    unsigned : 32;                                /**< reserved */
    unsigned : 32;                                /**< reserved */
    unsigned : 32;                                /**< reserved */
    unsigned : 32;                                /**< reserved */
    unsigned : 8;                                 /**< reserved */
    unsigned he_sigb_len : 6;                     /**< the length of HE-SIGB */
    unsigned : 2;                                 /**< reserved */
    unsigned : 8;                                 /**< reserved */
    unsigned : 8;                                 /**< reserved */
    unsigned : 32;                                /**< reserved */
    unsigned : 7;                                 /**< reserved */
    unsigned : 1;                                 /**< reserved */
    unsigned : 8;                                 /**< reserved */
    unsigned : 16;                                /**< reserved */
    unsigned sig_len : 14;                        /**< the length of the reception MPDU */
    unsigned : 2;                                 /**< reserved */
    unsigned dump_len : 14;                       /**< the length of the reception MPDU excluding the FCS */
    unsigned : 2;                                 /**< reserved */
    unsigned rx_state : 8;                        /**< reception state, 0: successful, others: failure */
    unsigned : 24;                                /**< reserved */
} __attribute__((packed)) esp_wifi_rxctrl_t;
#endif
```
## 3.不同芯片的收数据包结构不同
### 这是C5
 .. only:: esp32c5

        信道状态信息 (CSI) 是指 Wi-Fi 连接的信道信息。{IDF_TARGET_NAME} 中，CSI 由子载波的信道频率响应组成，CSI 从发送端接收数据包时开始估计。每个子载波信道频率响由两个字节的有符号字符记录，第一个字节是虚部，第二个字节是实部。除了 IEEE 802.11g 模式外，其他模式均存在两段 LTF 序列（LLTF + HT/VHT/HE-LTF）。{IDF_TARGET_NAME}可以通过 :cpp:struct:`wifi_csi_acquire_config_t` 中 ``acquire_csi_force_lltf`` 字段决定包含 LLTF 或 HT/VHT/HE-LTF。对于在不同状态的信道上接收到的不同类型的数据包，CSI 的子载波索引和总字节数如下表所示。

        +------------+------------------+-----------------------------------------+--------------------------------------------+----------------------------------------------------------------+----------------------------------------------------------------+
        | 信道       | 辅助信道         |                                                                                      | 下                                                             | 上                                                             |
        +------------+------------------+-------------+---------------------------+--------------------------------------------+-------------+--------------------------------------------------+-------------+--------------------------------------------------+
        | 数据包信息 | 信号模式         | 非 HT       | HT                        | HE                                         | 非 HT       | HT                                               | 非 HT       | HT                                               |
        |            +------------------+-------------+---------------------------+--------------------------------------------+-------------+----------------------+---------------------------+-------------+----------------------+---------------------------+
        |            | 信道带宽         | 20 MHz      | 20 MHz                    | 20 MHz                                     | 20 MHz      | 20 MHz               | 40 MHz                    | 20 MHz      | 20 MHz               | 40 MHz                    |
        |            +------------------+-------------+-------------+-------------+---------------+----------------------------+-------------+-------------+--------+-------------+-------------+-------------+-------------+--------+-------------+-------------+
        |            | STBC             | 非 STBC     | 非 STBC     | STBC        | 非 STBC       | STBC                       | 非 STBC     | 非 STBC     | STBC   | 非 STBC     | STBC        | 非 STBC     | 非 STBC     | STBC   | 非 STBC     | STBC        |
        +------------+------------------+-------------+-------------+-------------+---------------+----------------------------+-------------+-------------+--------+-------------+-------------+-------------+-------------+--------+-------------+-------------+
        | 子载波索引 | LLTF             | 0~26,-26~-1 |           — |           — |             — |                          — |   0~52      |           — |      — |           — |           — |       -53~-1|           — |      — |           — |           — |
        |            +------------------+-------------+-------------+-------------+---------------+----------------------------+-------------+-------------+--------+-------------+-------------+-------------+-------------+--------+-------------+-------------+
        |            | HT-LTF (HT-LTF1) |           — | 0~28,-28~-1 | 0~28,-28~-1 |             — |                          — |           — |         0~56|  0~56  | 0~58,-58~-1 | 0~58,-58~-1 |           — |       -57~-1| -57~-1 | 0~58,-58~-1 | 0~58,-58~-1 |
        |            +------------------+-------------+-------------+-------------+---------------+----------------------------+-------------+-------------+--------+-------------+-------------+-------------+-------------+--------+-------------+-------------+
        |            | HT-LTF2          |           — |           — | 0~28,-28~-1 |             — |                          — |           — |           — |  0~56  |           — | 0~58,-58~-1 |           — |           — | -57~-1 |           — | 0~58,-58~-1 |
        |            +------------------+-------------+-------------+-------------+---------------+----------------------------+-------------+-------------+--------+-------------+-------------+-------------+-------------+--------+-------------+-------------+
        |            | HE-LTF (HE-LTF1) |           — |           — |           — | 0~122,-122~1  |       由CSI配置决定        |           — |           — |      — |           — |           — |           — |           — |      — |           — |           — |
        |            +------------------+-------------+-------------+-------------+---------------+                            +-------------+-------------+--------+-------------+-------------+-------------+-------------+--------+-------------+-------------+
        |            | HE-LTF2          |           — |           — |           — |             — |                            |           — |           — |      — |           — |           — |           — |           — |      — |           — |           — |
        +------------+------------------+-------------+-------------+-------------+---------------+----------------------------+-------------+-------------+--------+-------------+-------------+-------------+-------------+--------+-------------+-------------+
        | 总字节数                      |         106 |          114|         228 |           490 |                        490 |         106 |          114|   228  |         234 |         468 |       106   |          114|    228 |         234 |         468 |
        +-------------------------------+-------------+-------------+-------------+---------------+----------------------------+-------------+-------------+--------+-------------+-------------+-------------+-------------+--------+-------------+-------------+

        .. note ::

            - HT/VHT/HE 模式均存在两段 LTF 序列即：LLTF + HT/VHT/HE-LTF， 如果 :cpp:struct:`wifi_csi_acquire_config_t` 中 ``acquire_csi_force_lltf`` 为 false 时， CSI 数据中仅包含 HT/VHT/HE-LTF(如上表所示)， 否则 CSI 数据中仅包含 LLTF。 HT/VHT/HE 模式 LLTF 的子载波索引和 CSI 总字节数同非 HT 模式 LLTF 子载波索引和 CSI 总字节数一致。
            - VHT 模式时子载波索引和 CSI 总字节数同 HT 模式的子载波索引和 CSI 总字节数一致。

        表中的信息可以在 :cpp:type:`wifi_csi_info_t` 结构体中找到。

            - 辅助信道指 ``rx_ctrl`` 字段的 ``second`` 字段。
            - 数据包的信号模式指 ``rx_ctrl`` 字段的 ``cur_bb_format`` 字段。
            - 总字节数指 ``len`` 字段。
            - 每个长训练字段 (LTF) 类型对应的 CSI 数据存储在从 buf 字段开始的缓冲区中。每个元素以两个字节的形式存储：虚部和实部。每个元素的顺序与表中的子载波相同。LTF 的顺序是 LLTF、HT/VHT/HE-LTF。根据信道和数据包的信息，LTF 的存在情况见上文。
            - 如果 ``first_word_invalid`` 字段为 true，表示由于 {IDF_TARGET_NAME} 的硬件限制，CSI 数据的前四个字节无效。
            - 如果 ``rx_ctrl`` 字段中的 ``rx_channel_estimate_info_vld`` 为 1,表明CSI数据是有效的，否则，CSI数据是无效的。
            - 更多信息，如 RSSI、射频底噪声、接收时间及天线，请参见 ``rx_ctrl`` 字段。

        对于 STBC 数据包，HE-LTF 和 STBC-HE-LTF 的子载波索引由 :cpp:type:`wifi_csi_config_t` 中的 ``acquire_csi_he_stbc_mode`` 字段决定，具体请参见下表。

        +---------------------+------------------+-------------------+
        | acquire_csi_he_stbc | HE-LTF1          | HE-LTF2           |
        +---------------------+------------------+-------------------+
        |         0           |  -122~-1, 0~122  |                 — |
        +---------------------+------------------+-------------------+
        |         1           |                — |  -122~-1, 0~122   |
        +---------------------+------------------+-------------------+
        |         2           | 在 HE-LTF1 和 HE-LTF2 中进行均匀采样 |
        +---------------------+------------------+-------------------+

        有效子载波的虚部和实部的使用请参考下表。

        +-----------------------+------------------+------------------------------------------+------------------------------+
        | PHY 标准              | 子载波范围       |               无效子载波                 | 子载波个数（总数/数据子载波）|
        +=======================+==================+==========================================+==============================+
        | 802.11a/g             | -26 to +26       |                    0                     | 53 total, 52 usable          |
        +-----------------------+------------------+------------------------------------------+------------------------------+
        | 802.11n, 20 MHz       | -28 to +28       |                    0                     | 57 total, 56 usable          |
        +-----------------------+------------------+------------------------------------------+------------------------------+
        | 802.11n, 40 MHz       | -58 to +58       |                -1, 0, 1                  | 117 total, 114 usable        |
        +-----------------------+------------------+------------------------------------------+------------------------------+
        | 802.11ac, 20 MHz      | -28 to +28       |                    0                     | 57 total, 56 usable          |
        +-----------------------+------------------+------------------------------------------+------------------------------+
        | 802.11ax, 20 MHz (SU) | -122 to + 122    |                -1, 0, 1                  | 245 total, 242 usable        |
        +-----------------------+------------------+------------------------------------------+------------------------------+
### 这是S3 S2 32 C3
.. only:: esp32 or esp32s2 or esp32c3 or esp32s3

        信道状态信息 (CSI) 是指 Wi-Fi 连接的信道信息。{IDF_TARGET_NAME} 中，CSI 由子载波的信道频率响应组成，CSI 从发送端接收数据包时开始估计。每个子载波信道频率响由两个字节的有符号字符记录，第一个字节是虚部，第二个字节是实部。根据接收数据包的类型，信道频率响应最多有三个字段。分别是 LLTF、HT-LTF 和 STBC-HT-LTF。对于在不同状态的信道上接收到的不同类型的数据包，CSI 的子载波索引和总字节数如下表所示。

        +------------+-------------+-----------------------------------------+-----------------------------------------------------+--------------------------------------------------------+
        | 信道       | 辅助信道    |                                         | 下                                                  | 上                                                     |
        +------------+-------------+-------------+---------------------------+---------+-------------------------------------------+---------+----------------------------------------------+
        | 数据包信息 | 信号模式    | 非 HT       | HT                        | 非 HT   | HT                                        | 非 HT   | HT                                           |
        |            +-------------+-------------+---------------------------+---------+---------------+---------------------------+---------+------------------+---------------------------+
        |            | 信道带宽    | 20 MHz      | 20 MHz                    | 20 MHz  | 20 MHz        | 40 MHz                    | 20 MHz  | 20 MHz           | 40 MHz                    |
        |            +-------------+-------------+-------------+-------------+---------+--------+------+-------------+-------------+---------+---------+--------+-------------+-------------+
        |            | STBC        | 非 STBC     | 非 STBC     | STBC        | 非 STBC | 非 STBC| STBC | 非 STBC     | STBC        | 非 STBC | 非 STBC | STBC   | 非 STBC     | STBC        |
        +------------+-------------+-------------+-------------+-------------+---------+--------+------+-------------+-------------+---------+---------+--------+-------------+-------------+
        | 子载波索引 | LLTF        | 0~31,-32~-1 | 0~31,-32~-1 | 0~31,-32~-1 |    0~63 |   0~63 | 0~63 |        0~63 |        0~63 |  -64~-1 |  -64~-1 | -64~-1 |      -64~-1 |      -64~-1 |
        |            +-------------+-------------+-------------+-------------+---------+--------+------+-------------+-------------+---------+---------+--------+-------------+-------------+
        |            | HT-LTF      |           — | 0~31,-32~-1 | 0~31,-32~-1 |       — |   0~63 | 0~62 | 0~63,-64~-1 | 0~60,-60~-1 |       — |  -64~-1 | -62~-1 | 0~63,-64~-1 | 0~60,-60~-1 |
        |            +-------------+-------------+-------------+-------------+---------+--------+------+-------------+-------------+---------+---------+--------+-------------+-------------+
        |            | STBC-HT-LTF |           — |           — | 0~31,-32~-1 |       — |      — | 0~62 |           — | 0~60,-60~-1 |       — |       — | -62~-1 |           — | 0~60,-60~-1 |
        +------------+-------------+-------------+-------------+-------------+---------+--------+------+-------------+-------------+---------+---------+--------+-------------+-------------+
        | 总字节数                 |         128 |         256 |         384 |     128 |    256 |  380 |         384 |         612 |     128 |     256 |    376 |         384 |         612 |
        +--------------------------+-------------+-------------+-------------+---------+--------+------+-------------+-------------+---------+---------+--------+-------------+-------------+

        表中的所有信息可以在 wifi_csi_info_t 结构中找到。

            - 辅助信道指 rx_ctrl 字段的 secondary_channel 字段。
            - 数据包的信号模式指 rx_ctrl 字段的 sig_mode 字段。
            - 信道带宽指 rx_ctrl 字段中的 cwb 字段。
            - STBC 指 rx_ctrl 字段的 stbc 字段。
            - 总字节数指 len 字段。
            - 每个长训练字段 (LTF) 类型对应的 CSI 数据存储在从 buf 字段开始的缓冲区中。每个元素以两个字节的形式存储：虚部和实部。每个元素的顺序与表中的子载波相同。LTF 的顺序是 LLTF、HT-LTF 和 STBC-HT-LTF。但是，根据信道和数据包的信息，3 个 LTF 可能都不存在（见上文）。
            - 如果 :cpp:type:`wifi_csi_info_t` 的 first_word_invalid 字段为 true，表示由于 {IDF_TARGET_NAME} 的硬件限制，CSI 数据的前四个字节无效。
            - 更多信息，如 RSSI，射频的噪声底，接收时间和天线 rx_ctrl 领域。

        子载波的虚部和实部的使用请参考下表。

        +----------------+-------------------+------------------------------+------------------------------+
        | PHY 标准       | 子载波范围        | 导频子载波                   | 子载波个数（总数/数据子载波）|
        +================+===================+==============================+==============================+
        | 802.11a/g      | -26 to +26        | -21, -7, +7, +21             | 52 total, 48 usable          |
        +----------------+-------------------+------------------------------+------------------------------+
        | 802.11n, 20 MHz| -28 to +28        | -21, -7, +7, +21             | 56 total, 52 usable          |
        +----------------+-------------------+------------------------------+------------------------------+
        | 802.11n, 40 MHz| -57 to +57        | -53, -25, -11, +11, +25, +53 | 114 total, 108 usable        |
        +----------------+-------------------+------------------------------+------------------------------+

        .. note ::

            - 对于 STBC 数据包，每个空时流都提供了 CSI，不会出现 CSD（循环移位延迟）。由于附加链上的每一次循环移位为 -200 ns，因为子载波 0 中没有信道频率响应，在 HT-LTF 和 STBC-HT-LTF 中只记录第一空时流的 CSD 角度。CSD[10:0] 是 11 位，范围从 -pi 到 pi。

            - 如果调用 API :cpp:func:`esp_wifi_set_csi_config()` 没有使能 LLTF、HT-LTF 或 STBC-HT-LTF，则 CSI 数据的总字节数会比表中的少。例如，如果没有使能 LLTF 和 HT-LTF，而使能 STBC-HT-LTF，当接收到上述条件、HT、40 MHz 或 STBC 的数据包时，CSI 数据的总字节数为 244（(61+60)*2+2=244，结果对齐为四个字节，最后两个字节无效）。
### 这是C5
    .. only:: esp32c5

        信道状态信息 (CSI) 是指 Wi-Fi 连接的信道信息。{IDF_TARGET_NAME} 中，CSI 由子载波的信道频率响应组成，CSI 从发送端接收数据包时开始估计。每个子载波信道频率响由两个字节的有符号字符记录，第一个字节是虚部，第二个字节是实部。除了 IEEE 802.11g 模式外，其他模式均存在两段 LTF 序列（LLTF + HT/VHT/HE-LTF）。{IDF_TARGET_NAME}可以通过 :cpp:struct:`wifi_csi_acquire_config_t` 中 ``acquire_csi_force_lltf`` 字段决定包含 LLTF 或 HT/VHT/HE-LTF。对于在不同状态的信道上接收到的不同类型的数据包，CSI 的子载波索引和总字节数如下表所示。

        +------------+------------------+-----------------------------------------+--------------------------------------------+----------------------------------------------------------------+----------------------------------------------------------------+
        | 信道       | 辅助信道         |                                                                                      | 下                                                             | 上                                                             |
        +------------+------------------+-------------+---------------------------+--------------------------------------------+-------------+--------------------------------------------------+-------------+--------------------------------------------------+
        | 数据包信息 | 信号模式         | 非 HT       | HT                        | HE                                         | 非 HT       | HT                                               | 非 HT       | HT                                               |
        |            +------------------+-------------+---------------------------+--------------------------------------------+-------------+----------------------+---------------------------+-------------+----------------------+---------------------------+
        |            | 信道带宽         | 20 MHz      | 20 MHz                    | 20 MHz                                     | 20 MHz      | 20 MHz               | 40 MHz                    | 20 MHz      | 20 MHz               | 40 MHz                    |
        |            +------------------+-------------+-------------+-------------+---------------+----------------------------+-------------+-------------+--------+-------------+-------------+-------------+-------------+--------+-------------+-------------+
        |            | STBC             | 非 STBC     | 非 STBC     | STBC        | 非 STBC       | STBC                       | 非 STBC     | 非 STBC     | STBC   | 非 STBC     | STBC        | 非 STBC     | 非 STBC     | STBC   | 非 STBC     | STBC        |
        +------------+------------------+-------------+-------------+-------------+---------------+----------------------------+-------------+-------------+--------+-------------+-------------+-------------+-------------+--------+-------------+-------------+
        | 子载波索引 | LLTF             | 0~26,-26~-1 |           — |           — |             — |                          — |   0~52      |           — |      — |           — |           — |       -53~-1|           — |      — |           — |           — |
        |            +------------------+-------------+-------------+-------------+---------------+----------------------------+-------------+-------------+--------+-------------+-------------+-------------+-------------+--------+-------------+-------------+
        |            | HT-LTF (HT-LTF1) |           — | 0~28,-28~-1 | 0~28,-28~-1 |             — |                          — |           — |         0~56|  0~56  | 0~58,-58~-1 | 0~58,-58~-1 |           — |       -57~-1| -57~-1 | 0~58,-58~-1 | 0~58,-58~-1 |
        |            +------------------+-------------+-------------+-------------+---------------+----------------------------+-------------+-------------+--------+-------------+-------------+-------------+-------------+--------+-------------+-------------+
        |            | HT-LTF2          |           — |           — | 0~28,-28~-1 |             — |                          — |           — |           — |  0~56  |           — | 0~58,-58~-1 |           — |           — | -57~-1 |           — | 0~58,-58~-1 |
        |            +------------------+-------------+-------------+-------------+---------------+----------------------------+-------------+-------------+--------+-------------+-------------+-------------+-------------+--------+-------------+-------------+
        |            | HE-LTF (HE-LTF1) |           — |           — |           — | 0~122,-122~1  |       由CSI配置决定        |           — |           — |      — |           — |           — |           — |           — |      — |           — |           — |
        |            +------------------+-------------+-------------+-------------+---------------+                            +-------------+-------------+--------+-------------+-------------+-------------+-------------+--------+-------------+-------------+
        |            | HE-LTF2          |           — |           — |           — |             — |                            |           — |           — |      — |           — |           — |           — |           — |      — |           — |           — |
        +------------+------------------+-------------+-------------+-------------+---------------+----------------------------+-------------+-------------+--------+-------------+-------------+-------------+-------------+--------+-------------+-------------+
        | 总字节数                      |         106 |          114|         228 |           490 |                        490 |         106 |          114|   228  |         234 |         468 |       106   |          114|    228 |         234 |         468 |
        +-------------------------------+-------------+-------------+-------------+---------------+----------------------------+-------------+-------------+--------+-------------+-------------+-------------+-------------+--------+-------------+-------------+

        .. note ::

            - HT/VHT/HE 模式均存在两段 LTF 序列即：LLTF + HT/VHT/HE-LTF， 如果 :cpp:struct:`wifi_csi_acquire_config_t` 中 ``acquire_csi_force_lltf`` 为 false 时， CSI 数据中仅包含 HT/VHT/HE-LTF(如上表所示)， 否则 CSI 数据中仅包含 LLTF。 HT/VHT/HE 模式 LLTF 的子载波索引和 CSI 总字节数同非 HT 模式 LLTF 子载波索引和 CSI 总字节数一致。
            - VHT 模式时子载波索引和 CSI 总字节数同 HT 模式的子载波索引和 CSI 总字节数一致。

        表中的信息可以在 :cpp:type:`wifi_csi_info_t` 结构体中找到。

            - 辅助信道指 ``rx_ctrl`` 字段的 ``second`` 字段。
            - 数据包的信号模式指 ``rx_ctrl`` 字段的 ``cur_bb_format`` 字段。
            - 总字节数指 ``len`` 字段。
            - 每个长训练字段 (LTF) 类型对应的 CSI 数据存储在从 buf 字段开始的缓冲区中。每个元素以两个字节的形式存储：虚部和实部。每个元素的顺序与表中的子载波相同。LTF 的顺序是 LLTF、HT/VHT/HE-LTF。根据信道和数据包的信息，LTF 的存在情况见上文。
            - 如果 ``first_word_invalid`` 字段为 true，表示由于 {IDF_TARGET_NAME} 的硬件限制，CSI 数据的前四个字节无效。
            - 如果 ``rx_ctrl`` 字段中的 ``rx_channel_estimate_info_vld`` 为 1,表明CSI数据是有效的，否则，CSI数据是无效的。
            - 更多信息，如 RSSI、射频底噪声、接收时间及天线，请参见 ``rx_ctrl`` 字段。

        对于 STBC 数据包，HE-LTF 和 STBC-HE-LTF 的子载波索引由 :cpp:type:`wifi_csi_config_t` 中的 ``acquire_csi_he_stbc_mode`` 字段决定，具体请参见下表。

        +---------------------+------------------+-------------------+
        | acquire_csi_he_stbc | HE-LTF1          | HE-LTF2           |
        +---------------------+------------------+-------------------+
        |         0           |  -122~-1, 0~122  |                 — |
        +---------------------+------------------+-------------------+
        |         1           |                — |  -122~-1, 0~122   |
        +---------------------+------------------+-------------------+
        |         2           | 在 HE-LTF1 和 HE-LTF2 中进行均匀采样 |
        +---------------------+------------------+-------------------+

        有效子载波的虚部和实部的使用请参考下表。

        +-----------------------+------------------+------------------------------------------+------------------------------+
        | PHY 标准              | 子载波范围       |               无效子载波                 | 子载波个数（总数/数据子载波）|
        +=======================+==================+==========================================+==============================+
        | 802.11a/g             | -26 to +26       |                    0                     | 53 total, 52 usable          |
        +-----------------------+------------------+------------------------------------------+------------------------------+
        | 802.11n, 20 MHz       | -28 to +28       |                    0                     | 57 total, 56 usable          |
        +-----------------------+------------------+------------------------------------------+------------------------------+
        | 802.11n, 40 MHz       | -58 to +58       |                -1, 0, 1                  | 117 total, 114 usable        |
        +-----------------------+------------------+------------------------------------------+------------------------------+
        | 802.11ac, 20 MHz      | -28 to +28       |                    0                     | 57 total, 56 usable          |
        +-----------------------+------------------+------------------------------------------+------------------------------+
        | 802.11ax, 20 MHz (SU) | -122 to + 122    |                -1, 0, 1                  | 245 total, 242 usable        |
        +-----------------------+------------------+------------------------------------------+------------------------------+
### 这是C6
   .. only:: esp32c6

        信道状态信息 (CSI) 是指 Wi-Fi 连接的信道信息。{IDF_TARGET_NAME} 中，CSI 由子载波的信道频率响应组成，CSI 从发送端接收数据包时开始估计。每个子载波信道频率响由两个字节的有符号字符记录，第一个字节是虚部，第二个字节是实部。根据接收数据包的类型，信道频率响应有五个字段。分别是 LLTF、HT-LTF、STBC-HT-LTF、HE-LTF、STBC-HE-LTF。对于在不同状态的信道上接收到的不同类型的数据包，CSI 的子载波索引和总字节数如下表所示。

        +------------+------------------+-----------------------------------------+--------------------------------------------+----------------------------------------------------------------+----------------------------------------------------------------+
        | 信道       | 辅助信道         |                                                                                      | 下                                                             | 上                                                             |
        +------------+------------------+-------------+---------------------------+--------------------------------------------+-------------+--------------------------------------------------+-------------+--------------------------------------------------+
        | 数据包信息 | 信号模式         | 非 HT       | HT                        | HE                                         | 非 HT       | HT                                               | 非 HT       | HT                                               |
        |            +------------------+-------------+---------------------------+--------------------------------------------+-------------+----------------------+---------------------------+-------------+----------------------+---------------------------+
        |            | 信道带宽         | 20 MHz      | 20 MHz                    | 20 MHz                                     | 20 MHz      | 20 MHz               | 40 MHz                    | 20 MHz      | 20 MHz               | 40 MHz                    |
        |            +------------------+-------------+-------------+-------------+---------------+----------------------------+-------------+-------------+--------+-------------+-------------+-------------+-------------+--------+-------------+-------------+
        |            | STBC             | 非 STBC     | 非 STBC     | STBC        | 非 STBC       | STBC                       | 非 STBC     | 非 STBC     | STBC   | 非 STBC     | STBC        | 非 STBC     | 非 STBC     | STBC   | 非 STBC     | STBC        |
        +------------+------------------+-------------+-------------+-------------+---------------+----------------------------+-------------+-------------+--------+-------------+-------------+-------------+-------------+--------+-------------+-------------+
        | 子载波索引 | LLTF             | 0~31,-32~-1 |           — |           — |             — |                          — |   0~63      |           — |      — |           — |           — |       -64~-1|           — |      — |           — |           — |
        |            +------------------+-------------+-------------+-------------+---------------+----------------------------+-------------+-------------+--------+-------------+-------------+-------------+-------------+--------+-------------+-------------+
        |            | HT-LTF (HT-LTF1) |           — | 0~31,-32~-1 | 0~31,-32~-1 |             — |                          — |           — |         0~63|  0~63  | 0~63,-64~-1 | 0~63,-64~-1 |           — |       -64~-1| -64~-1 | 0~63,-64~-1 | 0~63,-64~-1 |
        |            +------------------+-------------+-------------+-------------+---------------+----------------------------+-------------+-------------+--------+-------------+-------------+-------------+-------------+--------+-------------+-------------+
        |            | HT-LTF2          |           — |           — | 0~31,-32~-1 |             — |                          — |           — |           — |  0~63  |           — | 0~63,-64~-1 |           — |           — | -64~-1 |           — | 0~63,-64~-1 |
        |            +------------------+-------------+-------------+-------------+---------------+----------------------------+-------------+-------------+--------+-------------+-------------+-------------+-------------+--------+-------------+-------------+
        |            | HE-LTF (HE-LTF1) |           — |           — |           — | 0~127,-128~1  |       由CSI配置决定        |           — |           — |      — |           — |           — |           — |           — |      — |           — |           — |
        |            +------------------+-------------+-------------+-------------+---------------+                            +-------------+-------------+--------+-------------+-------------+-------------+-------------+--------+-------------+-------------+
        |            | HE-LTF2          |           — |           — |           — |             — |                            |           — |           — |      — |           — |           — |           — |           — |      — |           — |           — |
        +------------+------------------+-------------+-------------+-------------+---------------+----------------------------+-------------+-------------+--------+-------------+-------------+-------------+-------------+--------+-------------+-------------+
        | 总字节数                      |         128 |          128|         256 |           512 |                        512 |         128 |          128|   256  |         256 |         512 |       128   |          128|    256 |         256 |         512 |
        +-------------------------------+-------------+-------------+-------------+---------------+----------------------------+-------------+-------------+--------+-------------+-------------+-------------+-------------+--------+-------------+-------------+

        表中的信息可以在 :cpp:type:`wifi_csi_info_t` 结构体中找到。

            - 辅助信道指 ``rx_ctrl`` 字段的 ``second`` 字段。
            - 数据包的信号模式指 ``rx_ctrl`` 字段的 ``cur_bb_format`` 字段。
            - 总字节数指 ``len`` 字段
            - 每个长训练字段 (LTF) 类型对应的 CSI 数据存储在从 buf 字段开始的缓冲区中。每个元素以两个字节的形式存储：虚部和实部。每个元素的顺序与表中的子载波相同。LTF 的顺序是 LLTF、HT-LTF、STBC-HT-LTF、HE-LTF、STBC-HE-LTF。根据信道和数据包的信息，LTF 的存在情况见上文。
            - 如果 ``first_word_invalid`` 字段为 true，表示由于 {IDF_TARGET_NAME} 的硬件限制，CSI 数据的前四个字节无效。
            - 如果 ``rx_ctrl`` 字段中的 ``rx_channel_estimate_info_vld`` 为 1,表明CSI数据是有效的，否则，CSI数据是无效的。
            - 更多信息，如 RSSI、射频底噪声、接收时间及天线，请参见 ``rx_ctrl`` 字段。

        对于 STBC 数据包，HE-LTF1 和 HE-LTF2 的子载波索引由 :cpp:type:`wifi_csi_config_t` 中的 ``acquire_csi_he_stbc`` 字段决定，具体请参见下表。。

        +---------------------+------------------+-------------------+
        | acquire_csi_he_stbc | HE-LTF1          | HE-LTF2           |
        +---------------------+------------------+-------------------+
        |         0           |  -128~-1, 0~127  |                 — |
        +---------------------+------------------+-------------------+
        |         1           |                — |  -128~-1, 0~127   |
        +---------------------+------------------+-------------------+
        |         2           | 在 HE-LTF1 和 HE-LTF2 中进行均匀采样 |
        +---------------------+------------------+-------------------+

        子载波的虚部和实部的使用请参考下表。

        +-----------------------+--------------------+------------------------------------------+------------------------------+
        | PHY 标准              |  子载波范围        | 无效子载波                               | 子载波个数（总数/数据子载波）|
        +=======================+====================+==========================================+==============================+
        | 802.11a/g             |  -26 to +26        |                  0                       | 53 total, 52 usable          |
        +-----------------------+--------------------+------------------------------------------+------------------------------+
        | 802.11n, 20 MHz       |  -28 to +28        |                  0                       | 57 total, 56 usable          |
        +-----------------------+--------------------+------------------------------------------+------------------------------+
        | 802.11n, 40 MHz       |  -58 to +58        |              -1, 0, 1                    | 117 total, 114 usable        |
        +-----------------------+--------------------+------------------------------------------+------------------------------+
        | 802.11ax, 20 MHz (SU) |  -122 to + 122     |              -1, 0, 1                    | 245 total, 242 usable        |
        +-----------------------+--------------------+------------------------------------------+------------------------------+

        .. note ::

            - PHY 为 802.11ax时，MU 数据包的 CSI 子载波范围和无效子载波索引请参考协议。

## 4.不同芯片之间调度的差异总结
### 4.1 S2、S3、C3、32的csi数据获取方式是累加式的
即 获取非HT非STBC 就是lltf数据
获取其余模式是，是lltf数据加别的数据
### 4.2 C5 C6 的csi数据获取方式是替换式的
获取非HT非STBC 就是lltf数据
获取其余模式是，是仅有别的数据，没有lltf数据
并且c5的lltf数据是12位的，不是8位的
