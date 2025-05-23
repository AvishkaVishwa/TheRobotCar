/*
 * SPDX-FileCopyrightText: 2015-2024 Espressif Systems (Shanghai) CO LTD
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#ifndef __ESP_BT_H__
#define __ESP_BT_H__

#include <stdint.h>
#include <stdbool.h>
#include "esp_err.h"
#include "sdkconfig.h"
#include "esp_task.h"
#include "esp_assert.h"

#ifdef __cplusplus
extern "C" {
#endif

#define ESP_BT_CTRL_CONFIG_MAGIC_VAL    0x5A5AA5A5
#define ESP_BT_CTRL_CONFIG_VERSION      0x02410230

#define ESP_BT_HCI_TL_MAGIC_VALUE   0xfadebead
#define ESP_BT_HCI_TL_VERSION       0x00010000

/**
 * @brief Bluetooth mode for controller enable/disable
 */
typedef enum {
    ESP_BT_MODE_IDLE       = 0x00,   /*!< Bluetooth is not running */
    ESP_BT_MODE_BLE        = 0x01,   /*!< Run BLE mode */
    ESP_BT_MODE_CLASSIC_BT = 0x02,   /*!< Run Classic BT mode */
    ESP_BT_MODE_BTDM       = 0x03,   /*!< Run dual mode */
} esp_bt_mode_t;

/**
 * @brief Type of controller HCI transport layer
 */
typedef enum {
    ESP_BT_CTRL_HCI_TL_UART = 0,      /*!< HCI UART h4 transport layer */
    ESP_BT_CTRL_HCI_TL_VHCI = 1,      /*!< VHCI interface */
} esp_bt_ctrl_hci_tl_t;

/**
 * @brief type of BLE connection event length computation
 */
typedef enum {
    ESP_BLE_CE_LEN_TYPE_ORIG = 0,      /*!< original */
    ESP_BLE_CE_LEN_TYPE_CE   = 1,      /*!< use CE_LEN parameter from HCI commands */
    ESP_BLE_CE_LEN_TYPE_SD   = 1,      /*!< Espressif vendor defined */
} esp_ble_ce_len_t;

/**
 * @brief Bluetooth sleep mode
 */
typedef enum {
    ESP_BT_SLEEP_MODE_NONE = 0,      /*!< Bluetooth sleep mode disabled */
    ESP_BT_SLEEP_MODE_1    = 1,      /*!< Bluetooth sleep mode 1 */
} esp_bt_sleep_mode_t;

/**
 * @brief Bluetooth sleep clock
 */
typedef enum {
    ESP_BT_SLEEP_CLOCK_NONE            = 0,   /*!< Sleep clock not configured */
    ESP_BT_SLEEP_CLOCK_MAIN_XTAL       = 1,   /*!< SoC main crystal */
    ESP_BT_SLEEP_CLOCK_EXT_32K_XTAL    = 2,   /*!< External 32.768kHz crystal */
    ESP_BT_SLEEP_CLOCK_RTC_SLOW        = 3,   /*!< Internal 136kHz RC oscillator */
    ESP_BT_SLEEP_CLOCK_FPGA_32K        = 4,   /*!< Hardwired 32KHz clock temporarily used for FPGA */
} esp_bt_sleep_clock_t;

/**
 * @brief antenna index used for bluetooth
 */
enum {
    ESP_BT_ANT_IDX_0 = 0,               /*!< anntena NO 0 */
    ESP_BT_ANT_IDX_1 = 1,               /*!< anntena NO 1 */
};

/**
 * @brief Maximum Tx/Rx time limit on Coded-PHY connection
 */
enum {
    ESP_BT_COEX_PHY_CODED_TX_RX_TIME_LIMIT_FORCE_DISABLE = 0,    /*!< Disable the limit */
    ESP_BT_COEX_PHY_CODED_TX_RX_TIME_LIMIT_FORCE_ENABLE,         /*!< Always Enable the limit */
};

#define ESP_BT_HCI_TL_STATUS_OK            (0)   /*!< HCI_TL Tx/Rx operation status OK */

/**
 * @brief callback function for HCI Transport Layer send/receive operations
 */
typedef void (* esp_bt_hci_tl_callback_t) (void *arg, uint8_t status);

#ifdef CONFIG_BT_ENABLED

#define BT_CTRL_BLE_MAX_ACT_LIMIT           10  //Maximum BLE activity limitation
#define SLAVE_CE_LEN_MIN_DEFAULT             5

#ifdef CONFIG_BT_CTRL_SCAN_DUPL_TYPE
#define SCAN_DUPLICATE_TYPE_VALUE  CONFIG_BT_CTRL_SCAN_DUPL_TYPE
#else
#define SCAN_DUPLICATE_TYPE_VALUE  0
#endif

/* normal adv cache size */
#ifdef CONFIG_BT_CTRL_SCAN_DUPL_CACHE_SIZE
#define NORMAL_SCAN_DUPLICATE_CACHE_SIZE            CONFIG_BT_CTRL_SCAN_DUPL_CACHE_SIZE
#else
#define NORMAL_SCAN_DUPLICATE_CACHE_SIZE            20
#endif

#ifndef CONFIG_BT_CTRL_BLE_MESH_SCAN_DUPL_EN
#define CONFIG_BT_CTRL_BLE_MESH_SCAN_DUPL_EN FALSE
#endif

#define SCAN_DUPLICATE_MODE_NORMAL_ADV_ONLY         0
#define SCAN_DUPLICATE_MODE_NORMAL_ADV_MESH_ADV     1

#if CONFIG_BT_CTRL_BLE_MESH_SCAN_DUPL_EN
    #define SCAN_DUPLICATE_MODE                     SCAN_DUPLICATE_MODE_NORMAL_ADV_MESH_ADV
    #ifdef CONFIG_BT_CTRL_MESH_DUPL_SCAN_CACHE_SIZE
    #define MESH_DUPLICATE_SCAN_CACHE_SIZE          CONFIG_BT_CTRL_MESH_DUPL_SCAN_CACHE_SIZE
    #else
    #define MESH_DUPLICATE_SCAN_CACHE_SIZE          50
    #endif
#else
    #define SCAN_DUPLICATE_MODE SCAN_DUPLICATE_MODE_NORMAL_ADV_ONLY
    #define MESH_DUPLICATE_SCAN_CACHE_SIZE          0
#endif

#ifndef CONFIG_BT_CTRL_DUPL_SCAN_CACHE_REFRESH_PERIOD
#define DUPL_SCAN_CACHE_REFRESH_PERIOD 0
#else
#define DUPL_SCAN_CACHE_REFRESH_PERIOD CONFIG_BT_CTRL_DUPL_SCAN_CACHE_REFRESH_PERIOD
#endif

#ifdef CONFIG_BT_CTRL_SCAN_BACKOFF_UPPERLIMITMAX
#define BT_CTRL_SCAN_BACKOFF_UPPERLIMITMAX CONFIG_BT_CTRL_SCAN_BACKOFF_UPPERLIMITMAX
#else
#define BT_CTRL_SCAN_BACKOFF_UPPERLIMITMAX  0
#endif

#ifdef CONFIG_BT_CTRL_AGC_RECORRECT_EN
#define BT_CTRL_AGC_RECORRECT_EN  CONFIG_BT_CTRL_AGC_RECORRECT_EN
// ESP32-S3
#if CONFIG_IDF_TARGET_ESP32S3
#define BT_CTRL_AGC_RECORRECT_NEW       1
#else
//Check if chip target is ESP32-C3 101
#if CONFIG_ESP32C3_REV_MIN_101
#define BT_CTRL_AGC_RECORRECT_NEW       1
#else
#define BT_CTRL_AGC_RECORRECT_NEW       0
#endif // CONFIG_ESP32C3_REV_MIN_101
#endif // CONFIG_IDF_TARGET_ESP32S3

#else
#define BT_CTRL_AGC_RECORRECT_EN        0
#define BT_CTRL_AGC_RECORRECT_NEW       0
#endif

#ifdef CONFIG_BT_CTRL_CODED_AGC_RECORRECT_EN
#define BT_CTRL_CODED_AGC_RECORRECT  CONFIG_BT_CTRL_CODED_AGC_RECORRECT_EN
#else
#define BT_CTRL_CODED_AGC_RECORRECT        0
#endif

#if defined (CONFIG_BT_BLE_50_FEATURES_SUPPORTED) || defined (CONFIG_BT_NIMBLE_50_FEATURE_SUPPORT)
#ifdef CONFIG_BT_BLE_50_FEATURES_SUPPORTED
#define BT_CTRL_50_FEATURE_SUPPORT   (CONFIG_BT_BLE_50_FEATURES_SUPPORTED)
#endif // CONFIG_BT_BLE_50_FEATURES_SUPPORTED
#ifdef CONFIG_BT_NIMBLE_50_FEATURE_SUPPORT
#define BT_CTRL_50_FEATURE_SUPPORT   (CONFIG_BT_NIMBLE_50_FEATURE_SUPPORT)
#endif // CONFIG_BT_NIMBLE_50_FEATURE_SUPPORT
#else
#if defined (CONFIG_BT_BLUEDROID_ENABLED) || defined (CONFIG_BT_NIMBLE_ENABLED)
#define BT_CTRL_50_FEATURE_SUPPORT (0)
#else
#define BT_CTRL_50_FEATURE_SUPPORT (1)
#endif // (CONFIG_BT_BLUEDROID_ENABLED) || (CONFIG_BT_NIMBLE_ENABLED)
#endif // (CONFIG_BT_BLE_50_FEATURES_SUPPORTED) || (CONFIG_BT_NIMBLE_50_FEATURE_SUPPORT)

#if defined(CONFIG_BT_BLE_CCA_MODE)
#define BT_BLE_CCA_MODE (CONFIG_BT_BLE_CCA_MODE)
#else
#define BT_BLE_CCA_MODE (0)
#endif

#if defined(CONFIG_BT_BLE_ADV_DATA_LENGTH_ZERO_AUX)
#define BT_BLE_ADV_DATA_LENGTH_ZERO_AUX (CONFIG_BT_BLE_ADV_DATA_LENGTH_ZERO_AUX)
#else
#define BT_BLE_ADV_DATA_LENGTH_ZERO_AUX (0)
#endif

#if defined(CONFIG_BT_CTRL_CHAN_ASS_EN)
#define BT_CTRL_CHAN_ASS_EN (CONFIG_BT_CTRL_CHAN_ASS_EN)
#else
#define BT_CTRL_CHAN_ASS_EN (0)
#endif

#if defined(CONFIG_BT_CTRL_LE_PING_EN)
#define BT_CTRL_LE_PING_EN (CONFIG_BT_CTRL_LE_PING_EN)
#else
#define BT_CTRL_LE_PING_EN (0)
#endif

#define AGC_RECORRECT_EN       ((BT_CTRL_AGC_RECORRECT_EN << 0) | (BT_CTRL_CODED_AGC_RECORRECT <<1) | (BT_CTRL_AGC_RECORRECT_NEW << 2))

#define CFG_MASK_BIT_SCAN_DUPLICATE_OPTION    (1<<0)

#define CFG_MASK      CFG_MASK_BIT_SCAN_DUPLICATE_OPTION
#if CONFIG_IDF_TARGET_ESP32C3
#define BLE_HW_TARGET_CODE_CHIP_ECO0                      (0x01010000)
#else // CONFIG_IDF_TARGET_ESP32S3
#define BLE_HW_TARGET_CODE_CHIP_ECO0                      (0x02010000)
#endif

#ifdef CONFIG_BT_CTRL_BLE_LLCP_CONN_UPDATE
#define BT_CTRL_BLE_LLCP_CONN_UPDATE (1<<0)
#else
#define BT_CTRL_BLE_LLCP_CONN_UPDATE (0<<0)
#endif

#ifdef CONFIG_BT_CTRL_BLE_LLCP_CHAN_MAP_UPDATE
#define BT_CTRL_BLE_LLCP_CHAN_MAP_UPDATE (1<<1)
#else
#define BT_CTRL_BLE_LLCP_CHAN_MAP_UPDATE (0<<1)
#endif

#ifdef CONFIG_BT_CTRL_BLE_LLCP_PHY_UPDATE
#define BT_CTRL_BLE_LLCP_PHY_UPDATE (1<<2)
#else
#define BT_CTRL_BLE_LLCP_PHY_UPDATE (0<<2)
#endif

#define BT_CTRL_BLE_LLCP_DISC_FLAG (BT_CTRL_BLE_LLCP_CONN_UPDATE | BT_CTRL_BLE_LLCP_CHAN_MAP_UPDATE | BT_CTRL_BLE_LLCP_PHY_UPDATE)
#if defined(CONFIG_BT_CTRL_RUN_IN_FLASH_ONLY)
#define BT_CTRL_RUN_IN_FLASH_ONLY  CONFIG_BT_CTRL_RUN_IN_FLASH_ONLY
#else
#define BT_CTRL_RUN_IN_FLASH_ONLY  (0)
#endif

#if (BT_CTRL_RUN_IN_FLASH_ONLY == 1)

#if defined(CONFIG_BT_CTRL_DTM_ENABLE)
#define BT_CTRL_DTM_ENABLE  CONFIG_BT_CTRL_DTM_ENABLE
#else
#define BT_CTRL_DTM_ENABLE  (0)
#endif

#if defined(CONFIG_BT_CTRL_BLE_MASTER)
#define BT_CTRL_BLE_MASTER  CONFIG_BT_CTRL_BLE_MASTER
#else
#define BT_CTRL_BLE_MASTER  (0)
#endif

#if defined(CONFIG_BT_CTRL_BLE_TEST)
#define BT_CTRL_BLE_TEST  CONFIG_BT_CTRL_BLE_TEST
#else
#define BT_CTRL_BLE_TEST  (0)
#endif

#if defined (CONFIG_BT_NIMBLE_SECURITY_ENABLE) || defined (CONFIG_BT_BLE_SMP_ENABLE)
#ifdef CONFIG_BT_NIMBLE_SECURITY_ENABLE
#define BLE_SECURITY_ENABLE  (CONFIG_BT_NIMBLE_SECURITY_ENABLE)
#endif //CONFIG_BT_NIMBLE_SECURITY_ENABLE
#ifdef CONFIG_BT_BLE_SMP_ENABLE
#define BLE_SECURITY_ENABLE  (CONFIG_BT_BLE_SMP_ENABLE)
#endif //CONFIG_BT_BLE_SMP_ENABLE
#else
#if defined (CONFIG_BT_CTRL_BLE_SECURITY_ENABLE)
#define BLE_SECURITY_ENABLE  (CONFIG_BT_CTRL_BLE_SECURITY_ENABLE)
#else
#define BLE_SECURITY_ENABLE  (0)
#endif
#endif // (CONFIG_BT_NIMBLE_SECURITY_ENABLE) || (CONFIG_BT_BLE_SMP_ENABLE)

#if defined (CONFIG_BT_CTRL_BLE_SCAN)
#define BT_CTRL_BLE_SCAN    CONFIG_BT_CTRL_BLE_SCAN
#else
#define BT_CTRL_BLE_SCAN    (0)
#endif

#else
#define BT_CTRL_BLE_MASTER   (1)
#define BT_CTRL_DTM_ENABLE   (1)
#define BT_CTRL_BLE_TEST     (1)
#define BLE_SECURITY_ENABLE  (1)
#define BT_CTRL_BLE_SCAN     (1)
#endif // (BT_CTRL_RUN_IN_FLASH_ONLY == 1)

#define BT_CONTROLLER_INIT_CONFIG_DEFAULT() {                              \
    .magic = ESP_BT_CTRL_CONFIG_MAGIC_VAL,                                 \
    .version = ESP_BT_CTRL_CONFIG_VERSION,                                 \
    .controller_task_stack_size = ESP_TASK_BT_CONTROLLER_STACK,            \
    .controller_task_prio = ESP_TASK_BT_CONTROLLER_PRIO,                   \
    .controller_task_run_cpu = CONFIG_BT_CTRL_PINNED_TO_CORE,              \
    .bluetooth_mode = CONFIG_BT_CTRL_MODE_EFF,                             \
    .ble_max_act = CONFIG_BT_CTRL_BLE_MAX_ACT_EFF,                         \
    .sleep_mode = CONFIG_BT_CTRL_SLEEP_MODE_EFF,                           \
    .sleep_clock = CONFIG_BT_CTRL_SLEEP_CLOCK_EFF,                         \
    .ble_st_acl_tx_buf_nb = CONFIG_BT_CTRL_BLE_STATIC_ACL_TX_BUF_NB,       \
    .ble_hw_cca_check = CONFIG_BT_CTRL_HW_CCA_EFF,                         \
    .ble_adv_dup_filt_max = CONFIG_BT_CTRL_ADV_DUP_FILT_MAX,               \
    .coex_param_en = false,                                                \
    .ce_len_type = CONFIG_BT_CTRL_CE_LENGTH_TYPE_EFF,                      \
    .coex_use_hooks = false,                                               \
    .hci_tl_type = CONFIG_BT_CTRL_HCI_TL_EFF,                              \
    .hci_tl_funcs = NULL,                                                  \
    .txant_dft = CONFIG_BT_CTRL_TX_ANTENNA_INDEX_EFF,                      \
    .rxant_dft = CONFIG_BT_CTRL_RX_ANTENNA_INDEX_EFF,                      \
    .txpwr_dft = CONFIG_BT_CTRL_DFT_TX_POWER_LEVEL_EFF,                    \
    .cfg_mask = CFG_MASK,                                                  \
    .scan_duplicate_mode = SCAN_DUPLICATE_MODE,                            \
    .scan_duplicate_type = SCAN_DUPLICATE_TYPE_VALUE,                      \
    .normal_adv_size = NORMAL_SCAN_DUPLICATE_CACHE_SIZE,                   \
    .mesh_adv_size = MESH_DUPLICATE_SCAN_CACHE_SIZE,                       \
    .coex_phy_coded_tx_rx_time_limit = CONFIG_BT_CTRL_COEX_PHY_CODED_TX_RX_TLIM_EFF, \
    .hw_target_code = BLE_HW_TARGET_CODE_CHIP_ECO0,                        \
    .slave_ce_len_min = SLAVE_CE_LEN_MIN_DEFAULT,                          \
    .hw_recorrect_en = AGC_RECORRECT_EN,                                   \
    .cca_thresh = CONFIG_BT_CTRL_HW_CCA_VAL,                               \
    .scan_backoff_upperlimitmax = BT_CTRL_SCAN_BACKOFF_UPPERLIMITMAX,      \
    .dup_list_refresh_period = DUPL_SCAN_CACHE_REFRESH_PERIOD,             \
    .ble_50_feat_supp  = BT_CTRL_50_FEATURE_SUPPORT,                       \
    .ble_cca_mode = BT_BLE_CCA_MODE,                                       \
    .ble_data_lenth_zero_aux = BT_BLE_ADV_DATA_LENGTH_ZERO_AUX,            \
    .ble_chan_ass_en = BT_CTRL_CHAN_ASS_EN,                                \
    .ble_ping_en = BT_CTRL_LE_PING_EN,                                     \
    .ble_llcp_disc_flag = BT_CTRL_BLE_LLCP_DISC_FLAG,                      \
    .run_in_flash = BT_CTRL_RUN_IN_FLASH_ONLY,                             \
    .dtm_en = BT_CTRL_DTM_ENABLE,                                          \
    .enc_en = BLE_SECURITY_ENABLE,                                         \
    .qa_test = BT_CTRL_BLE_TEST,                                           \
    .master_en = BT_CTRL_BLE_MASTER,                                       \
    .scan_en = BT_CTRL_BLE_SCAN,                                           \
}

#else
#define BT_CONTROLLER_INIT_CONFIG_DEFAULT() {0}; ESP_STATIC_ASSERT(0, "please enable bluetooth in menuconfig to use esp_bt.h");
#endif

/**
 * @brief Controller HCI transport layer function structure
 *        This structure shall be registered when HCI transport layer is UART
 */
typedef struct {
    uint32_t _magic;                        /*!< Magic number */
    uint32_t _version;                      /*!< Version number of the defined structure */
    uint32_t _reserved;                     /*!< Reserved for future use */
    int (* _open)(void);                    /*!< HCI transport layer open function */
    void (* _close)(void);                  /*!< HCI transport layer close function */
    void (* _finish_transfers)(void);       /*!< HCI transport layer finish transfers function */
    void (* _recv)(uint8_t *buf, uint32_t len, esp_bt_hci_tl_callback_t callback, void* arg); /*!< HCI transport layer receive function */
    void (* _send)(uint8_t *buf, uint32_t len, esp_bt_hci_tl_callback_t callback, void* arg); /*!< HCI transport layer send function */
    bool (* _flow_off)(void);               /*!< HCI transport layer flow off function */
    void (* _flow_on)(void);                /*!< HCI transport layer flow on function */
} esp_bt_hci_tl_t;

/**
 * @brief Controller config options, depend on config mask.
 *        Config mask indicate which functions enabled, this means
 *        some options or parameters of some functions enabled by config mask.
 */
typedef struct {
    /*
     * Following parameters can not be configured runtime when call esp_bt_controller_init()
     * They will be overwritten by constant values from menuconfig options or from macros.
     * So, do not modify the value when esp_bt_controller_init()
     */
    uint32_t magic;                         /*!< Magic number */
    uint32_t version;                       /*!< version number of the defined structure */
    /*
     * Following parameters can be configured runtime, when call esp_bt_controller_init()
     */
    uint16_t controller_task_stack_size;    /*!< Bluetooth controller task stack size */
    uint8_t controller_task_prio;           /*!< Bluetooth controller task priority */
    uint8_t controller_task_run_cpu;        /*!< CPU num that Bluetooth controller task runs on */
    uint8_t bluetooth_mode;                 /*!< Controller mode: BR/EDR, BLE or Dual Mode */
    uint8_t ble_max_act;                    /*!< BLE maximum number of air activities */
    uint8_t sleep_mode;                     /*!< controller sleep mode */
    uint8_t sleep_clock;                    /*!< controller sleep clock */
    uint8_t ble_st_acl_tx_buf_nb;           /*!< controller static ACL TX BUFFER number */
    uint8_t ble_hw_cca_check;               /*!< controller hardware triggered CCA check */
    uint16_t ble_adv_dup_filt_max;          /*!< maximum number of duplicate scan filter */
    bool coex_param_en;                     /*!< deprecated */
    uint8_t ce_len_type;                    /*!< connection event length computation method */
    bool coex_use_hooks;                    /*!< deprecated */
    uint8_t hci_tl_type;                    /*!< HCI transport layer, UART, VHCI, etc */
    esp_bt_hci_tl_t *hci_tl_funcs;          /*!< hci transport functions used, must be set when hci_tl_type is UART */
    uint8_t txant_dft;                      /*!< default Tx antenna */
    uint8_t rxant_dft;                      /*!< default Rx antenna */
    uint8_t txpwr_dft;                      /*!< default Tx power */
    uint32_t cfg_mask;                      /*!< Configuration mask to set specific options */
    uint8_t scan_duplicate_mode;            /*!< scan duplicate mode */
    uint8_t scan_duplicate_type;            /*!< scan duplicate type */
    uint16_t normal_adv_size;               /*!< Normal adv size for scan duplicate */
    uint16_t mesh_adv_size;                 /*!< Mesh adv size for scan duplicate */
    uint8_t coex_phy_coded_tx_rx_time_limit;  /*!< limit on max tx/rx time in case of connection using CODED-PHY with Wi-Fi coexistence */
    uint32_t hw_target_code;                /*!< hardware target */
    uint8_t slave_ce_len_min;               /*!< slave minimum ce length*/
    uint8_t hw_recorrect_en;                /*!< Hardware re-correction enabled */
    uint8_t cca_thresh;                     /*!< cca threshold*/
    uint16_t scan_backoff_upperlimitmax;    /*!< scan backoff upperlimitmax value */
    uint16_t dup_list_refresh_period;       /*!< duplicate scan list refresh time */
    bool ble_50_feat_supp;                  /*!< BLE 5.0 feature support */
    uint8_t ble_cca_mode;                   /*!< BLE CCA mode */
    uint8_t ble_data_lenth_zero_aux;        /*!< Config ext adv aux option */
    uint8_t ble_chan_ass_en;                /*!< BLE channel assessment enable */
    uint8_t ble_ping_en;                    /*!< BLE ping procedure enable */
    uint8_t ble_llcp_disc_flag;             /*!< BLE disconnect flag when instant passed */
    bool run_in_flash;                      /*!< Check if controller code is in flash */
    bool dtm_en;                            /*!< Controller DTM feature is enabled or not */
    bool enc_en;                            /*!< Controller encryption feature is enabled or not */
    bool qa_test;                           /*!< Controller QA test feature is enabled or not */
    bool master_en;                         /*!< Controller master feature is enabled or not */
    bool scan_en;                           /*!< Controller scan feature is enabled or not */
} esp_bt_controller_config_t;

/**
 * @brief Bluetooth controller enable/disable/initialised/de-initialised status
 */
typedef enum {
    ESP_BT_CONTROLLER_STATUS_IDLE = 0,
    ESP_BT_CONTROLLER_STATUS_INITED,
    ESP_BT_CONTROLLER_STATUS_ENABLED,
    ESP_BT_CONTROLLER_STATUS_NUM,
} esp_bt_controller_status_t;

/**
 * @brief BLE tx power type
 *        ESP_BLE_PWR_TYPE_CONN_HDL0-8: for each connection, and only be set after connection completed.
 *                                      when disconnect, the correspond TX power is not effected.
 *        ESP_BLE_PWR_TYPE_ADV : for advertising/scan response.
 *        ESP_BLE_PWR_TYPE_SCAN : for scan.
 *        ESP_BLE_PWR_TYPE_DEFAULT : if each connection's TX power is not set, it will use this default value.
 *                                   if neither in scan mode nor in adv mode, it will use this default value.
 *        If none of power type is set, system will use ESP_PWR_LVL_P3 as default for ADV/SCAN/CONN0-9.
 */
typedef enum {
    ESP_BLE_PWR_TYPE_CONN_HDL0  = 0,            /*!< For connection handle 0 */
    ESP_BLE_PWR_TYPE_CONN_HDL1  = 1,            /*!< For connection handle 1 */
    ESP_BLE_PWR_TYPE_CONN_HDL2  = 2,            /*!< For connection handle 2 */
    ESP_BLE_PWR_TYPE_CONN_HDL3  = 3,            /*!< For connection handle 3 */
    ESP_BLE_PWR_TYPE_CONN_HDL4  = 4,            /*!< For connection handle 4 */
    ESP_BLE_PWR_TYPE_CONN_HDL5  = 5,            /*!< For connection handle 5 */
    ESP_BLE_PWR_TYPE_CONN_HDL6  = 6,            /*!< For connection handle 6 */
    ESP_BLE_PWR_TYPE_CONN_HDL7  = 7,            /*!< For connection handle 7 */
    ESP_BLE_PWR_TYPE_CONN_HDL8  = 8,            /*!< For connection handle 8 */
    ESP_BLE_PWR_TYPE_ADV        = 9,            /*!< For advertising */
    ESP_BLE_PWR_TYPE_SCAN       = 10,           /*!< For scan */
    ESP_BLE_PWR_TYPE_DEFAULT    = 11,           /*!< For default, if not set other, it will use default value */
    ESP_BLE_PWR_TYPE_NUM        = 12,           /*!< TYPE numbers */
} esp_ble_power_type_t;

/**
 * @brief Bluetooth TX power level(index), it's just a index corresponding to power(dbm).
 */
typedef enum {
    ESP_PWR_LVL_N24 = 0,              /*!< Corresponding to -24dbm */
    ESP_PWR_LVL_N21 = 1,              /*!< Corresponding to -21dbm */
    ESP_PWR_LVL_N18 = 2,              /*!< Corresponding to -18dbm */
    ESP_PWR_LVL_N15 = 3,              /*!< Corresponding to -15dbm */
    ESP_PWR_LVL_N12 = 4,              /*!< Corresponding to -12dbm */
    ESP_PWR_LVL_N9  = 5,              /*!< Corresponding to -9dbm */
    ESP_PWR_LVL_N6  = 6,              /*!< Corresponding to -6dbm */
    ESP_PWR_LVL_N3  = 7,              /*!< Corresponding to -3dbm */
    ESP_PWR_LVL_N0  = 8,              /*!< Corresponding to 0dbm */
    ESP_PWR_LVL_P3  = 9,              /*!< Corresponding to +3dbm */
    ESP_PWR_LVL_P6  = 10,             /*!< Corresponding to +6dbm */
    ESP_PWR_LVL_P9  = 11,             /*!< Corresponding to +9dbm */
    ESP_PWR_LVL_P12 = 12,             /*!< Corresponding to +12dbm */
    ESP_PWR_LVL_P15 = 13,             /*!< Corresponding to +15dbm */
    ESP_PWR_LVL_P18 = 14,             /*!< Corresponding to +18dbm */
    ESP_PWR_LVL_P20 = 15,             /*!< Corresponding to +20dbm */
    ESP_PWR_LVL_P21 = 15,             /*!< Corresponding to +20dbm, this enum variable has been deprecated */
    ESP_PWR_LVL_INVALID = 0xFF,         /*!< Indicates an invalid value */
} esp_power_level_t;

/**
 * @brief The enhanced type of which tx power, could set Advertising/Connection/Default and etc.
 */
typedef enum {
    ESP_BLE_ENHANCED_PWR_TYPE_DEFAULT = 0,
    ESP_BLE_ENHANCED_PWR_TYPE_ADV,
    ESP_BLE_ENHANCED_PWR_TYPE_SCAN,
    ESP_BLE_ENHANCED_PWR_TYPE_INIT,
    ESP_BLE_ENHANCED_PWR_TYPE_CONN,
    ESP_BLE_ENHANCED_PWR_TYPE_MAX,
} esp_ble_enhanced_power_type_t;

/**
 * @brief  Set BLE TX power
 *         Connection Tx power should only be set after connection created.
 * @param  power_type : The type of which tx power, could set Advertising/Connection/Default and etc
 * @param  power_level: Power level(index) corresponding to absolute value(dbm)
 * @return              ESP_OK - success, other - failed
 */
esp_err_t esp_ble_tx_power_set(esp_ble_power_type_t power_type, esp_power_level_t power_level);

/**
 * @brief  Get BLE TX power
 *         Connection Tx power should only be get after connection created.
 * @param  power_type : The type of which tx power, could set Advertising/Connection/Default and etc
 * @return             >= 0 - Power level, < 0 - Invalid
 */
esp_power_level_t esp_ble_tx_power_get(esp_ble_power_type_t power_type);

/**
 * @brief  ENHANCED API for Setting BLE TX power
 *         Connection Tx power should only be set after connection created.
 * @param  power_type : The enhanced type of which tx power, could set Advertising/Connection/Default and etc.
 * @param  handle : The handle of Advertising or Connection and the value 0 for other enhanced power types.
 * @param  power_level: Power level(index) corresponding to absolute value(dbm)
 * @return              ESP_OK - success, other - failed
 */
esp_err_t esp_ble_tx_power_set_enhanced(esp_ble_enhanced_power_type_t power_type, uint16_t handle, esp_power_level_t power_level);

/**
 * @brief  ENHANCED API of Getting BLE TX power
 *         Connection Tx power should only be get after connection created.
 * @param  power_type : The enhanced type of which tx power, could set Advertising/Connection/Default and etc
 * @param  handle : The handle of Advertising or Connection and the value 0 for other enhanced power types.
 * @return             >= 0 - Power level, < 0 - Invalid
 */
esp_power_level_t esp_ble_tx_power_get_enhanced(esp_ble_enhanced_power_type_t power_type, uint16_t handle);

/**
 * @brief       Initialize BT controller to allocate task and other resource.
 *              This function should be called only once, before any other BT functions are called.
 * @param  cfg: Initial configuration of BT controller. Different from previous version, there's a mode and some
 *              connection configuration in "cfg" to configure controller work mode and allocate the resource which is needed.
 * @return      ESP_OK - success, other - failed
 */
esp_err_t esp_bt_controller_init(esp_bt_controller_config_t *cfg);

/**
 * @brief  De-initialize BT controller to free resource and delete task.
 *         You should stop advertising and scanning, as well as
 *         disconnect all existing connections before de-initializing BT controller.
 *
 * This function should be called only once, after any other BT functions are called.
 * This function is not whole completed, esp_bt_controller_init cannot called after this function.
 * @return  ESP_OK - success, other - failed
 */
esp_err_t esp_bt_controller_deinit(void);

/**
 * @brief Enable BT controller.
 *               Due to a known issue, you cannot call esp_bt_controller_enable() a second time
 *               to change the controller mode dynamically. To change controller mode, call
 *               esp_bt_controller_disable() and then call esp_bt_controller_enable() with the new mode.
 * @param mode : the mode(BLE/BT/BTDM) to enable. For compatible of API, retain this argument. This mode must be
 *               equal as the mode in "cfg" of esp_bt_controller_init().
 * @return       ESP_OK - success, other - failed
 */
esp_err_t esp_bt_controller_enable(esp_bt_mode_t mode);

/**
 * @brief  Disable BT controller
 * @return       ESP_OK - success, other - failed
 */
esp_err_t esp_bt_controller_disable(void);

/**
 * @brief  Get BT controller is initialised/de-initialised/enabled/disabled
 * @return status value
 */
esp_bt_controller_status_t esp_bt_controller_get_status(void);

/** @brief esp_vhci_host_callback
 *  used for vhci call host function to notify what host need to do
 */
typedef struct esp_vhci_host_callback {
    void (*notify_host_send_available)(void);               /*!< callback used to notify that the host can send packet to controller */
    int (*notify_host_recv)(uint8_t *data, uint16_t len);   /*!< callback used to notify that the controller has a packet to send to the host*/
} esp_vhci_host_callback_t;

/** @brief esp_vhci_host_check_send_available
 *  used for check actively if the host can send packet to controller or not.
 *  @return true for ready to send, false means cannot send packet
 */
bool esp_vhci_host_check_send_available(void);

/** @brief esp_vhci_host_send_packet
 * host send packet to controller
 *
 * Should not call this function from within a critical section
 * or when the scheduler is suspended.
 *
 * @param data the packet point
 * @param len the packet length
 */
void esp_vhci_host_send_packet(uint8_t *data, uint16_t len);

/** @brief esp_vhci_host_register_callback
 * register the vhci reference callback
 * struct defined by vhci_host_callback structure.
 * @param callback esp_vhci_host_callback type variable
 * @return ESP_OK - success, ESP_FAIL - failed
 */
esp_err_t esp_vhci_host_register_callback(const esp_vhci_host_callback_t *callback);

/** @brief esp_bt_controller_mem_release
 * release the controller memory as per the mode
 *
 * This function releases the BSS, data and other sections of the controller to heap. The total size is about 70k bytes.
 *
 * esp_bt_controller_mem_release(mode) should be called only before esp_bt_controller_init()
 * or after esp_bt_controller_deinit().
 *
 * Note that once BT controller memory is released, the process cannot be reversed. It means you cannot use the bluetooth
 * mode which you have released by this function.
 *
 * If your firmware will later upgrade the Bluetooth controller mode (BLE -> BT Classic or disabled -> enabled)
 * then do not call this function.
 *
 * If the app calls esp_bt_controller_enable(ESP_BT_MODE_BLE) to use BLE only then it is safe to call
 * esp_bt_controller_mem_release(ESP_BT_MODE_CLASSIC_BT) at initialization time to free unused BT Classic memory.
 *
 * If the mode is ESP_BT_MODE_BTDM, then it may be useful to call API esp_bt_mem_release(ESP_BT_MODE_BTDM) instead,
 * which internally calls esp_bt_controller_mem_release(ESP_BT_MODE_BTDM) and additionally releases the BSS and data
 * consumed by the BT/BLE host stack to heap. For more details about usage please refer to the documentation of
 * esp_bt_mem_release() function
 *
 * @param mode : the mode want to release memory
 * @return ESP_OK - success, other - failed
 */
esp_err_t esp_bt_controller_mem_release(esp_bt_mode_t mode);

/** @brief esp_bt_mem_release
 * release controller memory and BSS and data section of the BT/BLE host stack as per the mode
 *
 * This function first releases controller memory by internally calling esp_bt_controller_mem_release().
 * Additionally, if the mode is set to ESP_BT_MODE_BTDM, it also releases the BSS and data consumed by the BT/BLE host stack to heap
 *
 * Note that once BT memory is released, the process cannot be reversed. It means you cannot use the bluetooth
 * mode which you have released by this function.
 *
 * If your firmware will later upgrade the Bluetooth controller mode (BLE -> BT Classic or disabled -> enabled)
 * then do not call this function.
 *
 * If you never intend to use bluetooth in a current boot-up cycle, you can call esp_bt_mem_release(ESP_BT_MODE_BTDM)
 * before esp_bt_controller_init or after esp_bt_controller_deinit.
 *
 * For example, if a user only uses bluetooth for setting the WiFi configuration, and does not use bluetooth in the rest of the product operation".
 * In such cases, after receiving the WiFi configuration, you can disable/deinit bluetooth and release its memory.
 * Below is the sequence of APIs to be called for such scenarios:
 *
 *      esp_bluedroid_disable();
 *      esp_bluedroid_deinit();
 *      esp_bt_controller_disable();
 *      esp_bt_controller_deinit();
 *      esp_bt_mem_release(ESP_BT_MODE_BTDM);
 *
 * @param mode : the mode whose memory is to be released
 * @return ESP_OK - success, other - failed
 */
esp_err_t esp_bt_mem_release(esp_bt_mode_t mode);

/**
 * @brief enable bluetooth to enter modem sleep
 *
 * Note that this function shall not be invoked before esp_bt_controller_enable()
 *
 * There are currently two options for bluetooth modem sleep, one is ORIG mode, and another is EVED Mode. EVED Mode is intended for BLE only.
 *
 * For ORIG mode:
 * Bluetooth modem sleep is enabled in controller start up by default if CONFIG_BTDM_CONTROLLER_MODEM_SLEEP is set and "ORIG mode" is selected. In ORIG modem sleep mode, bluetooth controller will switch off some components and pause to work every now and then, if there is no event to process; and wakeup according to the scheduled interval and resume the work. It can also wakeup earlier upon external request using function "esp_bt_controller_wakeup_request".
 *
 * @return
 *                  - ESP_OK : success
 *                  - other  : failed
 */
esp_err_t esp_bt_sleep_enable(void);


/**
 * @brief disable bluetooth modem sleep
 *
 * Note that this function shall not be invoked before esp_bt_controller_enable()
 *
 * If esp_bt_sleep_disable() is called, bluetooth controller will not be allowed to enter modem sleep;
 *
 * If ORIG modem sleep mode is in use, if this function is called, bluetooth controller may not immediately wake up if it is dormant then.
 * In this case, esp_bt_controller_wakeup_request() can be used to shorten the time for wakeup.
 *
 * @return
 *                  - ESP_OK : success
 *                  - other  : failed
 */
esp_err_t esp_bt_sleep_disable(void);

/**
 * @brief to check whether bluetooth controller is sleeping at the instant, if modem sleep is enabled
 *
 * Note that this function shall not be invoked before esp_bt_controller_enable()
 * This function is supposed to be used ORIG mode of modem sleep
 *
 * @return  true if in modem sleep state, false otherwise
 */
bool esp_bt_controller_is_sleeping(void);

/**
 * @brief request controller to wakeup from sleeping state during sleep mode
 *
 * Note that this function shall not be invoked before esp_bt_controller_enable()
 * Note that this function is supposed to be used ORIG mode of modem sleep
 * Note that after this request, bluetooth controller may again enter sleep as long as the modem sleep is enabled
 *
 * Profiling shows that it takes several milliseconds to wakeup from modem sleep after this request.
 * Generally it takes longer if 32kHz XTAL is used than the main XTAL, due to the lower frequency of the former as the bluetooth low power clock source.
 */
void esp_bt_controller_wakeup_request(void);

/**
 * @brief notify bluetooth controller task to process the event upon Tx or Rx done
 *
 * Note that this function shall not be invoked before esp_bt_controller_enable()
 * This function can be called in both ISR and non-ISR context
 *
 */
int esp_bt_h4tl_eif_io_event_notify(int event);

/**
 * @brief Get the Bluetooth module sleep clock source.
 *
 * Note that this function shall not be invoked before esp_bt_controller_init()
 *
 * @return  clock source used in Bluetooth low power mode
 */
esp_bt_sleep_clock_t esp_bt_get_lpclk_src(void);

#ifdef __cplusplus
}
#endif

#endif /* __ESP_BT_H__ */
