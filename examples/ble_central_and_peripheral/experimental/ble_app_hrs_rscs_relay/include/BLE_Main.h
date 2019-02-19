#ifndef __BLE_MAIN_H__
#define __BLE_MAIN_H__

#include "SEGGER_RTT.h"
#include "SEGGER_RTT_Conf.h"

#define __DEBUG_LOG__				1



#if (NRF_SD_BLE_API_VERSION == 3)
#define NRF_BLE_MAX_MTU_SIZE        GATT_MTU_SIZE_DEFAULT                         /**< MTU size used in the softdevice enabling and to reply to a BLE_GATTS_EVT_EXCHANGE_MTU_REQUEST event. */
#endif

#define APP_FEATURE_NOT_SUPPORTED   BLE_GATT_STATUS_ATTERR_APP_BEGIN + 2          /**< Reply when unsupported features are requested. */

#define CENTRAL_LINK_COUNT          2                                             /**< Number of central links used by the application. When changing this number remember to adjust the RAM settings*/
#define PERIPHERAL_LINK_COUNT       1                                             /**< Number of peripheral links used by the application. When changing this number remember to adjust the RAM settings*/

#define UART_TX_BUF_SIZE            256                                           /**< Size of the UART TX buffer, in bytes. Must be a power of two. */
#define UART_RX_BUF_SIZE            1                                             /**< Size of the UART RX buffer, in bytes. Must be a power of two. */

/* Central related. */

#define APP_TIMER_PRESCALER         0                                             /**< Value of the RTC1 PRESCALER register. */
#define APP_TIMER_MAX_TIMERS        (2 + BSP_APP_TIMERS_NUMBER)                   /**< Maximum number of timers used by the application. */
#define APP_TIMER_OP_QUEUE_SIZE     2                                             /**< Size of timer operation queues. */

#define SEC_PARAM_BOND              1                                             /**< Perform bonding. */
#define SEC_PARAM_MITM              0                                             /**< Man In The Middle protection not required. */
#define SEC_PARAM_LESC              0                                             /**< LE Secure Connections not enabled. */
#define SEC_PARAM_KEYPRESS          0                                             /**< Keypress notifications not enabled. */
#define SEC_PARAM_IO_CAPABILITIES   BLE_GAP_IO_CAPS_NONE                          /**< No I/O capabilities. */
#define SEC_PARAM_OOB               0                                             /**< Out Of Band data not available. */
#define SEC_PARAM_MIN_KEY_SIZE      7                                             /**< Minimum encryption key size in octets. */
#define SEC_PARAM_MAX_KEY_SIZE      16                                            /**< Maximum encryption key size in octets. */

#define SCAN_INTERVAL               0x00A0                                        /**< Determines scan interval in units of 0.625 millisecond. */
#define SCAN_WINDOW                 0x0050                                        /**< Determines scan window in units of 0.625 millisecond. */
#define SCAN_TIMEOUT                0

#define MIN_CONNECTION_INTERVAL     (uint16_t) MSEC_TO_UNITS(7.5, UNIT_1_25_MS)   /**< Determines minimum connection interval in milliseconds. */
#define MAX_CONNECTION_INTERVAL     (uint16_t) MSEC_TO_UNITS(30, UNIT_1_25_MS)    /**< Determines maximum connection interval in milliseconds. */
#define SLAVE_LATENCY               0                                             /**< Determines slave latency in terms of connection events. */
#define SUPERVISION_TIMEOUT         (uint16_t) MSEC_TO_UNITS(4000, UNIT_10_MS)    /**< Determines supervision time-out in units of 10 milliseconds. */

#define UUID16_SIZE                 2                                             /**< Size of a UUID, in bytes. */

/**@brief Macro to unpack 16bit unsigned UUID from an octet stream. */
#define UUID16_EXTRACT(DST, SRC) \
    do                           \
    {                            \
        (*(DST))   = (SRC)[1];   \
        (*(DST)) <<= 8;          \
        (*(DST))  |= (SRC)[0];   \
    } while (0)

/**@brief Variable length data encapsulation in terms of length and pointer to data. */
typedef struct
{
    uint8_t     * p_data;    /**< Pointer to data. */
    uint16_t      data_len;  /**< Length of data. */
} data_t;




/* Peripheral related. */

#define DEVICE_NAME                      "Moqi_UART"
/**< 蓝牙广播名称, 目前支持26个字符 */
#define MANUFACTURER_NAME                "NordicSemiconductor"
/**< Manufacturer. Will be passed to Device Information Service. */
#define APP_ADV_INTERVAL                 300
/**< The advertising interval (in units of 0.625 ms). This value corresponds to 187.5 ms. */
#define APP_ADV_TIMEOUT_IN_SECONDS       180
/**< The advertising timeout in units of seconds. */

#define FIRST_CONN_PARAMS_UPDATE_DELAY   APP_TIMER_TICKS(5000, APP_TIMER_PRESCALER)
/**< Time from initiating event (connect or start of notification) to first time sd_ble_gap_conn_param_update is called (5 seconds). */
#define NEXT_CONN_PARAMS_UPDATE_DELAY    APP_TIMER_TICKS(30000, APP_TIMER_PRESCALER)
/**< Time between each call to sd_ble_gap_conn_param_update after the first call (30 seconds). */
#define MAX_CONN_PARAMS_UPDATE_COUNT     3
/**< Number of attempts before giving up the connection parameter negotiation. */


/*********************** 调试 ***********************/
#if __DEBUG_LOG__
#define DEBUG_LOG(format, ...) SEGGER_RTT_printf(0, format, ##__VA_ARGS__)
#else
#define DEBUG_LOG(format, ...)
#endif
/****************************************************/


#endif
/* __BLE_MAIN_H__ */
