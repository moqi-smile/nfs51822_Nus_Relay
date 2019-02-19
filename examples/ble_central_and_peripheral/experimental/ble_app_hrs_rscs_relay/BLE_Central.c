#include <stdint.h>
#include <stdio.h>
#include <string.h>
#include "nordic_common.h"
#include "softdevice_handler.h"
#include "peer_manager.h"
#include "app_timer.h"
#include "boards.h"
#include "bsp.h"
#include "bsp_btn_ble.h"
#include "ble.h"
#include "ble_advdata.h"
#include "ble_advertising.h"
#include "ble_conn_params.h"
#include "ble_db_discovery.h"

#include "ble_conn_state.h"
#include "fstorage.h"
#include "fds.h"

#include "BLE_Main.h"
#include "BLE_Central.h"

#define NRF_LOG_MODULE_NAME "BLE_Central"
#include "nrf_log.h"
#include "nrf_log_ctrl.h"

uint16_t CentralIndex = 0;
BLE_CentralHandler CentralHandler[CENTRAL_LINK_COUNT+PERIPHERAL_LINK_COUNT];

/**
 * @brief Parameters used when scanning.
 */
static const ble_gap_scan_params_t m_scan_params =
{
    .active   = 1,
    .interval = SCAN_INTERVAL,
    .window   = SCAN_WINDOW,
    .timeout  = SCAN_TIMEOUT,
    #if (NRF_SD_BLE_API_VERSION == 2)
        .selective   = 0,
        .p_whitelist = NULL,
    #endif
    #if (NRF_SD_BLE_API_VERSION == 3)
        .use_whitelist = 0,
    #endif
};

static const ble_gap_conn_params_t m_connection_param =
{
	(uint16_t)MIN_CONNECTION_INTERVAL,  // Minimum connection
	(uint16_t)MAX_CONNECTION_INTERVAL,  // Maximum connection
	(uint16_t)SLAVE_LATENCY,            // Slave latency
	(uint16_t)SUPERVISION_TIMEOUT       // Supervision time-out
};

void BLE_CentralConnect(ble_gap_addr_t peer_addr)
{
    uint32_t err_code;

	err_code = sd_ble_gap_connect(&peer_addr,
								  &m_scan_params,
								  &m_connection_param);

	if (err_code == NRF_SUCCESS)
	{
		// scan is automatically stopped by the connect

		DEBUG_LOG ("Connecting to target %02x%02x%02x%02x%02x%02x\r\n",
				 peer_addr.addr[0], peer_addr.addr[1],
				 peer_addr.addr[2], peer_addr.addr[3],
				 peer_addr.addr[4], peer_addr.addr[5]
				 );
	}
}

void BLE_CentralScanStart(void)
{
    ret_code_t ret;

	DEBUG_LOG("BLE_CentralScanStart\r\n");

    ret = sd_ble_gap_scan_start(&m_scan_params);
    APP_ERROR_CHECK(ret);

    ret = bsp_indication_set(BSP_INDICATE_SCANNING);
    APP_ERROR_CHECK(ret);
}

void BLE_CentralStopScan(void)
{
	uint32_t err_code;

	err_code = sd_ble_gap_scan_stop();

	if (err_code != NRF_SUCCESS)
	{
		DEBUG_LOG("[APPL]: Scan stop failed, reason %d\r\n", err_code);
	}
}

__weak void BLE_CentralAdv(const ble_gap_evt_adv_report_t * p_adv_report)
{
	
}

__weak void BLE_CentralHandleEven(const ble_gap_evt_adv_report_t * p_adv_report)
{
	
}

void BLE_CentralEven(ble_evt_t * p_ble_evt)
{
    uint32_t              err_code;
    const ble_gap_evt_t * p_gap_evt = &p_ble_evt->evt.gap_evt;

    switch (p_ble_evt->header.evt_id)
    {
        case BLE_GAP_EVT_ADV_REPORT:
        {
			BLE_CentralAdv(&p_gap_evt->params.adv_report);
        }break; // BLE_GAP_EVT_ADV_REPORT

        case BLE_GAP_EVT_CONNECTED:
            DEBUG_LOG("Central to target\r\n");

			APP_ERROR_CHECK_BOOL(p_gap_evt->conn_handle < CENTRAL_LINK_COUNT + PERIPHERAL_LINK_COUNT);
			err_code = ble_db_discovery_start(&CentralHandler[p_gap_evt->conn_handle].Db_Discovery, p_gap_evt->conn_handle);
			APP_ERROR_CHECK(err_code);
			DEBUG_LOG("try to find Nuc on conn_handle 0x%x\r\n", p_gap_evt->conn_handle);
			if (p_gap_evt->conn_handle < CENTRAL_LINK_COUNT)
			{
				DEBUG_LOG("ReStart Scan\n");
				BLE_CentralScanStart();
			}

            break; // BLE_GAP_EVT_CONNECTED

        case BLE_GAP_EVT_TIMEOUT:
            if (p_gap_evt->params.timeout.src == BLE_GAP_TIMEOUT_SRC_SCAN)
            {
                DEBUG_LOG("Scan timed out.\r\n");
                BLE_CentralScanStart();
            }
            else if (p_gap_evt->params.timeout.src == BLE_GAP_TIMEOUT_SRC_CONN)
            {
                DEBUG_LOG("Connection Request timed out.\r\n");
            }
            break; // BLE_GAP_EVT_TIMEOUT

        case BLE_GAP_EVT_SEC_PARAMS_REQUEST:
            // Pairing not supported
            err_code = sd_ble_gap_sec_params_reply(p_ble_evt->evt.gap_evt.conn_handle, BLE_GAP_SEC_STATUS_PAIRING_NOT_SUPP, NULL, NULL);
            APP_ERROR_CHECK(err_code);
            break; // BLE_GAP_EVT_SEC_PARAMS_REQUEST

        case BLE_GAP_EVT_CONN_PARAM_UPDATE_REQUEST:
            // Accepting parameters requested by peer.
            err_code = sd_ble_gap_conn_param_update(p_gap_evt->conn_handle,
                                                    &p_gap_evt->params.conn_param_update_request.conn_params);
            APP_ERROR_CHECK(err_code);
            break; // BLE_GAP_EVT_CONN_PARAM_UPDATE_REQUEST

        case BLE_GATTC_EVT_TIMEOUT:
            // Disconnect on GATT Client timeout event.
            DEBUG_LOG("GATT Client Timeout.\r\n");
            err_code = sd_ble_gap_disconnect(p_ble_evt->evt.gattc_evt.conn_handle,
                                             BLE_HCI_REMOTE_USER_TERMINATED_CONNECTION);
            APP_ERROR_CHECK(err_code);
            break; // BLE_GATTC_EVT_TIMEOUT

        case BLE_GATTS_EVT_TIMEOUT:
            // Disconnect on GATT Server timeout event.
            DEBUG_LOG("GATT Server Timeout.\r\n");
            err_code = sd_ble_gap_disconnect(p_ble_evt->evt.gatts_evt.conn_handle,
                                             BLE_HCI_REMOTE_USER_TERMINATED_CONNECTION);
            APP_ERROR_CHECK(err_code);
            break; // BLE_GATTS_EVT_TIMEOUT

#if (NRF_SD_BLE_API_VERSION == 3)
        case BLE_GATTS_EVT_EXCHANGE_MTU_REQUEST:
            err_code = sd_ble_gatts_exchange_mtu_reply(p_ble_evt->evt.gatts_evt.conn_handle,
                                                       NRF_BLE_MAX_MTU_SIZE);
            APP_ERROR_CHECK(err_code);
            break; // BLE_GATTS_EVT_EXCHANGE_MTU_REQUEST
#endif

        default:
            break;
    }
}

static void db_disc_handler(ble_db_discovery_evt_t * p_evt)
{
    ble_nus_c_on_db_disc_evt(&CentralHandler[p_evt->conn_handle].Nus_c, p_evt);
}

static void db_discovery_init(void)
{
    uint32_t err_code = ble_db_discovery_init(db_disc_handler);

    APP_ERROR_CHECK(err_code);
}

static void ble_nus_c_evt_handler(ble_nus_c_t * p_ble_nus_c, const ble_nus_c_evt_t * p_ble_nus_evt)
{
    uint32_t err_code;

    switch (p_ble_nus_evt->evt_type)
    {
        case BLE_NUS_C_EVT_DISCOVERY_COMPLETE:
            DEBUG_LOG("The device has the Nordic UART Service\r\n");
            err_code = ble_nus_c_handles_assign(p_ble_nus_c, p_ble_nus_evt->conn_handle, &p_ble_nus_evt->handles);
            APP_ERROR_CHECK(err_code);

            err_code = ble_nus_c_rx_notif_enable(p_ble_nus_c);
            APP_ERROR_CHECK(err_code);
            break;

        case BLE_NUS_C_EVT_NUS_RX_EVT:
			DEBUG_LOG("Nus %d Recv Data\r\n", p_ble_nus_c->conn_handle);

            break;

        case BLE_NUS_C_EVT_DISCONNECTED:
            DEBUG_LOG("Disconnected\r\n");
			BLE_CentralScanStart();
            break;
    }
}

static void nus_c_init(void)
{
	uint16_t Index;
    uint32_t         err_code;
    ble_nus_c_init_t nus_c_init_t;

    nus_c_init_t.evt_handler = ble_nus_c_evt_handler;

	for (Index=PERIPHERAL_LINK_COUNT; Index<CENTRAL_LINK_COUNT+PERIPHERAL_LINK_COUNT; Index++)
	{
		CentralHandler[Index].conn_handle = BLE_CONN_HANDLE_INVALID;
		err_code = ble_nus_c_init(&CentralHandler[Index].Nus_c, &nus_c_init_t);
		APP_ERROR_CHECK(err_code);
	}
}

void BLE_CentralInit(void)
{
    db_discovery_init();
    nus_c_init();
	
    DEBUG_LOG ("BLE_CentralInit\n");
}

