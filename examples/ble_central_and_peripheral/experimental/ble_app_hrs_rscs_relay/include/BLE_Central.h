#ifndef __BLE_CENTRAL_H__
#define __BLE_CENTRAL_H__

#include "ble_nus_c.h"

typedef struct{
	ble_nus_c_t Nus_c;
	uint16_t conn_handle;
	ble_db_discovery_t Db_Discovery;
}BLE_CentralHandler;

extern uint16_t CentralIndex;
extern BLE_CentralHandler CentralHandler[];

void BLE_CentralInit(void);
void BLE_CentralEven(ble_evt_t * p_ble_evt);

void BLE_CentralConnect(ble_gap_addr_t peer_addr);
void BLE_CentralScanStart(void);
void BLE_CentralStopScan(void);


#endif
/* __BLE_CENTRAL_H__ */
