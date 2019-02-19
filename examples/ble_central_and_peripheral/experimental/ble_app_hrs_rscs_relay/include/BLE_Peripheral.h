#ifndef __BLE_PERIPHERAL_H__
#define __BLE_PERIPHERAL_H__

#include "ble_nus.h"

#define BLE_SERIVERS_NUM	1

typedef struct{
	ble_nus_t Nus;
	ble_uuid_t Uuids[1];
}BLE_PeripheralHandler;

extern BLE_PeripheralHandler PeripheralHandler;

//extern ble_nus_t                        m_nus;

void BLE_PeripheralInit(void);
void BLE_PeripheralEven(ble_evt_t * p_ble_evt);

#endif
/* __BLE_PERIPHERAL_H__ */
