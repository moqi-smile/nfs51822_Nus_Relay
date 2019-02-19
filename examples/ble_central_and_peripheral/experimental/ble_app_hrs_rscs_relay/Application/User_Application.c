#include "string.h"

#include "BLE_Main.h"
#include "BLE_Central.h"
#include "BLE_Peripheral.h"

#define SCAN_NAME           		""
 
void BLE_CentralAdv(const ble_gap_evt_adv_report_t * p_adv_report)
{
	char			* p_adv_neme = NULL;

	p_adv_neme=strstr((char *)p_adv_report->data,(char *)SCAN_NAME);

	if (p_adv_neme!=NULL)
	{
		DEBUG_LOG("SCAN_NAME is %s \r\n", p_adv_neme);

		BLE_CentralConnect(p_adv_report->peer_addr);
	}
}

void User_ApplicationInit(void)
{
	DEBUG_LOG("User_ApplicationInit\r\n");
}

void User_ApplicationFunc(void)
{
	
}
