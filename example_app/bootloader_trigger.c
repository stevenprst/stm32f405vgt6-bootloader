/*
 * What an application needs so the CAN bootloader can reflash it. Add this to the app's main.c (USER CODE blocks)
 * or to its own file, and read example_app/README.md for the linker and vector table changes.
 *
 * The app listens on CAN1 for the trigger frame:  ID 0x6A4, data = node id, then 0x07, 0xB0 (magic 0xB007, little-endian).
 * It then leaves the stay token in SRAM and resets. The bootloader sees the token and waits up to 30 s for the host
 * instead of the normal 250 ms.
 */

#include "main.h"

#define BOOTLOADER_NODE_ID		1		/* must equal BOOTLOADER_NODE_ID in the bootloader */
#define BOOTLOADER_TRIGGER_CAN_ID	0x6A4
#define BOOTLOADER_TRIGGER_MAGIC_LOW	0x07
#define BOOTLOADER_TRIGGER_MAGIC_HIGH	0xB0
#define BOOTLOADER_STAY_TOKEN		0x53544159u
#define BOOTLOADER_STAY_TOKEN_ADDRESS	(SRAM_BASE + 0x3004)

/* Call once after HAL_CAN_Start(): pass only the trigger frame into FIFO0 and enable the receive interrupt.
 * The app's own CAN IDs need their own filter banks or a wider filter. */
void bootloaderTriggerInit(CAN_HandleTypeDef *hcan)
{
	CAN_FilterTypeDef filter = {0};
	filter.FilterIdHigh = BOOTLOADER_TRIGGER_CAN_ID << 5;
	filter.FilterIdLow = 0x0000;
	filter.FilterMaskIdHigh = 0x7FF << 5;
	filter.FilterMaskIdLow = CAN_ID_EXT | CAN_RTR_REMOTE;
	filter.FilterFIFOAssignment = CAN_FILTER_FIFO0;
	filter.FilterBank = 0;
	filter.FilterMode = CAN_FILTERMODE_IDMASK;
	filter.FilterScale = CAN_FILTERSCALE_32BIT;
	filter.FilterActivation = CAN_FILTER_ENABLE;
	filter.SlaveStartFilterBank = 14;

	HAL_CAN_ConfigFilter(hcan, &filter);
	HAL_CAN_ActivateNotification(hcan, CAN_IT_RX_FIFO0_MSG_PENDING);
}

void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef *hcan)
{
	CAN_RxHeaderTypeDef header;
	uint8_t data[8];

	if (HAL_CAN_GetRxMessage(hcan, CAN_RX_FIFO0, &header, data) != HAL_OK) {
		return;
	}

	const int is_trigger_frame = header.IDE == CAN_ID_STD && header.RTR == CAN_RTR_DATA
				     && header.StdId == BOOTLOADER_TRIGGER_CAN_ID && header.DLC >= 3;

	if (!is_trigger_frame) {
		return;
	}

	const int for_this_node = data[0] == BOOTLOADER_NODE_ID;
	const int magic_matches = data[1] == BOOTLOADER_TRIGGER_MAGIC_LOW && data[2] == BOOTLOADER_TRIGGER_MAGIC_HIGH;

	if (!for_this_node || !magic_matches) {
		return;
	}

	/* The next reset starts the bootloader, which finds the token and waits for the host. */
	*(volatile uint32_t *)BOOTLOADER_STAY_TOKEN_ADDRESS = BOOTLOADER_STAY_TOKEN;
	NVIC_SystemReset();
}
