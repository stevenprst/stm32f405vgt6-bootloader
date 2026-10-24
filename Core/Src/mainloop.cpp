#include "mainloop.hpp"

#include <cstring>

namespace can_bootloader
{

namespace
{

// The one bootloader of this image.
Bootloader bootloader;

void sendResponse(Command command, uint16_t index, ResponseCode response_code, const uint8_t payload[3] = nullptr)
{
	ResponseFrame response{};
	response.node_id = this_node_id;
	response.command = static_cast<uint8_t>(command);
	response.code = static_cast<uint8_t>(response_code);
	response.index = index;

	if (payload != nullptr) {
		memcpy(response.payload, payload, sizeof(response.payload));
	}

	CAN_TxHeaderTypeDef header{};
	header.StdId = response_can_id;
	header.IDE = CAN_ID_STD;
	header.RTR = CAN_RTR_DATA;
	header.DLC = sizeof(response);

	uint32_t mailbox = 0;
	const HAL_StatusTypeDef queued = HAL_CAN_AddTxMessage(&hcan1, &header, reinterpret_cast<const uint8_t *>(&response),
							      &mailbox);

	if (queued != HAL_OK) {
		return;
	}

	// Wait for the frame to leave. With no other node on the bus it is never acknowledged, so give up after a while.
	const uint32_t queued_ms = HAL_GetTick();

	while (HAL_CAN_IsTxMessagePending(&hcan1, mailbox) != 0) {
		const bool waited_too_long = (HAL_GetTick() - queued_ms) > reply_timeout_ms;

		if (waited_too_long) {
			return;
		}
	}
}

} // namespace

void Bootloader::init()
{
	// Pass only command frames: standard ID command_can_id, data frame. The mask bits are the ID, IDE and RTR bits.
	CAN_FilterTypeDef filter{};
	filter.FilterIdHigh = command_can_id << 5;
	filter.FilterIdLow = 0x0000;
	filter.FilterMaskIdHigh = 0x7FF << 5;
	filter.FilterMaskIdLow = CAN_ID_EXT | CAN_RTR_REMOTE;
	filter.FilterFIFOAssignment = CAN_FILTER_FIFO0;
	filter.FilterBank = 0;
	filter.FilterMode = CAN_FILTERMODE_IDMASK;
	filter.FilterScale = CAN_FILTERSCALE_32BIT;
	filter.FilterActivation = CAN_FILTER_ENABLE;

	// All filter banks belong to CAN1 because CAN2 is not used.
	filter.SlaveStartFilterBank = 14;

	const bool filter_set = HAL_CAN_ConfigFilter(&hcan1, &filter) == HAL_OK;
	const bool started = filter_set && HAL_CAN_Start(&hcan1) == HAL_OK;
	const bool notified = started && HAL_CAN_ActivateNotification(&hcan1, CAN_IT_RX_FIFO0_MSG_PENDING) == HAL_OK;

	// Without CAN the host cannot be reached.
	_can_ready = notified;
}

void Bootloader::onCanFrameReceived(CAN_HandleTypeDef *can)
{
	CAN_RxHeaderTypeDef header;
	uint8_t data[8];

	if (HAL_CAN_GetRxMessage(can, CAN_RX_FIFO0, &header, data) != HAL_OK) {
		return;
	}

	const bool is_command_frame = header.IDE == CAN_ID_STD && header.RTR == CAN_RTR_DATA
				      && header.StdId == command_can_id && header.DLC == sizeof(CommandFrame);

	if (!is_command_frame) {
		return;
	}

	CommandFrame frame;
	memcpy(&frame, data, sizeof(frame));

	const bool for_this_node = frame.target_node_id == this_node_id;

	if (!for_this_node) {
		return;
	}

	// A host that sends more than ring_size frames without waiting for replies loses the newest ones.
	const uint32_t head = _ring_head.load(std::memory_order_relaxed);
	const uint32_t tail = _ring_tail.load(std::memory_order_acquire);
	const bool ring_full = (head - tail) >= ring_size;

	if (ring_full) {
		return;
	}

	_ring[head % ring_size] = frame;
	_ring_head.store(head + 1, std::memory_order_release);
}

void Bootloader::run()
{
	HAL_IWDG_Refresh(&hiwdg);

	if (!_can_ready) {
		return;
	}

	const uint32_t tail = _ring_tail.load(std::memory_order_relaxed);
	const uint32_t head = _ring_head.load(std::memory_order_acquire);
	const bool frame_waiting = head != tail;

	if (frame_waiting) {
		const CommandFrame frame = _ring[tail % ring_size];
		_handleCommand(frame);
		_ring_tail.store(tail + 1, std::memory_order_release);
		return;
	}
}

void Bootloader::_handleCommand(const CommandFrame &frame)
{
	switch (static_cast<Command>(frame.command)) {
	case Command::Ping:
		_handlePing(frame);
		break;

	default:
		sendResponse(static_cast<Command>(frame.command), frame.index, ResponseCode::UnknownCommand);
		break;
	}
}

void Bootloader::_handlePing(const CommandFrame &frame)
{
	sendResponse(Command::Ping, frame.index, ResponseCode::Ok);
}

} // namespace can_bootloader

// The three hooks are declared extern "C" in main.h and the HAL callback in the HAL CAN header, so these
// definitions keep C linkage.
void mainLoopJumpToAppIfRequested(void)
{
}

void mainLoopInit(void)
{
	can_bootloader::bootloader.init();
}

void mainLoopRun(void)
{
	can_bootloader::bootloader.run();
}

// HAL calls this from the CAN1 RX0 interrupt for every frame waiting in FIFO0.
void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef *can)
{
	can_bootloader::bootloader.onCanFrameReceived(can);
}

