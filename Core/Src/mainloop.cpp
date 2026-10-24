#include "mainloop.hpp"

#include <cstring>

namespace can_bootloader
{

namespace
{

// The one bootloader of this image.
Bootloader bootloader;

// True when every word in the range reads as erased flash.
bool isBlank(uint32_t address, uint32_t word_count)
{
	const volatile uint32_t *flash_words = reinterpret_cast<const volatile uint32_t *>(address);

	for (uint32_t word_number = 0; word_number < word_count; ++word_number) {
		if (flash_words[word_number] != 0xFFFFFFFFu) {
			return false;
		}
	}

	return true;
}

bool isInRange(uint32_t value, uint32_t first, uint32_t last)
{
	return value >= first && value <= last;
}

void clearFlashErrorFlags()
{
	__HAL_FLASH_CLEAR_FLAG(FLASH_FLAG_EOP | FLASH_FLAG_OPERR | FLASH_FLAG_WRPERR
			       | FLASH_FLAG_PGAERR | FLASH_FLAG_PGPERR | FLASH_FLAG_PGSERR);
}

// The data cache can hold flash lines read before a program or erase, so reset it before reading the result back.
void flushFlashCaches()
{
	__HAL_FLASH_DATA_CACHE_DISABLE();
	__HAL_FLASH_DATA_CACHE_RESET();
	__HAL_FLASH_DATA_CACHE_ENABLE();
}

// Erase the sector that starts at the address. False when the address is not a sector start or the sector is not blank.
bool eraseSectorStartingAt(uint32_t address)
{
	const flash_layout::FlashSector *sector = flash_layout::sectorStartingAt(address);

	if (sector == nullptr) {
		return false;
	}

	FLASH_EraseInitTypeDef erase_config{};
	erase_config.TypeErase = FLASH_TYPEERASE_SECTORS;
	erase_config.Sector = sector->number;
	erase_config.NbSectors = 1;
	erase_config.VoltageRange = FLASH_VOLTAGE_RANGE_3;

	uint32_t failed_sector_number = 0;

	// The erase stalls the CPU for up to 4 s, so the watchdog is kicked just before and just after it.
	HAL_IWDG_Refresh(&hiwdg);
	HAL_FLASH_Unlock();
	clearFlashErrorFlags();
	const HAL_StatusTypeDef erase_status = HAL_FLASHEx_Erase(&erase_config, &failed_sector_number);
	HAL_FLASH_Lock();
	HAL_IWDG_Refresh(&hiwdg);

	if (erase_status != HAL_OK) {
		return false;
	}

	flushFlashCaches();

	return isBlank(sector->start_address, sector->size_bytes / 4);
}

// Program words into erased flash and read them back. A page already holding the same words counts as programmed.
bool programWords(uint32_t address, const uint32_t *words, uint32_t word_count)
{
	const uint32_t *flash_words = reinterpret_cast<const uint32_t *>(address);
	const bool already_programmed = memcmp(flash_words, words, word_count * sizeof(uint32_t)) == 0;

	if (already_programmed) {
		return true;
	}

	HAL_FLASH_Unlock();
	clearFlashErrorFlags();

	for (uint32_t word_number = 0; word_number < word_count; ++word_number) {
		const uint32_t word_address = address + word_number * sizeof(uint32_t);
		const HAL_StatusTypeDef program_status = HAL_FLASH_Program(FLASH_TYPEPROGRAM_WORD, word_address,
							 words[word_number]);
		const bool word_programmed = program_status == HAL_OK;

		if (!word_programmed) {
			HAL_FLASH_Lock();
			return false;
		}
	}

	HAL_FLASH_Lock();
	HAL_IWDG_Refresh(&hiwdg);
	flushFlashCaches();

	return memcmp(flash_words, words, word_count * sizeof(uint32_t)) == 0;
}

// The hardware CRC of the words, fed one 32-bit word at a time.
uint32_t calculateWordsCrc(const uint32_t *words, uint32_t word_count)
{
	__HAL_CRC_DR_RESET(&hcrc);

	return HAL_CRC_Calculate(&hcrc, const_cast<uint32_t *>(words), word_count);
}

AppMetadata readMetadata()
{
	AppMetadata metadata;
	memcpy(&metadata, reinterpret_cast<const void *>(metadata_address), sizeof(metadata));

	return metadata;
}

// The app's first word is its stack pointer and the second its reset handler. A file linked for another address,
// for example 0x08000000, fails here even when its CRC is right.
bool appVectorTableIsValid()
{
	const uint32_t *vector_table = reinterpret_cast<const uint32_t *>(app_address);
	const uint32_t stack_pointer = vector_table[0];
	const uint32_t reset_vector = vector_table[1];

	const bool stack_in_ram = isInRange(stack_pointer, ram_begin_address, ram_end_address);
	const bool stack_in_ccm_ram = isInRange(stack_pointer, ccm_ram_begin_address, ccm_ram_end_address);

	// A Cortex-M reset vector has bit 0 set for Thumb code.
	const bool reset_is_thumb = (reset_vector & 1u) != 0;
	const uint32_t reset_address = reset_vector & ~1u;
	const bool reset_in_app = reset_address >= app_address && reset_address < app_end_address;

	return (stack_in_ram || stack_in_ccm_ram) && reset_is_thumb && reset_in_app;
}

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

void Bootloader::jumpToAppIfRequested()
{
	volatile uint32_t *token = reinterpret_cast<volatile uint32_t *>(jump_token_address);

	const bool jump_requested = *token == jump_token;

	if (!jump_requested) {
		return;
	}

	// Clear the token first, so a reset inside the app comes back to the bootloader.
	*token = 0;

	// The CRC was checked before the token was set, so this only guards against a token left by something else.
	if (!appVectorTableIsValid()) {
		return;
	}

	const uint32_t *app_vector_table = reinterpret_cast<const uint32_t *>(app_address);
	const uint32_t app_stack_pointer = app_vector_table[0];
	const uint32_t app_reset_vector = app_vector_table[1];

	// Hand the app a quiet core: no SysTick interrupt from the bootloader and the app's own vector table.
	SysTick->CTRL = 0;
	SCB->ICSR = SCB_ICSR_PENDSTCLR_Msk;
	SCB->VTOR = app_address;
	__DSB();
	__ISB();

	__set_MSP(app_stack_pointer);
	reinterpret_cast<void (*)()>(app_reset_vector)();
}

void Bootloader::init()
{
	_startup_window_ms = startup_window_ms;

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

	// Without CAN the host cannot be reached, so run() goes straight to starting the app.
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
		_startAppIfIntact();
	}

	const uint32_t tail = _ring_tail.load(std::memory_order_relaxed);
	const uint32_t head = _ring_head.load(std::memory_order_acquire);
	const bool frame_waiting = head != tail;

	if (frame_waiting) {
		const CommandFrame frame = _ring[tail % ring_size];
		_handleCommand(frame);

		// Timed after the command, because an erase can take seconds.
		_last_host_frame_ms = HAL_GetTick();
		_host_seen = true;
		_ring_tail.store(tail + 1, std::memory_order_release);
		return;
	}

	const uint32_t silence_limit_ms = _host_seen ? host_silence_timeout_ms : _startup_window_ms;
	const uint32_t silent_ms = HAL_GetTick() - _last_host_frame_ms;
	const bool host_still_active = silent_ms <= silence_limit_ms;

	if (host_still_active) {
		return;
	}

	_startAppIfIntact();
}

void Bootloader::_handleCommand(const CommandFrame &frame)
{
	switch (static_cast<Command>(frame.command)) {
	case Command::Ping:
		_handlePing(frame);
		break;

	case Command::WriteBuffer:
		_handleWriteBuffer(frame);
		break;

	case Command::WritePage:
		_handleWritePage(frame);
		break;

	case Command::WriteCrc:
		_handleWriteCrc(frame);
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

void Bootloader::_handleWriteBuffer(const CommandFrame &frame)
{
	const uint32_t word_index = frame.index;
	const bool word_in_range = word_index < page_size_words;

	if (!word_in_range) {
		sendResponse(Command::WriteBuffer, frame.index, ResponseCode::BadIndex);
		return;
	}

	// The first word of a new page starts a new page buffer.
	if (_page_committed) {
		memset(_word_present, 0, sizeof(_word_present));
		_present_word_count = 0;
		_page_committed = false;
	}

	// A word sent twice, for example after a lost reply, is counted once.
	if (!_word_present[word_index]) {
		_word_present[word_index] = true;
		_present_word_count++;
	}

	_page_buffer_words[word_index] = frame.value;
	sendResponse(Command::WriteBuffer, frame.index, ResponseCode::Ok);
}

void Bootloader::_handleWritePage(const CommandFrame &frame)
{
	const uint32_t page_index = frame.index;
	const uint32_t host_page_crc = frame.value;
	const bool page_in_range = page_index < max_page_count;

	if (!page_in_range) {
		sendResponse(Command::WritePage, frame.index, ResponseCode::BadIndex);
		return;
	}

	// Pages go in order: the next one, the last one again after a lost reply, or page 0 to start over.
	const bool is_restart = page_index == 0;
	const bool is_next_page = page_index == _next_page_index;
	const bool is_last_page_again = _next_page_index != 0 && page_index == _next_page_index - 1;
	const bool page_in_sequence = is_restart || is_next_page || is_last_page_again;

	if (!page_in_sequence) {
		sendResponse(Command::WritePage, frame.index, ResponseCode::BadSequence);
		return;
	}

	const bool page_complete = _present_word_count == page_size_words;

	if (!page_complete) {
		sendResponse(Command::WritePage, frame.index, ResponseCode::PageIncomplete);
		return;
	}

	const uint32_t buffer_crc = calculateWordsCrc(_page_buffer_words, page_size_words);
	const bool page_crc_matches = buffer_crc == host_page_crc;

	if (!page_crc_matches) {
		sendResponse(Command::WritePage, frame.index, ResponseCode::PageCrcMismatch);
		return;
	}

	const uint32_t page_address = app_address + page_index * page_size_bytes;

	// Page 0 erases the metadata sector first, so an upload that stops halfway leaves no valid metadata.
	const bool first_page = page_index == 0;

	if (first_page && !eraseSectorStartingAt(metadata_sector_address)) {
		sendResponse(Command::WritePage, frame.index, ResponseCode::EraseFailed);
		return;
	}

	// Pages arrive in ascending order, so a sector is erased when the page that opens it arrives.
	const bool starts_sector = flash_layout::isSectorStart(page_address);

	if (starts_sector && !eraseSectorStartingAt(page_address)) {
		sendResponse(Command::WritePage, frame.index, ResponseCode::EraseFailed);
		return;
	}

	const bool page_programmed = programWords(page_address, _page_buffer_words, page_size_words);

	if (!page_programmed) {
		sendResponse(Command::WritePage, frame.index, ResponseCode::ProgramFailed);
		return;
	}

	_next_page_index = page_index + 1;
	_page_committed = true;
	sendResponse(Command::WritePage, frame.index, ResponseCode::Ok);
}

void Bootloader::_handleWriteCrc(const CommandFrame &frame)
{
	const uint32_t page_count = frame.index;
	const uint32_t host_crc = frame.value;

	const bool page_count_in_range = page_count != 0 && page_count <= max_page_count;

	if (!page_count_in_range) {
		sendResponse(Command::WriteCrc, frame.index, ResponseCode::BadIndex);
		return;
	}

	const uint32_t flash_crc = calculateWordsCrc(reinterpret_cast<const uint32_t *>(app_address),
						      page_count * page_size_words);
	const bool crc_matches = flash_crc == host_crc;

	if (!crc_matches) {
		sendResponse(Command::WriteCrc, frame.index, ResponseCode::CrcMismatch);
		return;
	}

	// No metadata is stored for an image that cannot run, so the bootloader never starts it.
	if (!appVectorTableIsValid()) {
		sendResponse(Command::WriteCrc, frame.index, ResponseCode::BadImage);
		return;
	}

	const AppMetadata metadata = {page_count, host_crc};
	const bool metadata_programmed = programWords(metadata_address, reinterpret_cast<const uint32_t *>(&metadata),
					       sizeof(metadata) / sizeof(uint32_t));

	if (!metadata_programmed) {
		sendResponse(Command::WriteCrc, frame.index, ResponseCode::ProgramFailed);
		return;
	}

	sendResponse(Command::WriteCrc, frame.index, ResponseCode::Ok);
}

ResponseCode Bootloader::_checkStoredApp() const
{
	const AppMetadata metadata = readMetadata();

	// Blank flash reads 0xFFFFFFFF, which is above max_page_count, so a blank metadata sector is rejected here.
	const bool metadata_valid = metadata.page_count != 0 && metadata.page_count <= max_page_count;

	if (!metadata_valid) {
		return ResponseCode::BadImage;
	}

	const uint32_t flash_crc = calculateWordsCrc(reinterpret_cast<const uint32_t *>(app_address),
						      metadata.page_count * page_size_words);
	const bool crc_matches = flash_crc == metadata.app_crc;

	if (!crc_matches) {
		return ResponseCode::CrcMismatch;
	}

	if (!appVectorTableIsValid()) {
		return ResponseCode::BadImage;
	}

	return ResponseCode::Ok;
}

void Bootloader::_resetIntoApp()
{
	*reinterpret_cast<volatile uint32_t *>(jump_token_address) = jump_token;

	// With the token set the reset ends in the app.
	NVIC_SystemReset();
}

void Bootloader::_startAppIfIntact()
{
	const bool app_intact = _checkStoredApp() == ResponseCode::Ok;

	if (app_intact) {
		_resetIntoApp();
	}

	// Without a good app the reset ends in the bootloader again, which waits for the host once more.
	NVIC_SystemReset();
}

} // namespace can_bootloader

// The three hooks are declared extern "C" in main.h and the HAL callback in the HAL CAN header, so these
// definitions keep C linkage.
void mainLoopJumpToAppIfRequested(void)
{
	can_bootloader::bootloader.jumpToAppIfRequested();
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

