#ifndef MAINLOOP_HPP
#define MAINLOOP_HPP

#include <atomic>
#include <cstdint>

#include "main.h"

// The three hooks that main.c calls are declared in main.h, inside USER CODE EFP:
//   void mainLoopJumpToAppIfRequested(void);
//   void mainLoopInit(void);
//   void mainLoopRun(void);

// Handles that CubeMX creates in main.c.
extern "C" {
extern CAN_HandleTypeDef hcan1;
extern CRC_HandleTypeDef hcrc;
extern IWDG_HandleTypeDef hiwdg;
}

// Build options, set with -D in Properties, C/C++ Build, Settings, MCU G++ Compiler, Preprocessor.
//   BOOTLOADER_NODE_ID=n                 which board this image answers for, 1 when not set
#ifndef BOOTLOADER_NODE_ID
#define BOOTLOADER_NODE_ID 1
#endif

namespace can_bootloader
{

// CAN IDs, all standard 11-bit.
static constexpr uint16_t command_can_id = 0x6A5;	// host to bootloader
static constexpr uint16_t response_can_id = 0x6A6;	// bootloader to host
static constexpr uint16_t app_trigger_can_id = 0x6A4;	// host to the running app, handled by the app, not here

// The app trigger frame is: node id, then app_trigger_magic as two little-endian bytes.
static constexpr uint16_t app_trigger_magic = 0xB007;

// Which board this image answers for. One image per board id.
static constexpr uint8_t this_node_id = BOOTLOADER_NODE_ID;

// Flash layout. The bootloader owns sector 0, sector 1 holds the app metadata, the app starts at sector 2.
static constexpr uint32_t metadata_sector_address = 0x08004000;
static constexpr uint32_t app_address = 0x08008000;
static constexpr uint32_t app_end_address = 0x08100000;

// The metadata sits one page before the app, inside sector 1.
static constexpr uint32_t page_size_words = 256;
static constexpr uint32_t page_size_bytes = page_size_words * 4;
static constexpr uint32_t metadata_address = app_address - page_size_bytes;

// The most pages the app area holds, 992 pages of 1 KB.
static constexpr uint32_t max_page_count = (app_end_address - app_address) / page_size_bytes;

// RAM an app can put its stack in. An app whose first word is outside both is not started.
static constexpr uint32_t ram_begin_address = 0x20000000;
static constexpr uint32_t ram_end_address = 0x20020000;
static constexpr uint32_t ccm_ram_begin_address = 0x10000000;
static constexpr uint32_t ccm_ram_end_address = 0x10010000;

// How long to wait for a reply frame to leave the CAN mailbox.
static constexpr uint32_t reply_timeout_ms = 100;

// Frames the interrupt can queue for the main loop. A power of two, and the host sends at most 4 at once.
static constexpr uint32_t ring_size = 8;
static_assert((ring_size & (ring_size - 1)) == 0, "the ring index is taken with a modulo of a power of two");

enum class Command : uint8_t {
	WriteBuffer = 1,	// store one word of the page buffer
	WritePage = 2,		// program the page buffer into flash
	WriteCrc = 3,		// check the whole image and store its metadata
	Ping = 4,		// answer, nothing else
};

enum class ResponseCode : uint8_t {
	Ok = 0,
	BadIndex = 1,		// word, page, page count or info item out of range
	CrcMismatch = 2,	// the image in flash does not match the host CRC
	ProgramFailed = 3,	// flash programming or its read back failed
	EraseFailed = 4,	// a sector did not erase
	PageIncomplete = 5,	// WritePage came before all 256 words of the page
	PageCrcMismatch = 6,	// the page buffer does not match the CRC the host sent with WritePage
	BadSequence = 7,	// a page other than the next one, the last one again, or page 0
	BadImage = 8,		// no stored app, or its vector table is not a valid one for this address
	UnknownCommand = 9,
};

// The 8 bytes the host sends, little-endian.
struct __attribute__((packed)) CommandFrame {
	uint8_t target_node_id;
	uint8_t command;
	uint16_t index;		// word number for WriteBuffer, page number for WritePage, page count for WriteCrc
	uint32_t value;		// the word for WriteBuffer, the page CRC for WritePage, the image CRC for WriteCrc
};

static_assert(sizeof(CommandFrame) == 8, "a command is exactly one 8-byte CAN frame");

// The 8 bytes the bootloader answers with. It repeats the index of the command so the host can match a late reply.
struct __attribute__((packed)) ResponseFrame {
	uint8_t node_id;
	uint8_t command;
	uint8_t code;
	uint16_t index;
	uint8_t payload[3];	// zero except for GetInfo
};

static_assert(sizeof(ResponseFrame) == 8, "a reply is exactly one 8-byte CAN frame");

// Written once before the app, 0xFFFFFFFF in both words while no valid app is stored.
struct AppMetadata {
	uint32_t page_count;
	uint32_t app_crc;
};

static_assert(sizeof(AppMetadata) == 8, "the metadata is two flash words");
static_assert(max_page_count == 992, "the app area is 992 KB");
static_assert(metadata_address >= metadata_sector_address, "the metadata is inside its sector");

// The 12 flash sectors of the STM32F405VG, in address order.
namespace flash_layout
{

struct FlashSector {
	uint32_t start_address;
	uint32_t size_bytes;
	uint32_t number;
};

inline constexpr FlashSector flash_sectors[] = {
	{0x08000000, 16 * 1024, FLASH_SECTOR_0},
	{0x08004000, 16 * 1024, FLASH_SECTOR_1},
	{0x08008000, 16 * 1024, FLASH_SECTOR_2},
	{0x0800C000, 16 * 1024, FLASH_SECTOR_3},
	{0x08010000, 64 * 1024, FLASH_SECTOR_4},
	{0x08020000, 128 * 1024, FLASH_SECTOR_5},
	{0x08040000, 128 * 1024, FLASH_SECTOR_6},
	{0x08060000, 128 * 1024, FLASH_SECTOR_7},
	{0x08080000, 128 * 1024, FLASH_SECTOR_8},
	{0x080A0000, 128 * 1024, FLASH_SECTOR_9},
	{0x080C0000, 128 * 1024, FLASH_SECTOR_10},
	{0x080E0000, 128 * 1024, FLASH_SECTOR_11},
};

inline constexpr uint32_t flash_sector_count = sizeof(flash_sectors) / sizeof(flash_sectors[0]);

// The sector that starts exactly at the address, or nullptr when the address is not a sector start.
constexpr const FlashSector *sectorStartingAt(uint32_t address)
{
	for (const FlashSector &sector : flash_sectors) {
		if (sector.start_address == address) {
			return &sector;
		}
	}

	return nullptr;
}

// True when a sector starts exactly at the address.
constexpr bool isSectorStart(uint32_t address)
{
	return sectorStartingAt(address) != nullptr;
}

// The number of the sector that starts at the address, 0xFFFFFFFF when none does.
constexpr uint32_t sectorNumberStartingAt(uint32_t address)
{
	for (const FlashSector &sector : flash_sectors) {
		if (sector.start_address == address) {
			return sector.number;
		}
	}

	return 0xFFFFFFFFu;
}

// These checks run at build time, so a wrong table or a wrong layout constant stops the build.
static_assert(flash_sector_count == 12, "the F405VG has 12 flash sectors");
static_assert(flash_sectors[0].start_address == 0x08000000, "flash starts at 0x08000000");
static_assert(flash_sectors[flash_sector_count - 1].start_address + flash_sectors[flash_sector_count - 1].size_bytes
	      == app_end_address, "the last sector ends where the app area ends");
static_assert(sectorNumberStartingAt(metadata_sector_address) == FLASH_SECTOR_1, "the metadata sector is sector 1");
static_assert(sectorNumberStartingAt(app_address) == FLASH_SECTOR_2, "page 0 opens sector 2");
static_assert(!isSectorStart(app_address + 1 * page_size_bytes), "page 1 is inside sector 2");
static_assert(!isSectorStart(app_address + 15 * page_size_bytes), "page 15 is the last page of sector 2");
static_assert(sectorNumberStartingAt(app_address + 16 * page_size_bytes) == FLASH_SECTOR_3, "page 16 opens sector 3");
static_assert(!isSectorStart(app_address + 31 * page_size_bytes), "page 31 is the last page of sector 3");
static_assert(sectorNumberStartingAt(app_address + 32 * page_size_bytes) == FLASH_SECTOR_4, "page 32 opens sector 4");
static_assert(!isSectorStart(app_address + 95 * page_size_bytes), "page 95 is the last page of sector 4");
static_assert(sectorNumberStartingAt(app_address + 96 * page_size_bytes) == FLASH_SECTOR_5, "page 96 opens sector 5");
static_assert(sectorNumberStartingAt(app_end_address - 128 * 1024) == FLASH_SECTOR_11, "sector 11 is the last one");

} // namespace flash_layout

class Bootloader
{
public:
	// Set up the CAN filter, start CAN and enable the receive interrupt.
	void init();

	// One pass of the main loop: kick the watchdog and handle one command.
	void run();

	// Called from the CAN receive interrupt for each frame in FIFO0.
	void onCanFrameReceived(CAN_HandleTypeDef *can);

private:
	void _handleCommand(const CommandFrame &frame);
	void _handlePing(const CommandFrame &frame);
	void _handleWriteBuffer(const CommandFrame &frame);
	void _handleWritePage(const CommandFrame &frame);
	void _handleWriteCrc(const CommandFrame &frame);

	// Frames from the interrupt to the main loop. The interrupt only writes _ring_head, the main loop only _ring_tail.
	// Both count up forever and are taken modulo ring_size to find the slot.
	std::atomic<uint32_t> _ring_head{0};
	std::atomic<uint32_t> _ring_tail{0};
	CommandFrame _ring[ring_size]{};

	// The page the host is filling with WriteBuffer, and which of its words have arrived.
	uint32_t _page_buffer_words[page_size_words]{};
	bool _word_present[page_size_words]{};
	uint32_t _present_word_count{0};

	// True from a good WritePage until the first WriteBuffer of the next page, so the last page can be asked for again.
	bool _page_committed{false};

	// The page the host has to send next, 0 before the first page.
	uint32_t _next_page_index{0};

	// Set in init(): whether CAN started.
	bool _can_ready{false};
};

} // namespace can_bootloader

#endif // MAINLOOP_HPP
