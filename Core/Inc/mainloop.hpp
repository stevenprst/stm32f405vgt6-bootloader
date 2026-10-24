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

// How long to wait for a reply frame to leave the CAN mailbox.
static constexpr uint32_t reply_timeout_ms = 100;

// Frames the interrupt can queue for the main loop. A power of two, and the host sends at most 4 at once.
static constexpr uint32_t ring_size = 8;
static_assert((ring_size & (ring_size - 1)) == 0, "the ring index is taken with a modulo of a power of two");

enum class Command : uint8_t {
	Ping = 4,		// answer, nothing else
};

enum class ResponseCode : uint8_t {
	Ok = 0,
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

	// Frames from the interrupt to the main loop. The interrupt only writes _ring_head, the main loop only _ring_tail.
	// Both count up forever and are taken modulo ring_size to find the slot.
	std::atomic<uint32_t> _ring_head{0};
	std::atomic<uint32_t> _ring_tail{0};
	CommandFrame _ring[ring_size]{};

	// Set in init(): whether CAN started.
	bool _can_ready{false};
};

} // namespace can_bootloader

#endif // MAINLOOP_HPP
