# STM32F405VGT6 CAN Bus Bootloader

A bootloader that loads an application over CAN, written in C++ in `Core/Src/mainloop.cpp`. `main.c` is the CubeMX
file and only calls three hooks.

Board: STM32F405VGT6. The F407VGT6 bootloader speaks an older version of the same protocol (see "Differences from the
F407 bootloader").

## Memory map

| Address | Size | Content |
|---|---|---|
| `0x08000000` | 16 KB, sector 0 | Bootloader. The linker stops the build above 16 KB. |
| `0x08004000` | 16 KB, sector 1 | App metadata at `0x08007C00`: page count and CRC. |
| `0x08008000` | up to 992 KB | Application. |
| `0x20003000` | 4 bytes SRAM | Jump token `0x53544D33`: the bootloader checked the app and reset to start it. |
| `0x20003004` | 4 bytes SRAM | Stay token `0x53544159`: the app asks the bootloader to wait for the host. |

The linker script fails the build if the bootloader's RAM grows to the tokens.

## How it works

1. After a reset the bootloader waits 250 ms for a command frame for its node id. After a stay token (left by the app)
   or a watchdog reset it waits 30 s.
2. While the host is sending, every command is answered. After 2.5 s without a command, or on `StartApp`, the
   bootloader checks the stored app: metadata, CRC over the whole image, and a valid vector table. If all are good it
   sets the jump token and resets, and the next start jumps to the app.
3. A blank or damaged app keeps the bootloader running.
4. The CAN receive interrupt only queues the frame (8 slots). Erase, programming and replies run in the main loop.
5. If CAN cannot be started the bootloader skips the wait and starts the app.

### CAN frames

All IDs are standard 11-bit. Bit rate 125 kbit/s, 16 time quanta per bit, sample point 75%.

| ID | Direction | Content |
|---|---|---|
| `0x6A5` | host to bootloader | command |
| `0x6A6` | bootloader to host | reply |
| `0x6A4` | host to the running app | trigger: `node id, 0x07, 0xB0` (magic `0xB007`) |

Command, 8 bytes, little-endian: `node (u8) | command (u8) | index (u16) | value (u32)`.
Reply, 8 bytes: `node (u8) | command (u8) | code (u8) | index (u16, copy of the command's) | payload (3 bytes)`.
The host matches the reply on node, command and index, so a late reply is never taken for the next one.

| Command | Name | index | value | Notes |
|---|---|---|---|---|
| 1 | WriteBuffer | word 0..255 | the word | Stores one word of the 1 KB page buffer. Up to 4 can be in flight. |
| 2 | WritePage | page 0..991 | CRC of the 256 words | Needs all 256 words and a matching CRC. A page that opens a sector erases it first. Pages go in order from 0: the next one, the last one again, or 0 to start over. |
| 3 | WriteCrc | page count | image CRC | Checks the image in flash and its vector table, then stores the metadata. Repeating it with the same values verifies the flash. |
| 4 | Ping | | | Answers `ok`. |
| 5 | GetInfo | item | | Item 0: version (u8), largest app in pages (u16). Item 1: stored page count (u16), reset flags (u8). Item 2: CAN error bits (u24). |
| 6 | StartApp | | | Checks the stored app, answers, then resets into it. |

| Code | Meaning |
|---|---|
| 0 | ok |
| 1 | index out of range |
| 2 | CRC mismatch |
| 3 | programming failed |
| 4 | erase failed |
| 5 | page incomplete |
| 6 | page CRC mismatch |
| 7 | bad sequence |
| 8 | bad image (no stored app, or the vector table is not valid for `0x08008000`) |
| 9 | unknown command |

Reset flags: bit 0 power-on, 1 software, 2 watchdog, 3 brown-out, 4 pin.

The CRC is the STM32 hardware CRC: polynomial `0x04C11DB7`, start `0xFFFFFFFF`, no reflection, fed 32-bit words.
`tools/kvaser_flasher.py` computes the same value and `tools/test_crc.py` checks it.

## What the application needs

See `example_app/README.md` and `example_app/bootloader_trigger.c`. In short: link at `0x08008000` with 992K, set the
vector table offset, build a `.bin`, and answer the trigger frame by writing the stay token and resetting.

## Flashing

```bash
pip install -r tools/requirements.txt
python tools/kvaser_flasher.py 1 app.bin
python tools/kvaser_flasher.py 1 --info                 # version, stored app, last reset (no file needed)
python tools/kvaser_flasher.py 1 app.bin --verify-only  # compare flash with the file
for n in 1 2 3; do python tools/kvaser_flasher.py $n app.bin; done      # several boards (Linux, macOS, Git Bash)
python tools/kvaser_flasher.py 1 app.bin --bitrate 125000 --channel 0 --interface kvaser
python tools/kvaser_flasher.py 1 app.bin --interface socketcan --channel can0     # Linux
```

Expected output:

```
Connecting to board 1...
Uploading 10 pages
Page 1/10................ done
...
Verifying CRC...
Verified and flashed successfully
App started
```

The script refuses a file whose first two words are not a RAM stack pointer and a reset handler inside the app area,
which is what an app linked for `0x08000000` looks like, and a file over 992 pages. It does this before the bus is
opened. A reply that does not come is asked for again up to 3 times. `--channel` is a number for a Kvaser (`0`) and a
name for socketcan (`can0`).

## Build options

Set with `-D` in Properties, C/C++ Build, Settings, MCU G++ Compiler, Preprocessor.

| Option | Default | Effect |
|---|---|---|
| `BOOTLOADER_NODE_ID=n` | 1 | The board id this image answers for. One image per board. |
| `BOOTLOADER_PROTECT_SECTOR0=1` | off | Write protects the bootloader sector in the option bytes. Production only: afterwards the debugger cannot rewrite the bootloader until the protection is removed in STM32CubeProgrammer. |
| `BOOTLOADER_FREEZE_IWDG_IN_DEBUG=1` | off | Stops the watchdog while the debugger halts the core. Do not use when testing the watchdog behaviour. |

The brown-out reset is set to level 3 (about 2.7 V) in the option bytes at the first start. Writing the option bytes
resets the MCU once.

## Tests without hardware

```bash
cd tools
python test_crc.py        # the CRC against fixed values
python test_flasher.py    # the flasher against a software model of the bootloader on a virtual CAN bus
```

`tools/board_simulator.py` follows the rules of `mainloop.cpp` and imports nothing from the flasher: its protocol
numbers and its bit-by-bit CRC are its own, so a mistake in the flasher fails a test. When the protocol changes,
change the C++, the flasher and the simulator.

## Source

| File | Content |
|---|---|
| `Core/Inc/mainloop.hpp` | Constants, frames, the flash sector table with build time checks, the `Bootloader` class. |
| `Core/Src/mainloop.cpp` | Flash helpers, the command handlers, the jump, the main loop pass. |
| `Core/Src/main.c`, `Core/Inc/main.h` | CubeMX output with three hooks in USER CODE blocks. |
| `f405vgt6-can-bootloader.ioc` | CubeMX configuration: CAN1 on PB8/PB9, CRC, IWDG. |
| `STM32F405VGTX_FLASH.ld` | Stock script with `FLASH LENGTH = 16K` and the RAM check. |
| `f405vgt6-can-bootloader Debug.launch` | ST-LINK debug configuration. |
| `tools/kvaser_flasher.py` | The PC side. |
| `tools/board_simulator.py`, `tools/test_flasher.py`, `tools/test_crc.py` | Tests. |
| `example_app/` | What an app needs. |

## Differences from the F407 bootloader

The F407 bootloader and its flasher do not work with this one without changes: the reply is 8 bytes with the index, the
page CRC is required, the trigger frame is `0x6A4` instead of `0x6A5`, and the bit timing differs. Flash both sides from
the same version.
