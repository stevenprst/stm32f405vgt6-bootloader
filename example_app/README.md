# Making an application flashable

`bootloader_trigger.c` is the code part. This file is the project settings part. An app that has all of it can be
flashed with `tools/kvaser_flasher.py` and can be reflashed without touching the board.

## 1. Linker script

```ld
FLASH    (rx)    : ORIGIN = 0x08008000,   LENGTH = 992K
```

Not `0x08000000` and not `1008K`. The bootloader owns the first 32 KB, and the app area ends at `0x08100000`.

## 2. Vector table

In `system_stm32f4xx.c` (a CubeMX file, so it is inside USER CODE only for the defines at the top):

```c
#define USER_VECT_TAB_ADDRESS
#define VECT_TAB_OFFSET  0x00008000U
```

or `SCB->VTOR = 0x08008000;` first thing in `main()`. The bootloader sets `VTOR` before it jumps, but `SystemInit()` of
an app that does not set it leaves it as it was, so set it in the app too for the day the app runs without the
bootloader in front.

## 3. A `.bin`

Project Properties, C/C++ Build, Settings, MCU Post build outputs, Convert to binary file.

## 4. CAN

Same bit rate as the bootloader (125 kbit/s). Call `bootloaderTriggerInit(&hcan1)` after `HAL_CAN_Start()`, and keep the
app's own frames: the filter in the example passes only `0x6A4`, so add the app's IDs to another filter bank or widen
it. If the app uses the receive callback itself, call the trigger code from there for ID `0x6A4`.

## 5. Check

1. Build, flash with `python tools/kvaser_flasher.py 1 app.bin`.
2. The flasher prints the pages and `App started`.
3. Run it again: it sends the trigger first, so the running app resets into the bootloader and is flashed again with
   no button and no power cycle.
4. `python tools/kvaser_flasher.py 1 app.bin --info` shows `last reset : software` after a trigger.

## What happens if the app does not have the trigger

The flasher cannot reach the bootloader while the app runs. Power cycle or press reset, then run the flasher within
250 ms of the reset, or use `--info` repeatedly until it connects. The pings the flasher sends during connection are
what catches that window.
