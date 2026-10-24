"""Flash an application .bin into the STM32F405 CAN bootloader.

    python kvaser_flasher.py <node_id> <firmware.bin> [options]

    --bitrate 125000   CAN bit rate in bit/s
    --channel 0        CAN channel, 0 for the first Kvaser
    --interface kvaser python-can interface, for example kvaser, socketcan, pcan or slcan
    --verify-only      check the app in flash against the file, write nothing
    --info             print what the bootloader reports and exit (no file needed)

Several boards: for n in 1 2 3; do python kvaser_flasher.py $n app.bin; done

The app must be linked at 0x08008000 and, when it receives the trigger frame (ID 0x6A4), leave the stay token and reset.
"""

import argparse
import collections
import os
import struct
import sys
import time

import can

COMMAND_CAN_ID = 0x6A5   # host to bootloader
RESPONSE_CAN_ID = 0x6A6  # bootloader to host
TRIGGER_CAN_ID = 0x6A4   # host to the running app
TRIGGER_MAGIC = 0xB007

COMMAND_WRITE_BUFFER = 1
COMMAND_WRITE_PAGE = 2
COMMAND_WRITE_CRC = 3
COMMAND_PING = 4
COMMAND_GET_INFO = 5
COMMAND_START_APP = 6

INFO_ITEM_VERSION = 0
INFO_ITEM_STORED_APP = 1
INFO_ITEM_CAN_ERROR = 2

PAGE_SIZE_BYTES = 1024
PAGE_SIZE_WORDS = PAGE_SIZE_BYTES // 4
MAX_PAGE_COUNT = 992  # the app area is 992 KB

APP_ADDRESS = 0x08008000
APP_END_ADDRESS = 0x08100000
RAM_BEGIN_ADDRESS = 0x20000000
RAM_END_ADDRESS = 0x20020000
CCM_RAM_BEGIN_ADDRESS = 0x10000000
CCM_RAM_END_ADDRESS = 0x10010000

REPLY_TIMEOUT_S = 1.0
WRITE_PAGE_TIMEOUT_S = 20.0  # a page that opens a 128 KB sector waits for the erase, up to 4 s
START_APP_TIMEOUT_S = 5.0    # the check reads the whole image
WRITE_CRC_TIMEOUT_S = 5.0
PING_TRIES = 10
PING_TIMEOUT_S = 0.1
SEND_TIMEOUT_S = 2.0
RETRIES = 3  # extra tries when a reply does not come
WINDOW = 4   # WriteBuffer frames in flight; the bootloader queues 8 and a retry can double the frames in flight

RESPONSE_NAMES = {
    0: "ok",
    1: "bad index (word, page, page count or info item out of range)",
    2: "CRC mismatch (the image in flash differs from the file)",
    3: "flash programming failed",
    4: "sector erase failed",
    5: "page incomplete (not all 256 words arrived)",
    6: "page CRC mismatch (a word was damaged on the way)",
    7: "bad sequence (pages must go in order from page 0)",
    8: "bad image (no stored app, or its vector table is not valid for 0x08008000)",
    9: "unknown command (the bootloader is older than this script)",
}

RESET_FLAG_NAMES = [
    (1 << 0, "power-on"),
    (1 << 1, "software"),
    (1 << 2, "watchdog"),
    (1 << 3, "brown-out"),
    (1 << 4, "pin"),
]

CRC_POLYNOMIAL = 0x04C11DB7
CRC_INITIAL_VALUE = 0xFFFFFFFF

Reply = collections.namedtuple("Reply", "node_id command code index payload")


class FlasherError(Exception):
    pass


def build_crc_table():
    table = []

    for top_byte in range(256):
        crc = top_byte << 24

        for _ in range(8):
            if crc & 0x80000000:
                crc = (crc << 1) ^ CRC_POLYNOMIAL
            else:
                crc = crc << 1

            crc &= 0xFFFFFFFF

        table.append(crc)

    return table


CRC_TABLE = build_crc_table()


def stm32_crc(data):
    """CRC of the STM32 hardware CRC unit fed with 32-bit little-endian words.

    The unit shifts each word in from its top bit, so the bytes of a little-endian word go in reversed.
    """
    if len(data) % 4 != 0:
        raise ValueError("the CRC is calculated over whole 32-bit words")

    crc = CRC_INITIAL_VALUE

    for word_start in range(0, len(data), 4):
        word_bytes_top_first = data[word_start:word_start + 4][::-1]

        for byte in word_bytes_top_first:
            crc = ((crc << 8) & 0xFFFFFFFF) ^ CRC_TABLE[(crc >> 24) ^ byte]

    return crc


def check_firmware(firmware):
    """Refuse an image the bootloader would refuse, before anything is sent.

    The first two words are the stack pointer and the reset handler. A file linked for 0x08000000 has a good CRC
    and still cannot run at 0x08008000.
    """
    page_count = len(firmware) // PAGE_SIZE_BYTES

    if page_count == 0 or page_count > MAX_PAGE_COUNT:
        raise FlasherError("the image is %d pages, the bootloader takes 1 to %d" % (page_count, MAX_PAGE_COUNT))

    stack_pointer, reset_vector = struct.unpack("<II", firmware[:8])

    stack_in_ram = RAM_BEGIN_ADDRESS <= stack_pointer <= RAM_END_ADDRESS
    stack_in_ccm_ram = CCM_RAM_BEGIN_ADDRESS <= stack_pointer <= CCM_RAM_END_ADDRESS
    if not (stack_in_ram or stack_in_ccm_ram):
        raise FlasherError("the first word 0x%08X is not a RAM address, this is not an app image" % stack_pointer)

    reset_is_thumb = (reset_vector & 1) == 1
    reset_address = reset_vector & ~1
    reset_in_app = APP_ADDRESS <= reset_address < APP_END_ADDRESS
    if not (reset_is_thumb and reset_in_app):
        raise FlasherError(
            "the reset handler 0x%08X is not inside 0x%08X to 0x%08X, was the app linked at 0x08000000?"
            % (reset_vector, APP_ADDRESS, APP_END_ADDRESS))


def read_firmware(path):
    if not path.endswith(".bin"):
        raise FlasherError('the firmware file must end with ".bin"')

    if not os.path.isfile(path):
        raise FlasherError("file not found: %s" % path)

    with open(path, "rb") as firmware_file:
        data = bytearray(firmware_file.read())

    if len(data) < 8:
        raise FlasherError("the file is too short to be an app image")

    padding_bytes = (-len(data)) % PAGE_SIZE_BYTES
    data.extend(bytes(padding_bytes))

    firmware = bytes(data)
    check_firmware(firmware)

    return firmware


def send_command(bus, node_id, command, index, value):
    """Send the 8 byte command: node, command, index (u16), value (u32), little-endian."""
    data = struct.pack("<BBHI", node_id, command, index, value)
    message = can.Message(arbitration_id=COMMAND_CAN_ID, is_extended_id=False, data=data)
    bus.send(message, SEND_TIMEOUT_S)


def send_trigger(bus, node_id):
    """Ask a running app to leave the stay token and reset into the bootloader. A bootloader ignores it."""
    data = struct.pack("<BH", node_id, TRIGGER_MAGIC)
    message = can.Message(arbitration_id=TRIGGER_CAN_ID, is_extended_id=False, data=data)
    bus.send(message, SEND_TIMEOUT_S)


def parse_reply(message):
    """The 8 byte reply: node, command, code, index (u16), 3 payload bytes. None for a frame that is not a reply."""
    if message.arbitration_id != RESPONSE_CAN_ID or message.dlc < 5:
        return None

    node_id, command, code, index = struct.unpack("<BBBH", bytes(message.data[:5]))
    payload = bytes(message.data[5:8]).ljust(3, b"\0")

    return Reply(node_id, command, code, index, payload)


def wait_for_reply(bus, node_id, command, timeout_s, index=None):
    """The next reply of this node to this command, with this index when one is given. None when nothing came."""
    deadline_s = time.monotonic() + timeout_s

    while True:
        remaining_s = deadline_s - time.monotonic()

        if remaining_s <= 0:
            return None

        message = bus.recv(remaining_s)

        if message is None:
            return None

        reply = parse_reply(message)
        if reply is None:
            continue

        this_node = reply.node_id == node_id
        this_command = reply.command == command
        this_index = index is None or reply.index == index
        if this_node and this_command and this_index:
            return reply


def check_code(reply, node_id, command):
    if reply.code != 0:
        name = RESPONSE_NAMES.get(reply.code, "unknown")
        raise FlasherError("board %d answered command %d with error %d: %s" % (node_id, command, reply.code, name))


def run_command(bus, node_id, command, index, value, timeout_s=None):
    """Send a command and return its reply. A lost reply is asked for again, an error code is not."""
    if timeout_s is None:
        timeout_s = REPLY_TIMEOUT_S

    for _ in range(RETRIES + 1):
        send_command(bus, node_id, command, index, value)
        reply = wait_for_reply(bus, node_id, command, timeout_s, index)

        if reply is not None:
            check_code(reply, node_id, command)
            return reply

    raise FlasherError("no response from board %d to command %d" % (node_id, command))


def connect(bus, node_id):
    """Trigger a running app, then ping until the bootloader answers."""
    send_trigger(bus, node_id)

    for _ in range(PING_TRIES):
        send_command(bus, node_id, COMMAND_PING, 0, 0)

        if wait_for_reply(bus, node_id, COMMAND_PING, PING_TIMEOUT_S, 0) is not None:
            return True

    return False


def write_buffer(bus, node_id, page):
    """Send the 256 words of a page, WINDOW at a time, and wait until each one is acknowledged."""
    words = struct.unpack("<%dI" % PAGE_SIZE_WORDS, page)
    outstanding = set()
    next_word = 0
    acknowledged_words = 0
    timeouts = 0

    while next_word < PAGE_SIZE_WORDS or outstanding:
        while next_word < PAGE_SIZE_WORDS and len(outstanding) < WINDOW:
            send_command(bus, node_id, COMMAND_WRITE_BUFFER, next_word, words[next_word])
            outstanding.add(next_word)
            next_word += 1

        reply = wait_for_reply(bus, node_id, COMMAND_WRITE_BUFFER, REPLY_TIMEOUT_S)

        if reply is None:
            timeouts += 1

            if timeouts > RETRIES:
                raise FlasherError("no response from board %d to WriteBuffer" % node_id)

            # A word sent twice is harmless, so ask again for every word still without a reply.
            for word_index in sorted(outstanding):
                send_command(bus, node_id, COMMAND_WRITE_BUFFER, word_index, words[word_index])

            continue

        if reply.index not in outstanding:
            continue

        check_code(reply, node_id, COMMAND_WRITE_BUFFER)
        outstanding.remove(reply.index)
        acknowledged_words += 1

        # Only replies that fail one after the other count against the retries.
        timeouts = 0

        # One dot per 16 acknowledged words, so a page shows as 16 dots.
        if acknowledged_words % 16 == 0:
            print(".", end="", flush=True)


def upload_page(bus, node_id, page_index, page):
    write_buffer(bus, node_id, page)
    run_command(bus, node_id, COMMAND_WRITE_PAGE, page_index, stm32_crc(page), WRITE_PAGE_TIMEOUT_S)


def flash(bus, node_id, firmware):
    """Upload the image, check it, and start the app. The board must be in the bootloader (see connect)."""
    page_count = len(firmware) // PAGE_SIZE_BYTES
    print("Uploading %d pages" % page_count)

    for page_index in range(page_count):
        print("Page %d/%d..." % (page_index + 1, page_count), end="", flush=True)

        page = firmware[page_index * PAGE_SIZE_BYTES:(page_index + 1) * PAGE_SIZE_BYTES]
        upload_page(bus, node_id, page_index, page)

        print(" done")

    print("Verifying CRC...")
    run_command(bus, node_id, COMMAND_WRITE_CRC, page_count, stm32_crc(firmware), WRITE_CRC_TIMEOUT_S)
    print("Verified and flashed successfully")

    run_command(bus, node_id, COMMAND_START_APP, 0, 0, START_APP_TIMEOUT_S)
    print("App started")


def verify_only(bus, node_id, firmware):
    """Ask the bootloader to compare the app in flash with the file. The same words are programmed over themselves."""
    page_count = len(firmware) // PAGE_SIZE_BYTES

    run_command(bus, node_id, COMMAND_WRITE_CRC, page_count, stm32_crc(firmware), WRITE_CRC_TIMEOUT_S)
    print("The app in flash matches the file")


def print_info(bus, node_id):
    version_reply = run_command(bus, node_id, COMMAND_GET_INFO, INFO_ITEM_VERSION, 0)
    bootloader_version = version_reply.payload[0]
    max_pages = version_reply.payload[1] | (version_reply.payload[2] << 8)

    app_reply = run_command(bus, node_id, COMMAND_GET_INFO, INFO_ITEM_STORED_APP, 0)
    stored_pages = app_reply.payload[0] | (app_reply.payload[1] << 8)
    reset_flags = app_reply.payload[2]

    error_reply = run_command(bus, node_id, COMMAND_GET_INFO, INFO_ITEM_CAN_ERROR, 0)
    can_error = error_reply.payload[0] | (error_reply.payload[1] << 8) | (error_reply.payload[2] << 16)

    reset_names = [name for bit, name in RESET_FLAG_NAMES if reset_flags & bit]
    stored_text = "none" if stored_pages == 0 or stored_pages > max_pages else "%d pages" % stored_pages

    print("  bootloader version : %d" % bootloader_version)
    print("  largest app        : %d pages" % max_pages)
    print("  stored app         : %s" % stored_text)
    print("  last reset         : %s" % (", ".join(reset_names) or "unknown"))
    print("  CAN error bits     : 0x%06X" % can_error)


def parse_channel(text):
    """python-can wants a number for a Kvaser channel and a name for socketcan."""
    return int(text) if text.isdigit() else text


def parse_arguments(argv=None):
    parser = argparse.ArgumentParser(description="Flash an app into the STM32F405 CAN bootloader")
    parser.add_argument("node_id", type=int, help="board id, BOOTLOADER_NODE_ID in the bootloader")
    parser.add_argument("firmware", nargs="?", help="application .bin linked at 0x08008000")
    parser.add_argument("--bitrate", type=int, default=125000, help="CAN bit rate in bit/s")
    parser.add_argument("--channel", default="0", help="CAN channel, 0 for the first Kvaser")
    parser.add_argument("--interface", default="kvaser", help="python-can interface, for example kvaser or socketcan")
    parser.add_argument("--verify-only", action="store_true", help="compare flash with the file, write nothing")
    parser.add_argument("--info", action="store_true", help="print the bootloader information and exit")

    arguments = parser.parse_args(argv)

    if arguments.firmware is None and not arguments.info:
        parser.error("the firmware file is needed unless --info is given")

    return arguments


def main():
    arguments = parse_arguments()

    try:
        firmware = read_firmware(arguments.firmware) if arguments.firmware else None
        channel = parse_channel(arguments.channel)

        with can.Bus(interface=arguments.interface, channel=channel, bitrate=arguments.bitrate) as bus:
            print("Connecting to board %d..." % arguments.node_id)
            if not connect(bus, arguments.node_id):
                raise FlasherError("could not connect to board %d" % arguments.node_id)

            if arguments.info:
                print_info(bus, arguments.node_id)
            elif arguments.verify_only:
                verify_only(bus, arguments.node_id, firmware)
            else:
                flash(bus, arguments.node_id, firmware)

    except (FlasherError, can.CanError) as error:
        print("\nFlashing failed: %s" % error)
        sys.exit(1)


if __name__ == "__main__":
    main()
