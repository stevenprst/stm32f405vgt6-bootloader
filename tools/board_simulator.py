"""A software model of the bootloader in Core/Src/mainloop.cpp, for testing the flasher without a board.

It follows the same rules as the C++ (page buffer, page CRC, page order, lazy sector erase, metadata, vector table
check) and keeps its flash like a real one: erased bytes read 0xFF, and programming can only clear bits.

    board = SimulatedBoard(node_id=1)
    simulator = BoardSimulator(board, bus)   # a python-can bus, for example the virtual one
    simulator.start() ... simulator.stop()
"""

import struct
import threading

import can

# Nothing is imported from kvaser_flasher: the protocol numbers and the CRC are written again here from the README,
# so a wrong constant or a wrong CRC in the flasher shows up as a failing test instead of agreeing with itself.
COMMAND_CAN_ID = 0x6A5
RESPONSE_CAN_ID = 0x6A6
TRIGGER_CAN_ID = 0x6A4
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

PAGE_SIZE_WORDS = 256
PAGE_SIZE_BYTES = 1024
MAX_PAGE_COUNT = 992

APP_ADDRESS = 0x08008000
APP_END_ADDRESS = 0x08100000
RAM_BEGIN_ADDRESS = 0x20000000
RAM_END_ADDRESS = 0x20020000
CCM_RAM_BEGIN_ADDRESS = 0x10000000
CCM_RAM_END_ADDRESS = 0x10010000


def crc_bit_by_bit(words):
    """The STM32 hardware CRC, one bit at a time: polynomial 0x04C11DB7, start 0xFFFFFFFF, no reflection."""
    crc = 0xFFFFFFFF

    for word in words:
        crc ^= word

        for _ in range(32):
            if crc & 0x80000000:
                crc = ((crc << 1) ^ 0x04C11DB7) & 0xFFFFFFFF
            else:
                crc = (crc << 1) & 0xFFFFFFFF

    return crc


def crc_of_bytes(data):
    return crc_bit_by_bit(struct.unpack("<%dI" % (len(data) // 4), data))


FLASH_BASE = 0x08000000
FLASH_SIZE = 0x100000
METADATA_SECTOR_ADDRESS = 0x08004000
METADATA_ADDRESS = APP_ADDRESS - PAGE_SIZE_BYTES

# (start address, size) of the 12 sectors.
FLASH_SECTORS = [
    (0x08000000, 16 * 1024), (0x08004000, 16 * 1024), (0x08008000, 16 * 1024), (0x0800C000, 16 * 1024),
    (0x08010000, 64 * 1024),
    (0x08020000, 128 * 1024), (0x08040000, 128 * 1024), (0x08060000, 128 * 1024), (0x08080000, 128 * 1024),
    (0x080A0000, 128 * 1024), (0x080C0000, 128 * 1024), (0x080E0000, 128 * 1024),
]

OK = 0
BAD_INDEX = 1
CRC_MISMATCH = 2
PROGRAM_FAILED = 3
ERASE_FAILED = 4
PAGE_INCOMPLETE = 5
PAGE_CRC_MISMATCH = 6
BAD_SEQUENCE = 7
BAD_IMAGE = 8
UNKNOWN_COMMAND = 9

BOOTLOADER_VERSION = 2


class SimulatedBoard:
    def __init__(self, node_id=1, app_area_fill=0xFF):
        self.node_id = node_id
        self.flash = bytearray(b"\xFF" * FLASH_SIZE)

        # A used board has old data in the app area, which only an erase clears.
        app_offset = APP_ADDRESS - FLASH_BASE
        self.flash[app_offset:] = bytes([app_area_fill]) * (FLASH_SIZE - app_offset)

        self.in_bootloader = False
        self.app_started = False
        self.reset_flags = 1  # power-on
        self.can_error = 0

        self.page_buffer = [0] * PAGE_SIZE_WORDS
        self.word_present = [False] * PAGE_SIZE_WORDS
        self.page_committed = False
        self.next_page_index = 0
        self.erase_log = []

        # Test hooks: change one word on its way into the buffer, once.
        self.corrupt_word_index = None

    # ---- flash ----

    def read_word(self, address):
        offset = address - FLASH_BASE
        return struct.unpack_from("<I", self.flash, offset)[0]

    def erase_sector_starting_at(self, address):
        for start, size in FLASH_SECTORS:
            if start == address:
                offset = start - FLASH_BASE
                self.flash[offset:offset + size] = b"\xFF" * size
                self.erase_log.append(start)
                return True

        return False

    def program_words(self, address, words):
        offset = address - FLASH_BASE
        wanted = struct.pack("<%dI" % len(words), *words)

        if bytes(self.flash[offset:offset + len(wanted)]) == wanted:
            return True

        # Programming can only clear bits, so a word written over old data is the AND of both.
        for index, byte in enumerate(wanted):
            self.flash[offset + index] &= byte

        return bytes(self.flash[offset:offset + len(wanted)]) == wanted

    def metadata(self):
        return struct.unpack_from("<II", self.flash, METADATA_ADDRESS - FLASH_BASE)

    def vector_table_is_valid(self):
        stack_pointer = self.read_word(APP_ADDRESS)
        reset_vector = self.read_word(APP_ADDRESS + 4)

        stack_in_ram = RAM_BEGIN_ADDRESS <= stack_pointer <= RAM_END_ADDRESS
        stack_in_ccm_ram = CCM_RAM_BEGIN_ADDRESS <= stack_pointer <= CCM_RAM_END_ADDRESS
        reset_is_thumb = (reset_vector & 1) == 1
        reset_in_app = APP_ADDRESS <= (reset_vector & ~1) < APP_END_ADDRESS

        return (stack_in_ram or stack_in_ccm_ram) and reset_is_thumb and reset_in_app

    def app_image(self, page_count):
        offset = APP_ADDRESS - FLASH_BASE
        return bytes(self.flash[offset:offset + page_count * PAGE_SIZE_BYTES])

    def check_stored_app(self):
        page_count, app_crc = self.metadata()

        if page_count == 0 or page_count > MAX_PAGE_COUNT:
            return BAD_IMAGE

        if crc_of_bytes(self.app_image(page_count)) != app_crc:
            return CRC_MISMATCH

        if not self.vector_table_is_valid():
            return BAD_IMAGE

        return OK

    # ---- commands ----

    def handle_trigger(self, data):
        node_id, magic = struct.unpack("<BH", bytes(data[:3]))

        if node_id == self.node_id and magic == TRIGGER_MAGIC and not self.in_bootloader:
            self.in_bootloader = True
            self.app_started = False

    def handle_command(self, data):
        """The 8 command bytes in, the 8 reply bytes out. None when the bootloader sends no reply."""
        if not self.in_bootloader or len(data) != 8:
            return None

        node_id, command, index, value = struct.unpack("<BBHI", bytes(data))

        if node_id != self.node_id:
            return None

        payload = b"\0\0\0"

        if command == COMMAND_PING:
            code = OK
        elif command == COMMAND_WRITE_BUFFER:
            code = self.write_buffer(index, value)
        elif command == COMMAND_WRITE_PAGE:
            code = self.write_page(index, value)
        elif command == COMMAND_WRITE_CRC:
            code = self.write_crc(index, value)
        elif command == COMMAND_GET_INFO:
            code, payload = self.get_info(index)
        elif command == COMMAND_START_APP:
            code = self.check_stored_app()
            if code == OK:
                self.in_bootloader = False
                self.app_started = True
        else:
            code = UNKNOWN_COMMAND

        return struct.pack("<BBBH", self.node_id, command, code, index) + payload

    def write_buffer(self, index, value):
        if index >= PAGE_SIZE_WORDS:
            return BAD_INDEX

        if self.page_committed:
            self.word_present = [False] * PAGE_SIZE_WORDS
            self.page_committed = False

        if self.corrupt_word_index == index:
            value ^= 0x00010000
            self.corrupt_word_index = None

        self.word_present[index] = True
        self.page_buffer[index] = value
        return OK

    def write_page(self, page_index, page_crc):
        if page_index >= MAX_PAGE_COUNT:
            return BAD_INDEX

        is_restart = page_index == 0
        is_next_page = page_index == self.next_page_index
        is_last_page_again = self.next_page_index != 0 and page_index == self.next_page_index - 1

        if not (is_restart or is_next_page or is_last_page_again):
            return BAD_SEQUENCE

        if not all(self.word_present):
            return PAGE_INCOMPLETE

        buffer_bytes = struct.pack("<%dI" % PAGE_SIZE_WORDS, *self.page_buffer)

        if crc_of_bytes(buffer_bytes) != page_crc:
            return PAGE_CRC_MISMATCH

        page_address = APP_ADDRESS + page_index * PAGE_SIZE_BYTES

        if page_index == 0 and not self.erase_sector_starting_at(METADATA_SECTOR_ADDRESS):
            return ERASE_FAILED

        starts_sector = any(start == page_address for start, _ in FLASH_SECTORS)

        if starts_sector and not self.erase_sector_starting_at(page_address):
            return ERASE_FAILED

        if not self.program_words(page_address, self.page_buffer):
            return PROGRAM_FAILED

        self.next_page_index = page_index + 1
        self.page_committed = True
        return OK

    def write_crc(self, page_count, host_crc):
        if page_count == 0 or page_count > MAX_PAGE_COUNT:
            return BAD_INDEX

        if crc_of_bytes(self.app_image(page_count)) != host_crc:
            return CRC_MISMATCH

        if not self.vector_table_is_valid():
            return BAD_IMAGE

        if not self.program_words(METADATA_ADDRESS, [page_count, host_crc]):
            return PROGRAM_FAILED

        return OK

    def get_info(self, item):
        if item == INFO_ITEM_VERSION:
            return OK, struct.pack("<BH", BOOTLOADER_VERSION, MAX_PAGE_COUNT)

        if item == INFO_ITEM_STORED_APP:
            page_count, _ = self.metadata()
            return OK, struct.pack("<HB", page_count & 0xFFFF, self.reset_flags)

        if item == INFO_ITEM_CAN_ERROR:
            return OK, struct.pack("<I", self.can_error)[:3]

        return BAD_INDEX, b"\0\0\0"


class BoardSimulator:
    """Runs a SimulatedBoard on a python-can bus in a thread."""

    def __init__(self, board, bus, drop_every_nth_reply=0):
        self.board = board
        self.bus = bus
        self.drop_every_nth_reply = drop_every_nth_reply
        self._replies_seen = 0
        self._stop = threading.Event()
        self._thread = threading.Thread(target=self._run, daemon=True)

    def start(self):
        self._thread.start()

    def stop(self):
        self._stop.set()
        self._thread.join(timeout=2.0)

    def _run(self):
        while not self._stop.is_set():
            message = self.bus.recv(0.02)

            if message is None:
                continue

            if message.arbitration_id == TRIGGER_CAN_ID:
                self.board.handle_trigger(message.data)
                continue

            if message.arbitration_id != COMMAND_CAN_ID:
                continue

            reply = self.board.handle_command(message.data)

            if reply is None:
                continue

            self._replies_seen += 1
            drop_this_reply = self.drop_every_nth_reply != 0 and self._replies_seen % self.drop_every_nth_reply == 0

            if not drop_this_reply:
                self.bus.send(can.Message(arbitration_id=RESPONSE_CAN_ID, is_extended_id=False, data=reply))
