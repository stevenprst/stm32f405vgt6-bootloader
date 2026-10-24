"""Run the flasher against the simulated board on a virtual CAN bus. No CAN hardware needed.

    python test_flasher.py
"""

import struct
import uuid

import can

import kvaser_flasher
from board_simulator import BoardSimulator, SimulatedBoard, crc_of_bytes
from kvaser_flasher import FlasherError

PAGE_BYTES = 1024


def make_image(page_count, reset_vector=0x08008101):
    """An app image of whole pages with a valid vector table and recognisable words after it."""
    words = [0x20020000, reset_vector]
    words += [(0xA5000000 + index) & 0xFFFFFFFF for index in range(page_count * PAGE_BYTES // 4 - 2)]

    return struct.pack("<%dI" % len(words), *words)


class Rig:
    """A board simulator and the host's bus, joined by a virtual CAN channel."""

    def __init__(self, drop_every_nth_reply=0, app_area_fill=0xFF):
        channel = "flashertest-" + uuid.uuid4().hex
        self.host_bus = can.Bus(interface="virtual", channel=channel)
        self.board_bus = can.Bus(interface="virtual", channel=channel)
        self.board = SimulatedBoard(node_id=1, app_area_fill=app_area_fill)
        self.simulator = BoardSimulator(self.board, self.board_bus, drop_every_nth_reply)

    def __enter__(self):
        self.simulator.start()
        assert kvaser_flasher.connect(self.host_bus, 1)
        return self

    def __exit__(self, *exc_info):
        self.simulator.stop()
        self.host_bus.shutdown()
        self.board_bus.shutdown()


def test_flash_crosses_sector_boundaries():
    # 40 pages open sector 2 (page 0), sector 3 (page 16) and sector 4 (page 32) over old data, so each must be erased.
    image = make_image(40)

    with Rig(app_area_fill=0x00) as rig:
        kvaser_flasher.flash(rig.host_bus, 1, image)

        assert rig.board.app_image(40) == image
        assert rig.board.metadata() == (40, crc_of_bytes(image))
        assert rig.board.erase_log == [0x08004000, 0x08008000, 0x0800C000, 0x08010000]
        assert rig.board.app_started


def test_flash_with_window_of_one():
    image = make_image(2)
    window = kvaser_flasher.WINDOW
    kvaser_flasher.WINDOW = 1

    try:
        with Rig() as rig:
            kvaser_flasher.flash(rig.host_bus, 1, image)

            assert rig.board.app_image(2) == image
    finally:
        kvaser_flasher.WINDOW = window


def test_lost_replies_are_asked_for_again():
    image = make_image(3)
    reply_timeout_s = kvaser_flasher.REPLY_TIMEOUT_S
    kvaser_flasher.REPLY_TIMEOUT_S = 0.05  # a lost reply costs one timeout, keep the test short

    try:
        with Rig(drop_every_nth_reply=7) as rig:
            kvaser_flasher.flash(rig.host_bus, 1, image)

            assert rig.board.app_image(3) == image
            assert rig.board.app_started
    finally:
        kvaser_flasher.REPLY_TIMEOUT_S = reply_timeout_s


def test_image_linked_for_the_wrong_address_is_refused():
    # The check runs in read_firmware, before the bus is even opened.
    try:
        kvaser_flasher.check_firmware(make_image(2, reset_vector=0x08000101))
    except FlasherError as error:
        assert "0x08000000" in str(error)
        return

    raise AssertionError("an image linked at 0x08000000 must be refused")


def test_image_too_big_or_empty_is_refused():
    for length in (0, (kvaser_flasher.MAX_PAGE_COUNT + 1) * PAGE_BYTES):
        try:
            kvaser_flasher.check_firmware(bytes(length))
        except FlasherError as error:
            assert "pages" in str(error)
        else:
            raise AssertionError("an image of %d bytes must be refused" % length)


def test_word_damaged_on_the_way_is_caught_by_the_page_crc():
    image = make_image(2)

    with Rig() as rig:
        rig.board.corrupt_word_index = 10

        try:
            kvaser_flasher.flash(rig.host_bus, 1, image)
        except FlasherError as error:
            assert "page CRC" in str(error)
            assert rig.board.metadata() == (0xFFFFFFFF, 0xFFFFFFFF)
            return

    raise AssertionError("a damaged word must stop the upload")


def test_page_out_of_order_is_refused():
    with Rig() as rig:
        page = make_image(1)[:PAGE_BYTES]
        kvaser_flasher.write_buffer(rig.host_bus, 1, page)

        try:
            kvaser_flasher.run_command(rig.host_bus, 1, kvaser_flasher.COMMAND_WRITE_PAGE, 5, crc_of_bytes(page))
        except FlasherError as error:
            assert "bad sequence" in str(error)
            return

    raise AssertionError("page 5 first must be refused")


def test_page_sent_before_all_words_is_refused():
    with Rig() as rig:
        kvaser_flasher.run_command(rig.host_bus, 1, kvaser_flasher.COMMAND_WRITE_BUFFER, 0, 0x12345678)

        try:
            kvaser_flasher.run_command(rig.host_bus, 1, kvaser_flasher.COMMAND_WRITE_PAGE, 0, 0)
        except FlasherError as error:
            assert "incomplete" in str(error)
            return

    raise AssertionError("a page with one word must be refused")


def test_image_with_a_bad_vector_table_gets_no_metadata():
    # The host check is skipped on purpose, to see what the bootloader does with a file that gets past it.
    image = make_image(2, reset_vector=0x08000101)

    with Rig() as rig:
        for page_index in range(2):
            page = image[page_index * PAGE_BYTES:(page_index + 1) * PAGE_BYTES]
            kvaser_flasher.upload_page(rig.host_bus, 1, page_index, page)

        try:
            kvaser_flasher.run_command(rig.host_bus, 1, kvaser_flasher.COMMAND_WRITE_CRC, 2, crc_of_bytes(image))
        except FlasherError as error:
            assert "bad image" in str(error)
            assert rig.board.metadata() == (0xFFFFFFFF, 0xFFFFFFFF)
            return

    raise AssertionError("the bootloader must not store metadata for an image that cannot run")


def test_verify_only_and_info():
    image = make_image(3)
    other_image = image[:100] + b"\x00\x00\x00\x00" + image[104:]

    with Rig() as rig:
        kvaser_flasher.flash(rig.host_bus, 1, image)

        # The app is running now; the trigger frame of connect() brings the board back into the bootloader.
        assert kvaser_flasher.connect(rig.host_bus, 1)
        kvaser_flasher.verify_only(rig.host_bus, 1, image)

        try:
            kvaser_flasher.verify_only(rig.host_bus, 1, other_image)
        except FlasherError as error:
            assert "CRC mismatch" in str(error)
        else:
            raise AssertionError("a different file must not verify")

        kvaser_flasher.print_info(rig.host_bus, 1)

        stored = kvaser_flasher.run_command(rig.host_bus, 1, kvaser_flasher.COMMAND_GET_INFO,
                                            kvaser_flasher.INFO_ITEM_STORED_APP, 0)
        assert stored.payload[0] | (stored.payload[1] << 8) == 3


def test_start_app_refused_without_an_app():
    with Rig() as rig:
        try:
            kvaser_flasher.run_command(rig.host_bus, 1, kvaser_flasher.COMMAND_START_APP, 0, 0)
        except FlasherError as error:
            assert "bad image" in str(error)
            return

    raise AssertionError("StartApp with a blank metadata sector must be refused")


def test_channel_is_a_number_for_kvaser_and_a_name_for_socketcan():
    assert kvaser_flasher.parse_channel("0") == 0
    assert kvaser_flasher.parse_channel("can0") == "can0"


if __name__ == "__main__":
    tests = [value for name, value in sorted(globals().items()) if name.startswith("test_")]

    for test in tests:
        test()
        print("passed:", test.__name__)

    print("all flasher checks passed")
