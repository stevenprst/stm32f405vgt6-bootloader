"""Check the flasher CRC against the STM32 hardware CRC. No CAN hardware needed.

    python test_crc.py

The expected values come from the definition of the unit: polynomial 0x04C11DB7, start value 0xFFFFFFFF, no
reflection, one 32-bit word at a time. 0x12345678 giving 0xDF8A8A2B is the value the ST CRC examples show.
"""

import struct

from board_simulator import crc_bit_by_bit
from kvaser_flasher import stm32_crc


def little_endian_bytes(words):
    return b"".join(struct.pack("<I", word) for word in words)


def test_known_values():
    assert stm32_crc(little_endian_bytes([0x12345678])) == 0xDF8A8A2B
    assert stm32_crc(little_endian_bytes([0, 0])) == 0x6904BB59
    assert stm32_crc(little_endian_bytes([0xDEADBEEF, 1, 0xFFFFFFFF, 0x08008000])) == 0xBC983515


def test_table_matches_bit_by_bit():
    words = [(index * 0x9E3779B1) & 0xFFFFFFFF for index in range(256)]

    assert stm32_crc(little_endian_bytes(words)) == crc_bit_by_bit(words)


def test_partial_word_is_refused():
    try:
        stm32_crc(b"\x01\x02\x03")
    except ValueError:
        return

    raise AssertionError("a length that is not a multiple of 4 must be refused")


if __name__ == "__main__":
    test_known_values()
    test_table_matches_bit_by_bit()
    test_partial_word_is_refused()
    print("all CRC checks passed")
