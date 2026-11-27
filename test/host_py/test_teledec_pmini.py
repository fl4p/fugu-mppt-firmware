import base64
import struct
import sys
from binascii import crc32

sys.path.insert(0, "etc/fugu")  # import the decoder without the serial console extras
import teledec


# Three rows, one u32 field, and a literal-free data stream.  The fixture was
# produced by the canonical pmini encoder; the expected bytes are the PMA2
# envelope's reconstructed length-prefixed sym_line_protocol frames.
PACKET = base64.b64decode(
    "UE1BMgEEBQEBAgMAAQQHAB4dAgFtcHB0AAACZGV2aWNlAAADZmxhdAAABHAABwABAQFNVFAyAwQ6yLYAVRFMQyN1IIueiIjGXdZWD/UVaxQAAwAEAC/SP1WbJqJi"
)
RAW = bytes.fromhex(
    "1d02016d707074000002646576696365000003666c617400000470000700"
    "0c010102030004785634120064"
    "0c010102030004efbeadde0065"
    "100101020300043412000000e680808010"
)


def test_pmini_packet_reconstructs_exact_wire_bytes():
    assert teledec.decode_packet(PACKET) == RAW


def test_pmini_packet_uses_existing_table_and_data_decoder():
    table = {}
    lines = list(teledec.decode_payload(2, PACKET, table))
    assert lines == [
        "mppt,device=flat p=305419896i 100",
        "mppt,device=flat p=3735928559i 101",
        "mppt,device=flat p=4660i 4294967398",
    ]
    assert table[1] == ("mppt", 0)


def test_pmini_crc_failure_does_not_mutate_table():
    packet = bytearray(PACKET)
    packet[-1] ^= 1
    table = {99: ("old", 7)}
    try:
        list(teledec.decode_payload(2, packet, table))
    except ValueError as exc:
        assert "CRC" in str(exc)
    else:
        raise AssertionError("corrupt PMA2 packet was accepted")
    assert table == {99: ("old", 7)}


def test_pmini_crc_valid_bad_descriptor_and_truncation_are_rejected():
    malformed = bytearray(PACKET)
    malformed[15] = 2  # undefined event kind, then repair the outer CRC
    struct.pack_into("<I", malformed, len(malformed) - 4, crc32(malformed[:-4]) & 0xFFFFFFFF)
    table = {99: ("old", 7)}
    try:
        list(teledec.decode_payload(2, malformed, table))
    except ValueError as exc:
        assert "descriptor" in str(exc)
    else:
        raise AssertionError("CRC-valid malformed PMA2 was accepted")
    assert table == {99: ("old", 7)}
    for size in range(len(PACKET) - 1):
        try:
            teledec.decode_packet(PACKET[:size])
        except ValueError:
            pass
        else:
            raise AssertionError(f"truncated PMA2 accepted at {size}")


def test_pmini_oversized_rows_rejected_before_block_decode():
    # Replace the MTP2 row-count varint (3) with canonical 129, then repair
    # both the shard CRC and the enclosing PMA2 CRC. The packet is CRC-valid,
    # but the PMA2 128-row bound must fire before rANS materialization.
    shard_at = PACKET.index(b"MTP2")
    shard = PACKET[shard_at:-4]
    body = shard[:-4]
    body = body[:4] + b"\x81\x01" + body[5:]
    shard = body + struct.pack("<I", crc32(body) & 0xFFFFFFFF)
    envelope = PACKET[:shard_at] + shard
    oversized = envelope + struct.pack("<I", crc32(envelope) & 0xFFFFFFFF)
    try:
        teledec.decode_packet(oversized)
    except ValueError as exc:
        assert "row/event" in str(exc)
    else:
        raise AssertionError("CRC-valid 129-row PMA2 packet was accepted")
