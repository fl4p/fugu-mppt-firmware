
import argparse
import os
import sys
from enum import Enum


# Standard page size for external SPI flash.
# I don't know that this value could be modified anyway.
# The design of the NVS system seems to be built around the assumption that page size = 4096 bytes.
PAGE_SIZE = 4096

# Page state bitmasks
# Copied from nvs_page.hpp
PSB_INIT = 0x01     # Page is initialized
PSB_FULL = 0x02     # Page is full, no more NVS entries available
PSB_FREE = 0x04     # Page is in the process of being freed (do not use this page)
PSB_CORRUPT = 0x08  # Page is corrupt (do not use this page)

# Entry state bitmasks
# copied from nvs_page.hpp
ESB_WRITTEN = 0x01  # This entry has been used
ESB_ERASED = 0x02   # This entry was in use but has been erased (do not use this entry)

# Offset and length of header fields within each page
PAGE_STATE_OFFSET = 0
PAGES_SEQ_NO_OFFSET = 4
ESB_OFFSET = 32
ESB_LENGTH = 32
ENTRY_0_OFFSET = 64
ENTRY_LENGTH = 32

# Offset of fields within each entry
NS_OFFSET = 0           # Namespace identifier for this entry
TYPE_OFFSET = 1         # Data type identifier for this entry
SPAN_OFFSET = 2         # Number of consecutive entry fields spanned by this entry
CHUNK_IDX_OFFSET = 3    # For use with blobs
#CRC_OFFSET = 4         # CRC is not checked
KEY_OFFSET = 8          # Key
KEY_LENGTH = 16
DATA_OFFSET = 24        # Data (length is determined by data type and span)

# Not part of NVS definition
# Used internally by Python script
class EntryState(Enum):
    UNUSED = 1
    WRITTEN = 2
    ERASED = 3

class EntryType(Enum):
    U8 = 0x01
    I8 = 0x11
    U16 = 0x02
    I16 = 0x12
    U32 = 0x04
    I32 = 0x14
    U64 = 0x08
    I64 = 0x18
    STR = 0x21
    BLOB = 0x42
    BLOB_IDX = 0x48

# Class used for reassambling complete BLOB data from individual chunks
class BlobReassemblyClass:
    def __init__(self):
        self.size = 0
        self.chunk_count = 0
        self.chunk_start = 0
        self.chunks = []

    def add_chunk(self, chunk_index, data):
        self.chunks.append( (chunk_index, data) )

    def add_blob_index_info(self, size, chunk_count, chunk_start):
        self.size = size
        self.chunk_count = chunk_count
        self.chunk_start = chunk_start

    def get_reassembled_blob(self):
        if self.size == 0:
            print("Cannot reassemble BLOB.  Info not set.")
            return bytearray()
        else:
            # Make sure list of chunks is sorted by chunk index
            self.chunks.sort(key=lambda x:x[0])

            # Sanity checks to make sure BLOB is being reassembled properly
            if self.chunks[0][0] != self.chunk_start:
                raise ValidationError("BLOB parsing error: Starting chunk index %d does not match chunk start value %d from BLOB index." % (self.chunks[0][0], self.chunk_start))
                return bytearray()
            if len(self.chunks) != self.chunk_count:
                raise ValidationError("BLOB parsing error: Number of chunks being reassembled %d does not match chunk count %d from BLOB index." % (len(self.chunks), self.chunk_count))
                return bytearray()

            # Now reassemble the chunks.
            # No additional checking of chunk indices is done at this point.
            blob_data = bytearray()
            for chunk in self.chunks:
                blob_data.extend(chunk[1])

            return blob_data


# Keys used for SSIDs
# These are stored as BLOBs but can be printed as strings
SSID_KEY_VALUES = ["sta.ssid", "ap.ssid"]

# Keys used for passwords
# These are stored as BLOBs but can be printed as strings
PASSWORD_KEY_VALUES = ["sta.pswd", "ap.passwd"]


# A dictionary where key = namespace name and value = namespace index
# User to convert the index back to the namespace name, which is used as the keys for nvs_table
ns_idx_to_name = {}

# The entire NVS table is parsed into this dictionary
# Keys = namespace name, Each value is another dictionary
# For each namespace dictionary
#   Key = NVS entry key, data = tuple of following format
#      (entry type, data)
#      where data may be integer, list, or string
nvs_table = {}

# This is organized similar to nvs_table with top level key being namespace name
# and second level key being key value for BLOB entry.
# It differs in that the BLOB value itself is represented by a BlobReassemblyClass, which is
# used to collect the BLOB pieces before generating the final byte array that goes into nvs_table.
blob_reassembly_table = {}

def verify_nvs_size(input):
    if (len(input) % PAGE_SIZE) != 0:
        message = "Partition size (%d bytes) is not a multiple of page size (%d bytes)" % (len(input), PAGE_SIZE)
        raise ValidationError(message)

    global num_pages
    num_pages = int(len(input) / PAGE_SIZE)
    # print("NVS partition has %d pages" % (num_pages))


def build_page_array(input):
    # List of tuples
    # Each tuple is (page_index, sequence_number)
    # Where each page is a PAGE_SIZE-byte starting with page index 0 at beginning of file
    global page_array
    page_array = []

    for page_index in range(num_pages):
        page_base_address = PAGE_SIZE * page_index

        page_state = int.from_bytes(input[(page_base_address+PAGE_STATE_OFFSET):(page_base_address+PAGE_STATE_OFFSET+4)], byteorder='little', signed=False)
        seq_no = int.from_bytes(input[(page_base_address+PAGES_SEQ_NO_OFFSET):(page_base_address+PAGES_SEQ_NO_OFFSET+4)], byteorder='little', signed=False)

        # print("Page %d: State=0x%08X, Seq=%d" % (page_index, page_state, seq_no))

        # If page is in use...
        if (page_state & PSB_INIT) == 0:
            # ...and page is not corrupt or in the process of being freed
            if (page_state & (PSB_FREE | PSB_CORRUPT)) == (PSB_FREE | PSB_CORRUPT):
                # ...then add page to page array
                page_array.append( (page_index, seq_no) )
                # print("  Added this page")

    # Finished scanning pages and adding them to the array
    # Now we sort it based on sequence number
    page_array.sort(key=lambda x:x[1])
    # print("Page array (page index, seq number):")
    # for e in page_array:
    #     print(e)


def scan_in_page(page_data, page_index):
    entry_state_bitmap = page_data[ESB_OFFSET:(ESB_OFFSET+ESB_LENGTH)]

    entry_states = []
    for byte in entry_state_bitmap:
        for i in range(4):
            entry_state_bits = byte & 0x3
            if (entry_state_bits & ESB_WRITTEN) == 0:
                if (entry_state_bits & ESB_ERASED) == 0:
                    entry_states.append(EntryState.ERASED)
                else:
                    entry_states.append(EntryState.WRITTEN)
            else:
                entry_states.append(EntryState.UNUSED)
            byte >>= 2

    # Only 126 entries are available since the first 64 bytes are taken up by header and entry state bitmap
    del entry_states[126:128]

    i = 0
    while i < len(entry_states):
        # print("Entry %d at offset %02X is %s" % (i, ENTRY_0_OFFSET + (ENTRY_LENGTH*i), entry_states[i].name))
        if entry_states[i] == EntryState.WRITTEN:
            entry_base = ENTRY_0_OFFSET + (ENTRY_LENGTH*i)
            # print("Parsing entry %d at offset %02X" % (i, entry_base))

            # Parse header fields for entry
            entry_ns = page_data[entry_base+NS_OFFSET]
            entry_type = page_data[entry_base+TYPE_OFFSET]
            entry_span = page_data[entry_base+SPAN_OFFSET]
            entry_chunk_idx = page_data[entry_base+CHUNK_IDX_OFFSET]
            entry_key_data = page_data[entry_base+KEY_OFFSET:entry_base+KEY_OFFSET+KEY_LENGTH]

            # Have to do some special handling here.  Simply decoding directly to string will leave in the null terminators.
            # We have to first slice the byte array at the first null terminator.
            if 0 in entry_key_data:
                entry_key_data = entry_key_data[0:entry_key_data.find(0)]
            entry_key = entry_key_data.decode('ascii')

            # Read out entry data
            # Note that for standard integer types, the enum values can be used directly and not interpreted as enums.
            do_not_add = False
            if entry_type < 0x20:
                num_bytes = entry_type & 0xf
                is_signed = False if (entry_type & 0x10) == 0 else True
                entry_data = int.from_bytes(page_data[entry_base+DATA_OFFSET:entry_base+DATA_OFFSET+num_bytes], byteorder='little', signed=is_signed)

            elif EntryType(entry_type) == EntryType.STR:
                data_size = int.from_bytes(page_data[entry_base+DATA_OFFSET:entry_base+DATA_OFFSET+2], byteorder='little', signed=False)
                entry_data = page_data[entry_base+DATA_OFFSET+8:entry_base+DATA_OFFSET+8+data_size].decode('ascii')

            elif EntryType(entry_type) == EntryType.BLOB:
                data_size = int.from_bytes(page_data[entry_base+DATA_OFFSET:entry_base+DATA_OFFSET+2], byteorder='little', signed=False)
                entry_data = page_data[entry_base+DATA_OFFSET+8:entry_base+DATA_OFFSET+8+data_size]
                namespace_name = ns_idx_to_name[entry_ns]
                namespace_dict = blob_reassembly_table[namespace_name]
                if entry_key not in namespace_dict:
                    namespace_dict[entry_key] = BlobReassemblyClass()
                namespace_dict[entry_key].add_chunk(entry_chunk_idx, entry_data)
                # print("  Saved %d byte chunk with index %d for BLOB %s" % (len(entry_data), entry_chunk_idx, entry_key))
                do_not_add = True   # BLOB will be added when all pieces are reassembled

            elif EntryType(entry_type) == EntryType.BLOB_IDX:
                blob_size = int.from_bytes(page_data[entry_base+DATA_OFFSET:entry_base+DATA_OFFSET+4], byteorder='little', signed=False)
                chunk_count = int.from_bytes(page_data[entry_base+DATA_OFFSET+4:entry_base+DATA_OFFSET+5], byteorder='little', signed=False)
                chunk_start = int.from_bytes(page_data[entry_base+DATA_OFFSET+5:entry_base+DATA_OFFSET+6], byteorder='little', signed=False)
                namespace_name = ns_idx_to_name[entry_ns]
                namespace_dict = blob_reassembly_table[namespace_name]
                namespace_dict[entry_key].add_blob_index_info(blob_size, chunk_count, chunk_start)
                # print("  Reassembling BLOB %s" % (entry_key))
                entry_data = namespace_dict[entry_key].get_reassembled_blob()
                entry_type = EntryType.BLOB.value   # Store into nvs_table as type BLOB

            else:
                do_not_add = True;
                print("  Unknown entry type 0x%02X for entry %d at offset 0x%X in page %d" % (entry_type, i, entry_base, page_index))

            # print("  NS = %d, type = %02X, span = %d, key = %s" % (entry_ns, entry_type, entry_span, entry_key))
            if entry_ns == 0:
                # print("  Added namespace %s" % (entry_key))
                ns_idx_to_name[entry_data] = entry_key
                nvs_table[entry_key] = {}
                blob_reassembly_table[entry_key] = {}
            elif do_not_add == False:
                namespace_name = ns_idx_to_name[entry_ns]
                namespace_dict = nvs_table[namespace_name]
                namespace_dict[entry_key] = (entry_type, entry_data)
                # print("  Added key %s to namespace %s" % (entry_key, namespace_name))

            if entry_span > 0:
                i += entry_span
            else:
                i += 1      # Failsafe - Span should never equal 0 on a valid entry.
        else:
            i += 1


def parse_nvs_binary(input):
    build_page_array(input)
    for page in page_array:
        page_idx = page[0]
        seq_no = page[1]
        # print("Parsing page %d with seq no %d" % (page_idx, seq_no))
        page_base = page_idx * PAGE_SIZE
        scan_in_page(input[page_base:page_base+PAGE_SIZE], page_idx)


def entry_data_to_string(entry_key, entry_type, entry_data):
    data_str = ''
    if entry_type < 0x20:
        data_str = str(entry_data)
    elif EntryType(entry_type) == EntryType.STR:
        data_str = entry_data
    elif EntryType(entry_type) == EntryType.BLOB:
        byte_count = 0
        if special_handling:
            handle_ssid = True if entry_key in SSID_KEY_VALUES else False
            handle_passphrase = True if entry_key in PASSWORD_KEY_VALUES else False
        else:
            handle_ssid = False
            handle_passphrase = False

        if handle_ssid:
            # Format for SSIDs is 4 bytes of length followed by string data
            ssid_len = int.from_bytes(entry_data[0:4], byteorder='little', signed=False)
            if ssid_len <= 32:
                data_str = entry_data[4:4+ssid_len].decode('utf-8')
            else:
                data_str = ''                                           # This field is not initialized
        elif handle_passphrase:
            if entry_data[0] == 0xFF:
                data_str = ''                                           # This field is not initialized
            else:
                # Same treatment as we did with entry keys.  Data needs to be truncated at the null terminator.
                if 0 in entry_data:
                    entry_data = entry_data[0:entry_data.find(0)]
                data_str = entry_data.decode('utf-8')
        else:
            for data_byte in entry_data:
                if (blob_limit>0) and (byte_count==blob_limit):
                    data_str += '...'
                    break
                else:
                    data_str += format(data_byte, "02X") + ' '
                    byte_count += 1
    return data_str


def dump_nvs_data():
    for ns_name in nvs_table:
        ns_dict = nvs_table[ns_name]
        print("Namespace %s" % (ns_name))
        for entry_key in ns_dict:
            entry_info = ns_dict[entry_key]

            entry_type = entry_info[0]
            entry_data = entry_info[1]

            data_str = entry_data_to_string(entry_key, entry_type, entry_data)

            print("  %-16s: %-4s  %s" % (entry_key, EntryType(entry_type).name, data_str))


def main():
    global blob_limit
    global special_handling

    parser = argparse.ArgumentParser(description='ESP32 NVS partition analyzer')
    parser.add_argument('input', help='Path to binary dump of NVS partition.', type=argparse.FileType('rb'))
    parser.add_argument('--blob_limit', '-b', help='Set maximum number of bytes to dump per BLOB (0 = no limit)', type=int, default=0)
    parser.add_argument('--special_handling', '-s', help='Display SSID and password BLOBs as strings', action='store_true')
    args = parser.parse_args()

    blob_limit = args.blob_limit
    special_handling = args.special_handling
    input = args.input.read()
    verify_nvs_size(input)

    parse_nvs_binary(input)
    dump_nvs_data()



class InputError(RuntimeError):
    def __init__(self, e):
        super(InputError, self).__init__(e)


class ValidationError(InputError):
    def __init__(self, message):
        super(ValidationError, self).__init__(
            "NVS partition invalid: %s" % (message))


if __name__ == '__main__':
    try:
        main()
    except InputError as e:
        print(e, file=sys.stderr)
        sys.exit(2)