# Import("env")

import struct
import subprocess
import sys
import os
import re


def read_header(file):
    struct_fmt = '=4s1I1I1I1I'
    struct_len = struct.calcsize(struct_fmt)
    struct_unpack = struct.Struct(struct_fmt).unpack_from

    header = {}

    data = file.read(struct_len)
    s = struct_unpack(data)
    header["header"] = s[0]
    header["pointer_size"] = s[1]
    header["samples_per_bank"] = s[2]
    header["samples_per_second"] = s[3]
    header["cores"] = s[4]
    return header


def read_bank(file, samples_per_bank):
    profiling_item_fmt = "1I1I1I1I1I"
    profiling_item_len = struct.calcsize(profiling_item_fmt)
    profiling_item_unpack = struct.Struct(profiling_item_fmt).unpack_from

    profiling_bank_fmt = "1I1I"
    profiling_bank_len = struct.calcsize(profiling_bank_fmt)
    profiling_bank_unpack = struct.Struct(profiling_bank_fmt).unpack_from

    items = []

    for i in range(0, samples_per_bank):
        data = file.read(profiling_item_len)
        if not data:
            break
        s = profiling_item_unpack(data)
        item = {}
        item["callee"] = s[0]
        item["caller"] = s[1]
        item["calls"] = s[2]
        item["cycles"] = s[3]
        item["instructions"] = s[4]

        items.append(item)

    if len(items) == 0:
        return None

    data = file.read(profiling_bank_len)
    if not data:
        return None
    s = profiling_bank_unpack(data)

    bank = {}
    bank["items"] = items
    bank["last_index"] = s[0]
    bank["check_number"] = s[1]
    return bank


def update_samples_item(samples, item):
    exists = False
    for sample in samples:
        if sample["caller"] == item["caller"] and sample["callee"] == item["callee"]:
            sample["calls"] += item["calls"]
            exists = True
    if not exists:
        samples.append(item)


def merge_samples(banks):
    samples = []
    for bank in banks:
        for i in range(0, bank["last_index"]):
            item = bank["items"][i]
            update_samples_item(samples, item)
    return samples


ADDR2LINE_BINARY = 'riscv32-esp-elf-addr2line'

def addr2line(binary_filename, address):
    output = subprocess.check_output(
        [ADDR2LINE_BINARY, '-C', '-f', '-e', binary_filename, address]).decode(sys.stdout.encoding).split("\n")
    return (output[0].strip(), output[1].strip())


def line_get_filename(line):
    return ":".join(line.split(":")[:-1])


def line_get_line_number(line):
    line_number = line.split(":")[-1]
    line_number = re.sub(r'\(discriminator \d+\)', '', line_number)
    return line_number


def merge_function_info(samples):
    firmware_name = "../build/main.elf"
    for sample in samples:
        caller_addrinfo = addr2line(
            firmware_name, hex(sample["caller"]))
        callee_addrinfo = addr2line(
            firmware_name, hex(sample["callee"]))
        sample["caller_function"] = caller_addrinfo[0]
        caller_fullfile = line_get_filename(caller_addrinfo[1])
        caller_line = line_get_line_number(caller_addrinfo[1])
        sample["caller_fullfile"] = caller_fullfile
        sample["caller_file"] = os.path.basename(
            caller_fullfile.replace("\\", "/"))
        sample["caller_line"] = caller_line

        sample["callee_function"] = callee_addrinfo[0]

        callee_fullfile = line_get_filename(callee_addrinfo[1])
        callee_line = line_get_line_number(callee_addrinfo[1])
        sample["callee_fullfile"] = callee_fullfile
        sample["callee_file"] = os.path.basename(
            callee_fullfile.replace("\\", "/"))
        sample["callee_line"] = callee_line

def has_children(samples, callee):
    for sample in samples:
        if sample["caller"] == callee:
            return True
    return False

def filter_top_of_stack(samples):
    filtered = []
    for sample in samples:
        if not has_children(samples, sample["callee"]):
            filtered.append(sample)
    return filtered

def generate_callgrind(sorted_samples):

    with open("callgrind.sprof", "w") as cg:
        cg.write("# callgrind format\n")
        cg.write("events: Captures Instructions Cycles\n\n")

        no_callers = filter_top_of_stack(sorted_samples)

        for sample in no_callers:
            caller_function = sample["caller_function"]
            if caller_function == "??":
                caller_function = hex(sample["caller"])
            callee_function = sample["callee_function"]
            if callee_function == "??":
                callee_function = hex(sample["callee"])
            cg.write('fl=%s\n' % (sample["callee_fullfile"]))
            cg.write('fn=%s\n' % (callee_function))
            cg.write('%s %s %s %s\n' % (sample["callee_line"], sample["calls"], sample["instructions"], sample["cycles"]))
            cg.write("\n")

        for sample in sorted_samples:
            caller_function = sample["caller_function"]
            if caller_function == "??":
                caller_function = hex(sample["caller"])
            callee_function = sample["callee_function"]
            if callee_function == "??":
                callee_function = hex(sample["callee"])
            cg.write('fl=%s\n' % (sample["caller_fullfile"]))
            cg.write('fn=%s\n' % (caller_function))
            cg.write('%s %s %s %s\n' % (sample["caller_line"], 1, 1, 1))
            cg.write('cfi=%s\n' % (sample["callee_fullfile"]))
            cg.write('cfn=%s\n' % (callee_function))
            cg.write('calls=%s %s\n' % (1, sample["callee_line"]))
            cg.write('%s %s %s %s\n' % (sample["caller_line"], sample["calls"], sample["instructions"], sample["cycles"]))
            cg.write("\n")

def read_sprofiler():
    with open("sprof.out", "rb") as f:
        header = read_header(f)
        print("Analysing sprof.out")
        print("Pointer Size: ", header["pointer_size"])
        print("Samples Per Bank: ", header["samples_per_bank"])
        print("Samples Per Second: ", header["samples_per_second"])
        print("Cores: ", header["cores"])
        banks = []
        while (True):
            bank = read_bank(f, header["samples_per_bank"])
            if not bank:
                break
            banks.append(bank)

        print("Resolving Symbols...")
        samples = merge_samples(banks)
        merge_function_info(samples)
        sorted_samples = sorted(
            samples, key=lambda x: int(x["calls"]), reverse=True)
        for sample in sorted_samples:
            caller_function = sample["caller_function"]
            if caller_function == "??":
                caller_function = hex(sample["caller"])
            callee_function = sample["callee_function"]
            if callee_function == "??":
                callee_function = hex(sample["callee"])

            print(caller_function, '%s:%s' % (sample["caller_file"], sample["caller_line"]), "->", callee_function, '%s:%s' % (sample["callee_file"], sample["callee_line"]), ":",
                sample["calls"],
                sample["cycles"],
                sample["instructions"])

        print("Generating callgrind.sprof")
        generate_callgrind(sorted_samples)
        print("Done")

def analyze_results(*arg, **kwargs):
    read_sprofiler()

#env.AddCustomTarget(
#    "analyze",
#    None,
#    analyze_results,
#    title="Analyze sprofiler",
#    description="Analyze sprofiler and generate callgrind results")

def main():
    read_sprofiler()

main()

#  /Users/fab/.espressif/tools/riscv32-esp-elf/esp-12.2.0_20230208/riscv32-esp-elf/bin/riscv32-esp-elf-addr2line