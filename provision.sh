#!/bin/bash
set -e

BOARD="$1"
echo "BOARD=$BOARD"
if [[ ! -d "provisioning/$BOARD" ]] || [[ "" == "$BOARD" ]]; then
  echo "invalid board, choose from"
  ls provisioning/
  exit 1
fi

# TODO
# check pins.conf/mcu == current target chip

littlefs-python create provisioning/"$BOARD" provisioning/"$BOARD".bin -v --fs-size=0x20000 --name-max=64 --block-size=4096
parttool.py --port $ESPPORT write_partition --partition-name littlefs --input provisioning/"$BOARD".bin