#!/bin/bash
set -e

FOLDER="config"
BOARD="$1"
echo "BOARD=$BOARD"
if [[ ! -d "$FOLDER/$BOARD" ]] || [[ "" == "$BOARD" ]]; then
  echo "invalid board, choose from"
  ls $FOLDER/
  exit 1
fi

# TODO
# check pins.conf/mcu == current target chip

littlefs-python create $FOLDER/"$BOARD" $FOLDER/"$BOARD".bin -v --fs-size=0x20000 --name-max=64 --block-size=4096
 littlefs-python  list $FOLDER/"$BOARD".bin --block-size=4096
parttool.py --port $ESPPORT write_partition --partition-name littlefs --input $FOLDER/"$BOARD".bin