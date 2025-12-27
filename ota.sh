#!/bin/bash
set -e

(python3 -m http.server 9000 2> /dev/null || true) &
trap "trap - SIGTERM && kill -- -$$" SIGINT SIGTERM EXIT
# ^https://stackoverflow.com/questions/360201/how-do-i-kill-background-processes-jobs-when-my-shell-script-exits

if ! which idf.py ; then
  export IDF_TARGET=esp32s3
  deactivate 2> /dev/null || true
  . ../../esp/idf5.5/export.sh
fi


idf.py build
PYTHONPATH=./ python3 etc/ota.py

exit 0

#  python3 -m http.server 9000
