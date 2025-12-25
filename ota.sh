#!/bin/bash
set -e

(python3 -m http.server 9000 2> /dev/null || true) &

if ! which idf.py ; then
  export IDF_TARGET=esp32s3
  deactivate 2> /dev/null || true
  . ../../esp/idf5.5/export.sh
fi


idf.py build
PYTHONPATH=./ python3 etc/ota.py

#  python3 -m http.server 9000
