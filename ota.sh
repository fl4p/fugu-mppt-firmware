#!/bin/bash
set -e

idf.py build
PYTHONPATH=./ python3 etc/ota.py