#!/bin/sh
# PVGIS 5.3 TMY for two generic public sites (city centres, not the converters' location). Usage: fetch_tmy.sh <empty dir>
set -e
cd "$1"
curl -sS -o tmy_lisbon.json 'https://re.jrc.ec.europa.eu/api/v5_3/tmy?lat=38.72&lon=-9.14&outputformat=json'
curl -sS -o tmy_evora.json 'https://re.jrc.ec.europa.eu/api/v5_3/tmy?lat=38.57&lon=-7.91&outputformat=json'
