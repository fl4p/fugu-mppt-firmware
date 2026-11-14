#!/bin/bash
set -e

# ./ota.sh -n | -m <hostname-regex> [more etc/ota.py args]; an unscoped run would update every discovered device
case " $* " in
  *" -n "* | *" --dry-run "* | *" -m "* | *" --match "* | *" --match="*) ;;
  *) echo "usage: $0 -n | -m <hostname-regex> [etc/ota.py args] (refusing an unscoped OTA)" >&2; exit 2 ;;
esac

# etc/ota.py serves build/ on :9000 itself (ensure_http_server) when it pushes
trap "trap - SIGTERM && kill -- -$$" SIGINT SIGTERM EXIT
# ^https://stackoverflow.com/questions/360201/how-do-i-kill-background-processes-jobs-when-my-shell-script-exits

if ! which idf.py ; then
  . ./idf-export.sh esp32s3
fi


idf.py build
PYTHONPATH=./ python3 etc/ota.py "$@"

exit 0
