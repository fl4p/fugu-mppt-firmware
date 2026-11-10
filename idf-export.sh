# Source me (bash or zsh): `. ./idf-export.sh`. Works from the main checkout and any git worktree.
#
# ESP-IDF location, first match wins:
#   $FUGU_IDF_PATH                      explicit IDF directory
#   $FUGU_ESP_DIR/$FUGU_IDF             directory holding the IDF versions
#   <main checkout>/../../esp/$FUGU_IDF (default layout, resolved via git, so worktrees find it too)
#   $IDF_PATH                           an IDF sourced earlier (only without FUGU_IDF)
# FUGU_IDF=idf5.5 selects ESP-IDF 5.5 (default idf6.0.3); use a separate build dir + sdkconfig for it:
#   idf.py -B build-idf55 -DSDKCONFIG=build-idf55/sdkconfig build
# An ESPPORT already set is kept; otherwise the first USB serial port is picked (pass -p with 2+ boards).

if [ -n "${ZSH_VERSION:-}" ]; then
    eval '_fugu_src=${(%):-%x}'
else
    _fugu_src=${BASH_SOURCE[0]:-$PWD/idf-export.sh}
fi
_fugu_root=$(cd "$(dirname "$_fugu_src")" && pwd)
_fugu_main=$(git -C "$_fugu_root" rev-parse --path-format=absolute --git-common-dir 2>/dev/null)
_fugu_main=${_fugu_main:+$(dirname "$_fugu_main")}
_fugu_main=${_fugu_main:-$_fugu_root}

_fugu_idf=
_fugu_prev=${IDF_PATH:-}
[ -n "${FUGU_IDF:-}" ] && _fugu_prev=
for _d in "${FUGU_IDF_PATH:-}" "${FUGU_ESP_DIR:+$FUGU_ESP_DIR/${FUGU_IDF:-idf6.0.3}}" \
          "$_fugu_main/../../esp/${FUGU_IDF:-idf6.0.3}" "$_fugu_prev"; do
    if [ -n "$_d" ] && [ -f "$_d/export.sh" ]; then
        _fugu_idf=$(cd "$_d" && pwd)
        break
    fi
done

if [ -z "$_fugu_idf" ]; then
    echo "idf-export.sh: ESP-IDF '${FUGU_IDF:-idf6.0.3}' not found in $_fugu_main/../../esp;" \
         "set FUGU_IDF_PATH=<idf dir> or FUGU_ESP_DIR=<dir with idf versions>" >&2
else
    export IDF_TARGET=esp32s3
    deactivate 2>/dev/null || true
    . "$_fugu_idf/export.sh"

    if [ -z "$(ls -A "$_fugu_root/components/Adafruit_BusIO" 2>/dev/null)" ]; then
        echo "idf-export.sh: submodules are not checked out, the build will fail. Run:" >&2
        echo "  git -C '$_fugu_root' submodule update --init --recursive" >&2
    fi

    if [ -z "${ESPPORT:-}" ]; then
        ESPPORT=$(find /dev -maxdepth 1 \( -name 'cu.usbmodem*' -o -name 'cu.usbserial*' \) 2>/dev/null | sort | head -n1)
        [ -n "$ESPPORT" ] && export ESPPORT
    fi
    echo "IDF: $_fugu_idf  port: ${ESPPORT:-none}"
fi
unset _fugu_src _fugu_root _fugu_main _fugu_idf _fugu_prev _d
