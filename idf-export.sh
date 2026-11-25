# Source me (bash or zsh): `. ./idf-export.sh [target]`. Works from the main checkout and any git worktree.
#
# IDF_TARGET: the target argument (`. ./idf-export.sh esp32`), else an IDF_TARGET already exported, else esp32s3.
# (`IDF_TARGET=esp32 . ./idf-export.sh` does not stick: a prefix assignment on `.` is undone when it returns.)
#
# ESP-IDF location, first match wins:
#   $FUGU_IDF_PATH                      explicit IDF directory
#   $FUGU_ESP_DIR/$FUGU_IDF             directory holding the IDF versions
#   <main checkout>/../../esp/$FUGU_IDF (default layout, resolved via git, so worktrees find it too)
#   $IDF_PATH                           an IDF sourced earlier (only without FUGU_IDF)
# FUGU_IDF selects another IDF directory name (default idf6.0.3).
# Enables ccache for idf.py when installed (IDF_CCACHE_ENABLE=0 to opt out).
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
    case "${1:-}" in esp32*) IDF_TARGET=$1 ;; esac
    export IDF_TARGET=${IDF_TARGET:-esp32s3}
    deactivate 2>/dev/null || true
    . "$_fugu_idf/export.sh"
    # idf.py uses ccache only when asked; IDF_CCACHE_ENABLE=0 turns it off again
    command -v ccache >/dev/null 2>&1 && export IDF_CCACHE_ENABLE=${IDF_CCACHE_ENABLE:-1}

    # Missing or half-checked-out submodules (an interrupted first update leaves files deleted).
    _fugu_bad=$(git -C "$_fugu_root" config -f .gitmodules --get-regexp '\.path$' 2>/dev/null |
        while read -r _k _p; do
            [ -n "$(git -C "$_fugu_root/$_p" ls-files 2>/dev/null | head -n1)" ] &&
                [ -z "$(git -C "$_fugu_root/$_p" ls-files --deleted 2>/dev/null | head -n1)" ] ||
                printf ' %s' "$_p"
        done)
    if [ -n "$_fugu_bad" ]; then
        echo "idf-export.sh: submodules missing or incomplete:$_fugu_bad; the build will fail. Run:" >&2
        echo "  git -C '$_fugu_root' -c protocol.file.allow=always submodule update --init --recursive --force" >&2
    fi
    unset _fugu_bad

    if [ -z "${ESPPORT:-}" ]; then
        ESPPORT=$(find /dev -maxdepth 1 \( -name 'cu.usbmodem*' -o -name 'cu.usbserial*' \) 2>/dev/null | sort | head -n1)
        [ -n "$ESPPORT" ] && export ESPPORT
    fi
    echo "IDF: $_fugu_idf  target: $IDF_TARGET  port: ${ESPPORT:-none}"
fi
unset _fugu_src _fugu_root _fugu_main _fugu_idf _fugu_prev _d
