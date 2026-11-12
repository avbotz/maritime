#!/bin/bash

# Formats C with clang-format and devicetree with dts-linter, then checks C with checkpatch.

set -euo pipefail

CHECK_MODE=false

for arg in "$@"; do
  if [[ "$arg" == "--check" ]]; then
    CHECK_MODE=true
    break # Stop searching once found
  else
    echo "Usage: $0 [--check]" >&2
    exit 2
  fi
done

cd "$(dirname "$0")"
ZEPHYR_BASE="${ZEPHYR_BASE:-../zephyr}"

mapfile -t c_files < <(find src include -name '*.[ch]')
mapfile -t dts_files < <(find boards dts -name '*.dts' -o -name '*.dtsi' -o -name '*.overlay')

dts_args=()
for file in "${dts_files[@]}"; do
    dts_args+=(--file "$file")
done

dts_linter() {
    # The version Zephyr pins, installed by: npm --prefix "$ZEPHYR_BASE/scripts/ci" ci
    npx --prefix "$ZEPHYR_BASE/scripts/ci" --no dts-linter -- "$@"
}

status=0

if $CHECK_MODE; then
    clang-format --dry-run --Werror "${c_files[@]}" || status=1
    dts_linter --format "${dts_args[@]}" || status=1
else
    clang-format -i "${c_files[@]}"
    dts_linter --formatFixAll "${dts_args[@]}"
fi

# Settings come from .checkpatch.conf
"$ZEPHYR_BASE/scripts/checkpatch.pl" --no-tree --quiet \
    --typedefsfile "$ZEPHYR_BASE/scripts/checkpatch/typedefsfile" -f "${c_files[@]}" || status=1

exit $status
