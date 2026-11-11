#!/usr/bin/env bash
set -euo pipefail

# Probe-run helper for STM32F405 (Simple FOCer 2 / VESC hardware)
# Adjust chip name if you use a different package.
CMD=(probe-rs run --chip STM32F405RGTx "$@")

if [[ "$OSTYPE" == "darwin"* || "$OSTYPE" == *"bsd"* ]] && command -v script >/dev/null 2>&1; then
  # macOS / BSD script: -F flushes output, command and arguments follow output file
  script -q -F /dev/null "${CMD[@]}" \
    | awk '{ sub(/^\[defmt\][[:space:]]*/, ""); print; fflush(); }'
elif command -v script >/dev/null 2>&1; then
  # Linux (util-linux script): -f flushes output, -c passes command string
  script -q -f -c "${CMD[*]}" /dev/null \
    | awk '{ sub(/^\[defmt\][[:space:]]*/, ""); print; fflush(); }'
else
  CLICOLOR_FORCE=1 "${CMD[@]}" \
    | awk '{ sub(/^\[defmt\][[:space:]]*/, ""); print; fflush(); }'
fi
