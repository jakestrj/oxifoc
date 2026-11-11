#!/usr/bin/env bash
set -euo pipefail

# Forward all args from Cargo (path to ELF and any filters)
# Run probe-rs and strip the leading "[defmt] " channel tag from each line.
# Preserve ANSI colors by running probe-rs in a pseudo-TTY using `script` if available.

CMD=(probe-rs run --chip STM32G474RETx "$@")

if [[ "$OSTYPE" == "darwin"* || "$OSTYPE" == *"bsd"* ]] && command -v script >/dev/null 2>&1; then
  # macOS / BSD script: -F flushes output, command and arguments follow output file
  script -q -F /dev/null "${CMD[@]}" \
    | awk '{ sub(/^\[defmt\][[:space:]]*/, ""); print; fflush(); }'
elif command -v script >/dev/null 2>&1; then
  # Linux (util-linux script): -f flushes output, -c passes command string
  script -q -f -c "${CMD[*]}" /dev/null \
    | awk '{ sub(/^\[defmt\][[:space:]]*/, ""); print; fflush(); }'
else
  # Fallback: force colors in some CLIs using CLICOLOR_FORCE
  CLICOLOR_FORCE=1 "${CMD[@]}" \
    | awk '{ sub(/^\[defmt\][[:space:]]*/, ""); print; fflush(); }'
fi
