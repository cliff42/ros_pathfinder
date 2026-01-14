#!/usr/bin/env bash
set -euo pipefail

source "$(dirname "$0")/.venv/bin/activate"
source "$(dirname "$0")/install/setup.bash"

exec "$@"
