#!/bin/bash
set -e

# Documented in doc/manual/contributing/docker.md

if [ -f "/home/piconut-user/piconut/compose-entrypoint.local.sh" ]; then
    bash "/home/piconut-user/piconut/compose-entrypoint.local.sh"
fi

# preserves `command:` from compose / devcontainer
exec "$@"
