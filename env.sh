#!/usr/bin/env bash

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
export STARTOUCH_SDK="$SCRIPT_DIR"
export PATH="/usr/local/bin:${PATH}"
export LD_LIBRARY_PATH="/usr/local/lib:$STARTOUCH_SDK/src${LD_LIBRARY_PATH:+:$LD_LIBRARY_PATH}"
export PYTHONPATH="$STARTOUCH_SDK/interface_py${PYTHONPATH:+:$PYTHONPATH}"
