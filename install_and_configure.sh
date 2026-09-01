#!/usr/bin/env bash
set -euo pipefail

# One-shot installer/runtime adapter for the StarTouch SDK.
#
# Default usage on the UniTeleop controller:
#   ./install_and_configure.sh
#
# Optional real-hardware CAN setup:
#   ./install_and_configure.sh --setup-can

SDK_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
CONDA_ENV="${STARTOUCH_CONDA_ENV:-${ARM_CONDA_ENV:-lumosteleop}}"
CONDA_SH="${ARM_CONDA_SH:-}"
SETUP_CAN=0
CAN_LEFT="${ARM_CAN_LEFT:-can0}"
CAN_RIGHT="${ARM_CAN_RIGHT:-can1}"
CAN_BITRATE="${ARM_CAN_BITRATE:-1000000}"
CAN_TX_QUEUE_LEN="${ARM_CAN_TX_QUEUE_LEN:-1000}"
CAN_RESTART_MS="${ARM_CAN_RESTART_MS:-100}"

usage() {
    cat <<'EOF'
Usage: install_and_configure.sh [options]

Options:
  --conda-env NAME     Conda environment to install into (default: lumosteleop)
  --conda-sh PATH      Path to conda.sh
  --setup-can          Configure and bring up both SocketCAN interfaces
  --can-left NAME      Left-arm CAN interface (default: can0)
  --can-right NAME     Right-arm CAN interface (default: can1)
  --bitrate RATE       CAN bitrate (default: 1000000)
  -h, --help           Show this help

The script installs the SDK with `python -m pip install .`, writes conda
activate/deactivate hooks for the SDK paths, validates configuration and
calibration files, and runs the SDK installation verifier.
EOF
}

while (($# > 0)); do
    case "$1" in
        --conda-env)
            [[ $# -ge 2 ]] || { echo "--conda-env requires a value" >&2; exit 2; }
            CONDA_ENV="$2"
            shift 2
            ;;
        --conda-sh)
            [[ $# -ge 2 ]] || { echo "--conda-sh requires a value" >&2; exit 2; }
            CONDA_SH="$2"
            shift 2
            ;;
        --setup-can)
            SETUP_CAN=1
            shift
            ;;
        --can-left)
            [[ $# -ge 2 ]] || { echo "--can-left requires a value" >&2; exit 2; }
            CAN_LEFT="$2"
            shift 2
            ;;
        --can-right)
            [[ $# -ge 2 ]] || { echo "--can-right requires a value" >&2; exit 2; }
            CAN_RIGHT="$2"
            shift 2
            ;;
        --bitrate)
            [[ $# -ge 2 ]] || { echo "--bitrate requires a value" >&2; exit 2; }
            CAN_BITRATE="$2"
            shift 2
            ;;
        -h|--help)
            usage
            exit 0
            ;;
        *)
            echo "Unknown option: $1" >&2
            usage >&2
            exit 2
            ;;
    esac
done

if [[ -z "$CONDA_SH" ]]; then
    for candidate in \
        "$HOME/miniforge3/etc/profile.d/conda.sh" \
        "$HOME/miniconda3/etc/profile.d/conda.sh"; do
        if [[ -f "$candidate" ]]; then
            CONDA_SH="$candidate"
            break
        fi
    done
fi

[[ -n "$CONDA_SH" && -f "$CONDA_SH" ]] || {
    echo "conda.sh not found; pass --conda-sh PATH or set ARM_CONDA_SH." >&2
    exit 1
}

# shellcheck source=/dev/null
source "$CONDA_SH"
conda activate "$CONDA_ENV"

PYTHON_BIN="$(command -v python)"
[[ -n "$PYTHON_BIN" ]] || { echo "Python is unavailable in conda env: $CONDA_ENV" >&2; exit 1; }

for command_name in cmake c++ ldd nm; do
    command -v "$command_name" >/dev/null 2>&1 || {
        echo "Required system command is missing: $command_name" >&2
        echo "Install the SDK system dependencies described in README_INSTALL.md." >&2
        exit 1
    }
done

CONFIG_DIR="$SDK_ROOT/src/config"
PARAM_DIR="$SDK_ROOT/src/param_csv_gripper"
INTERFACE_DIR="$SDK_ROOT/interface_py"
LIB_DIR="$SDK_ROOT/src"

required_files=(
    "$CONFIG_DIR/robot_kinematics.yaml"
    "$CONFIG_DIR/FastTouchV2.SLDASM.urdf"
    "$PARAM_DIR/permutationMatrix.csv"
    "$PARAM_DIR/pi_b.csv"
    "$PARAM_DIR/pi_fr.csv"
)
for required_file in "${required_files[@]}"; do
    [[ -f "$required_file" ]] || {
        echo "Required SDK resource is missing: $required_file" >&2
        exit 1
    }
done

case "$(uname -m)" in
    aarch64|arm64)
        PREBUILT_LIB="$SDK_ROOT/src/libstartouch.so.arm64"
        ;;
    x86_64|amd64)
        OS_VERSION=""
        if [[ -r /etc/os-release ]]; then
            # shellcheck source=/dev/null
            source /etc/os-release
            OS_VERSION="${VERSION_ID:-}"
        fi
        OS_MAJOR="${OS_VERSION%%.*}"
        if [[ "$OS_MAJOR" =~ ^[0-9]+$ ]] && ((OS_MAJOR >= 24)); then
            PREBUILT_LIB="$SDK_ROOT/src/libstartouch.so.24"
        elif [[ "$OS_MAJOR" =~ ^[0-9]+$ ]] && ((OS_MAJOR >= 22)); then
            PREBUILT_LIB="$SDK_ROOT/src/libstartouch.so.22"
        elif [[ "$OS_MAJOR" =~ ^[0-9]+$ ]] && ((OS_MAJOR >= 20)); then
            PREBUILT_LIB="$SDK_ROOT/src/libstartouch.so.20"
        else
            echo "Unsupported x86 Linux VERSION_ID: ${OS_VERSION:-unknown}" >&2
            exit 1
        fi
        ;;
    *)
        echo "Unsupported architecture: $(uname -m)" >&2
        exit 1
        ;;
esac
[[ -n "$PREBUILT_LIB" && -f "$PREBUILT_LIB" ]] || {
    echo "No matching prebuilt libstartouch library was found." >&2
    exit 1
}

echo "=== StarTouch SDK adaptation ==="
echo "SDK root : $SDK_ROOT"
echo "Conda env: $CONDA_ENV"
echo "Python   : $PYTHON_BIN"
echo "Runtime  : $PREBUILT_LIB"

LEGACY_PARAM_LINK="$SDK_ROOT/param_csv_gripper"
if [[ -L "$LEGACY_PARAM_LINK" ]]; then
    [[ "$(readlink -f "$LEGACY_PARAM_LINK")" == "$(readlink -f "$PARAM_DIR")" ]] || {
        echo "Existing parameter symlink points elsewhere: $LEGACY_PARAM_LINK" >&2
        exit 1
    }
elif [[ -e "$LEGACY_PARAM_LINK" ]]; then
    echo "Existing legacy parameter path will not be overwritten: $LEGACY_PARAM_LINK" >&2
    echo "Move it aside or make it point to: $PARAM_DIR" >&2
    exit 1
else
    # startouchclass.py still supports the historical SDK-root-relative path.
    # Keep that API working while STARTOUCH_PARAM_DIR remains the canonical path.
    ln -s "src/param_csv_gripper" "$LEGACY_PARAM_LINK"
fi

echo "=== Installing Python package and runtime dependencies ==="
(
    cd "$SDK_ROOT"
    "$PYTHON_BIN" -m pip install .
)

ACTIVATE_DIR="$CONDA_PREFIX/etc/conda/activate.d"
DEACTIVATE_DIR="$CONDA_PREFIX/etc/conda/deactivate.d"
ACTIVATE_HOOK="$ACTIVATE_DIR/startouch_sdk.sh"
DEACTIVATE_HOOK="$DEACTIVATE_DIR/startouch_sdk.sh"
mkdir -p "$ACTIVATE_DIR" "$DEACTIVATE_DIR"

write_activate_hook() {
    {
        echo '# Generated by startouch_sdk/install_and_configure.sh.'
        printf 'export STARTOUCH_SDK_ROOT=%q\n' "$SDK_ROOT"
        printf 'export STARTOUCH_CONFIG_DIR=%q\n' "$CONFIG_DIR"
        printf 'export STARTOUCH_PARAM_DIR=%q\n' "$PARAM_DIR"
        printf 'export ARM_STARTOUCH_SDK_ROOT=%q\n' "$SDK_ROOT"
        printf 'export ARM_STARTOUCH_SDK_INTERFACE_PY=%q\n' "$INTERFACE_DIR"
        printf 'export ARM_CAN_LEFT=%q\n' "$CAN_LEFT"
        printf 'export ARM_CAN_RIGHT=%q\n' "$CAN_RIGHT"
        cat <<'EOF'
_startouch_prepend_unique() {
    local variable_name="$1"
    local entry="$2"
    local current_value="${!variable_name-}"
    case ":$current_value:" in
        *":$entry:"*) ;;
        *)
            if [[ -n "$current_value" ]]; then
                printf -v "$variable_name" '%s:%s' "$entry" "$current_value"
            else
                printf -v "$variable_name" '%s' "$entry"
            fi
            export "$variable_name"
            ;;
    esac
}
EOF
        printf '_startouch_prepend_unique PYTHONPATH %q\n' "$INTERFACE_DIR"
        printf '_startouch_prepend_unique LD_LIBRARY_PATH %q\n' "$LIB_DIR"
        echo 'unset -f _startouch_prepend_unique'
    } >"$ACTIVATE_HOOK"
    chmod 0644 "$ACTIVATE_HOOK"
}

write_deactivate_hook() {
    {
        echo '# Generated by startouch_sdk/install_and_configure.sh.'
        cat <<'EOF'
_startouch_remove_path_entry() {
    local variable_name="$1"
    local remove_entry="$2"
    local current_value="${!variable_name-}"
    local entry
    local rebuilt=""
    local entries=()
    IFS=':' read -r -a entries <<<"$current_value"
    for entry in "${entries[@]}"; do
        [[ -z "$entry" || "$entry" == "$remove_entry" ]] && continue
        rebuilt="${rebuilt:+$rebuilt:}$entry"
    done
    printf -v "$variable_name" '%s' "$rebuilt"
    export "$variable_name"
}
EOF
        printf '_startouch_remove_path_entry PYTHONPATH %q\n' "$INTERFACE_DIR"
        printf '_startouch_remove_path_entry LD_LIBRARY_PATH %q\n' "$LIB_DIR"
        printf 'if [[ "${STARTOUCH_SDK_ROOT:-}" == %q ]]; then\n' "$SDK_ROOT"
        echo '    unset STARTOUCH_SDK_ROOT STARTOUCH_CONFIG_DIR STARTOUCH_PARAM_DIR'
        echo '    unset ARM_STARTOUCH_SDK_ROOT ARM_STARTOUCH_SDK_INTERFACE_PY'
        echo '    unset ARM_CAN_LEFT ARM_CAN_RIGHT'
        echo 'fi'
        echo 'unset -f _startouch_remove_path_entry'
    } >"$DEACTIVATE_HOOK"
    chmod 0644 "$DEACTIVATE_HOOK"
}

write_activate_hook
write_deactivate_hook

# Apply the hook to this installer process immediately. Future shells receive
# the same variables whenever this conda environment is activated.
# shellcheck source=/dev/null
source "$ACTIVATE_HOOK"

if [[ "$SETUP_CAN" == "1" ]]; then
    echo "=== Configuring SocketCAN ==="
    for can_interface in "$CAN_LEFT" "$CAN_RIGHT"; do
        if ! ip link show "$can_interface" >/dev/null 2>&1; then
            echo "CAN interface does not exist: $can_interface" >&2
            exit 1
        fi
        sudo ip link set "$can_interface" down || true
        sudo ip link set "$can_interface" type can \
            bitrate "$CAN_BITRATE" restart-ms "$CAN_RESTART_MS"
        sudo ip link set "$can_interface" txqueuelen "$CAN_TX_QUEUE_LEN"
        sudo ip link set "$can_interface" up
        ip -details link show "$can_interface"
    done
fi

SDK_VERSION="$(awk -F'"' '/^[[:space:]]*version[[:space:]]*=/{print $2; exit}' "$SDK_ROOT/pyproject.toml")"
[[ "$SDK_VERSION" =~ ^[0-9]+\.[0-9]+\.[0-9]+$ ]] || {
    echo "Unable to read a valid SDK version from: $SDK_ROOT/pyproject.toml" >&2
    exit 1
}

version_at_least() {
    local current="$1"
    local minimum="$2"
    local current_major current_minor current_patch
    local minimum_major minimum_minor minimum_patch
    IFS='.' read -r current_major current_minor current_patch <<<"$current"
    IFS='.' read -r minimum_major minimum_minor minimum_patch <<<"$minimum"
    ((
        current_major > minimum_major ||
        (current_major == minimum_major && current_minor > minimum_minor) ||
        (current_major == minimum_major && current_minor == minimum_minor && current_patch >= minimum_patch)
    ))
}

if version_at_least "$SDK_VERSION" "0.1.8"; then
    echo "=== Verifying SDK installation (version $SDK_VERSION) ==="
    PYTHON_BIN="$PYTHON_BIN" "$SDK_ROOT/verify_install.sh"
else
    echo "=== Skipping verify_install.sh ==="
    echo "SDK version $SDK_VERSION is lower than 0.1.8; the verifier is specific to the 0.1.8 force-control release."
fi

echo "=== Adaptation complete ==="
echo "Conda activation hook: $ACTIVATE_HOOK"
echo "Configuration directory: $STARTOUCH_CONFIG_DIR"
echo "Calibration directory  : $STARTOUCH_PARAM_DIR"
echo "For a new shell, run: conda activate $CONDA_ENV"
if [[ "$SETUP_CAN" != "1" ]]; then
    echo "CAN was not changed. Add --setup-can when hardware setup is required."
fi
