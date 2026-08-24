#!/usr/bin/env bash
set -euo pipefail

SDK_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
PYTHON_BIN="${PYTHON_BIN:-/home/lumos/miniforge3/envs/lumostouch/bin/python}"
fail() {
    echo "verify_install: FAIL: $*" >&2
    exit 1
}

[[ -x "$PYTHON_BIN" ]] || fail "Python not found: $PYTHON_BIN"
[[ -f "$SDK_ROOT/src/config/robot_kinematics.yaml" ]] || fail "robot config missing"
for file in permutationMatrix.csv pi_b.csv pi_fr.csv; do
    [[ -f "$SDK_ROOT/src/param_csv_gripper/$file" ]] || fail "gripper parameter missing: $file"
done

library_hashes=()
case "$(uname -m)" in
    aarch64|arm64)
        runtime_libraries=(
            "$SDK_ROOT/src/libstartouch.so"
            "$SDK_ROOT/src/libstartouch.so.arm64"
        )
        ;;
    *)
        runtime_libraries=(
            "$SDK_ROOT/src/libstartouch.so"
            "$SDK_ROOT/src/libstartouch.so.20"
            "$SDK_ROOT/src/libstartouch.so.22"
            "$SDK_ROOT/src/libstartouch.so.24"
        )
        ;;
esac
for library in "${runtime_libraries[@]}"; do
    [[ -f "$library" ]] || fail "library missing: $library"
    actual_sha="$(sha256sum "$library" | awk '{print $1}')"
    library_hashes+=("$library=$actual_sha")
    nm -D -C "$library" | grep 'ArmController::setGripperDistanceEffort' >/dev/null || \
        fail "$library does not contain the 0.1.8 gripper force-control symbols"
done

module_path="$(
    STARTOUCH_SDK_ROOT="$SDK_ROOT" \
    LD_LIBRARY_PATH="$SDK_ROOT/src${LD_LIBRARY_PATH:+:$LD_LIBRARY_PATH}" \
    PYTHONPATH="$SDK_ROOT/interface_py${PYTHONPATH:+:$PYTHONPATH}" \
    "$PYTHON_BIN" - <<'PY'
import startouch
import startouchclass

assert startouch.__version__ == "0.1.8", startouch.__version__
assert startouchclass.__version__ == "0.1.8", startouchclass.__version__
assert hasattr(startouch.ArmController, "setGripperDistanceEffort")
assert hasattr(startouch.ArmController, "setGripperPositionEffort")
assert hasattr(startouch.ArmController, "get_gripper_state")
print(startouch.__file__)
PY
)"
[[ -f "$module_path" ]] || fail "imported extension missing: $module_path"
case "$module_path" in
    "$SDK_ROOT"/interface_py/*) ;;
    *) fail "imported extension is outside this SDK checkout: $module_path" ;;
esac

ldd_output="$(LD_LIBRARY_PATH="$SDK_ROOT/src${LD_LIBRARY_PATH:+:$LD_LIBRARY_PATH}" ldd "$module_path")"
if grep -q 'not found' <<<"$ldd_output"; then
    echo "$ldd_output" >&2
    fail "extension has unresolved shared libraries"
fi
grep -q 'libstartouch.so' <<<"$ldd_output" || fail "extension does not link libstartouch.so"

"$PYTHON_BIN" -m pip check
echo "verify_install: PASS"
echo "  Python: $PYTHON_BIN"
echo "  extension: $module_path"
printf '  library: %s\n' "${library_hashes[@]}"
