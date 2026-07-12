export STARTOUCH_SDK="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
export STARTOUCH_SDK_ROOT="$STARTOUCH_SDK"
export STARTOUCH_CONFIG_DIR="$STARTOUCH_SDK/src/config"
export STARTOUCH_PARAM_DIR="$STARTOUCH_SDK/src/param_csv_gripper"
export PATH="/home/lumos/bin:$PATH"

if [ -f "$HOME/miniconda3/etc/profile.d/conda.sh" ]; then
    # shellcheck source=/dev/null
    source "$HOME/miniconda3/etc/profile.d/conda.sh"
elif [ -f "$HOME/miniforge3/etc/profile.d/conda.sh" ]; then
    # shellcheck source=/dev/null
    source "$HOME/miniforge3/etc/profile.d/conda.sh"
elif [ -n "${CONDA_EXE:-}" ]; then
    # shellcheck source=/dev/null
    source "$(dirname "$CONDA_EXE")/../etc/profile.d/conda.sh"
else
    echo "env.sh: conda not found. Install miniconda3/miniforge3 or set CONDA_EXE." >&2
    return 1 2>/dev/null || exit 1
fi

conda activate "${STARTOUCH_CONDA_ENV:-lumostouch}"

# libstartouch.so lives under src/. Bundled deps/ may target newer glibc (e.g. 2.35)
# and break Ubuntu 20.04 / glibc 2.31 boards; opt in explicitly when needed.
export LD_LIBRARY_PATH="$STARTOUCH_SDK/src${LD_LIBRARY_PATH:+:$LD_LIBRARY_PATH}"
if [ "${STARTOUCH_USE_BUNDLED_DEPS:-0}" = "1" ] && [ -d "$STARTOUCH_SDK/deps" ]; then
    export LD_LIBRARY_PATH="$STARTOUCH_SDK/deps:$LD_LIBRARY_PATH"
fi
export PYTHONPATH="$STARTOUCH_SDK/interface_py${PYTHONPATH:+:$PYTHONPATH}"

_machine="$(uname -m)"
case "$_machine" in
    aarch64|arm64)
        _conda_compiler_pkg="gcc_linux-aarch64 gxx_linux-aarch64"
        _conda_gcc_candidates=(
            "$CONDA_PREFIX/bin/aarch64-conda-linux-gnu-gcc"
            "$CONDA_PREFIX/bin/gcc"
        )
        _conda_gxx_candidates=(
            "$CONDA_PREFIX/bin/aarch64-conda-linux-gnu-g++"
            "$CONDA_PREFIX/bin/g++"
        )
        ;;
    x86_64|amd64)
        _conda_compiler_pkg="gcc_linux-64 gxx_linux-64"
        _conda_gcc_candidates=(
            "$CONDA_PREFIX/bin/x86_64-conda-linux-gnu-gcc"
            "$CONDA_PREFIX/bin/gcc"
        )
        _conda_gxx_candidates=(
            "$CONDA_PREFIX/bin/x86_64-conda-linux-gnu-g++"
            "$CONDA_PREFIX/bin/g++"
        )
        ;;
    *)
        _conda_compiler_pkg="<platform-specific conda compiler package>"
        _conda_gcc_candidates=("$CONDA_PREFIX/bin/gcc")
        _conda_gxx_candidates=("$CONDA_PREFIX/bin/g++")
        ;;
esac

_cc=""
_cxx=""
for _candidate in "${_conda_gcc_candidates[@]}"; do
    if [ -x "$_candidate" ]; then
        _cc="$_candidate"
        break
    fi
done
for _candidate in "${_conda_gxx_candidates[@]}"; do
    if [ -x "$_candidate" ]; then
        _cxx="$_candidate"
        break
    fi
done

if [ -n "$_cc" ] && [ -n "$_cxx" ]; then
    export CC="$_cc"
    export CXX="$_cxx"
elif command -v gcc >/dev/null 2>&1 && command -v g++ >/dev/null 2>&1; then
    export CC="$(command -v gcc)"
    export CXX="$(command -v g++)"
else
    echo "env.sh: no C/C++ compiler found for $_machine." >&2
    echo "  Install conda compilers: conda install -y -c conda-forge $_conda_compiler_pkg" >&2
    echo "  Or install system tools: sudo apt-get install -y build-essential" >&2
    return 1 2>/dev/null || exit 1
fi

# Conda activate scripts may leave x86_64 flags (e.g. -march=nocona) on ARM hosts.
unset CFLAGS CXXFLAGS CPPFLAGS LDFLAGS
unset CMAKE_C_FLAGS CMAKE_CXX_FLAGS CMAKE_EXE_LINKER_FLAGS CMAKE_SHARED_LINKER_FLAGS

if command -v ninja >/dev/null 2>&1; then
    export CMAKE_GENERATOR=Ninja
else
    unset CMAKE_GENERATOR
fi

unset _machine _conda_compiler_pkg _conda_gcc_candidates _conda_gxx_candidates _cc _cxx _candidate
