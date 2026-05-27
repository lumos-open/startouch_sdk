export STARTOUCH_SDK=/home/lumos/openpi_client/startouch_sdk
export PATH=/home/lumos/bin:$PATH
source /home/lumos/miniforge3/etc/profile.d/conda.sh
conda activate LumosTouch
export LD_LIBRARY_PATH="$STARTOUCH_SDK/src:$STARTOUCH_SDK/deps${LD_LIBRARY_PATH:+:$LD_LIBRARY_PATH}"
export PYTHONPATH="$STARTOUCH_SDK/interface_py${PYTHONPATH:+:$PYTHONPATH}"
export CC="$CONDA_PREFIX/bin/gcc"
export CXX="$CONDA_PREFIX/bin/g++"
export CMAKE_GENERATOR=Ninja
