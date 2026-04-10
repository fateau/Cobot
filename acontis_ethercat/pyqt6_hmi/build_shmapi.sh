#!/bin/bash
# Build libShmAPI.so for Linux (acontis EC-Master)
set -e

SCRIPT_DIR="$(cd "$(dirname "$0")" && pwd)"
SHM_SRC="${SCRIPT_DIR}/../ecatlibrary/ShmAPI"
SDK_INC="/home/user/COBOT/ethercat/steven_ecmaster/EC-Master-V3.2.2.99-Linux-x86_64Bit-Eval-Intelgbe/SDK/INC"

echo "Building libShmAPI.so ..."
g++ -shared -fPIC -O2 -o "${SCRIPT_DIR}/libShmAPI.so" \
    "${SHM_SRC}/ShmAPI.cpp" \
    -I"${SHM_SRC}/.." \
    -I"${SHM_SRC}/../RtxAppLayer" \
    -I"${SDK_INC}" \
    -I"${SDK_INC}/Linux" \
    -D__LINUX_GCC__ \
    -lrt -lpthread

echo "Done: ${SCRIPT_DIR}/libShmAPI.so"
ls -la "${SCRIPT_DIR}/libShmAPI.so"
