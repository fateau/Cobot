#!/bin/bash
# test_shm.sh — 一鍵測試 shared memory 通訊
set -e

LIBS_DIR="$(cd "$(dirname "$0")" && pwd)"
cd "$LIBS_DIR"

echo "============================="
echo "  Shared Memory 通訊測試"
echo "============================="

# 清除舊的共享記憶體 (如果存在)
rm -f /dev/shm/DataSpace 2>/dev/null || true

# 1) 啟動 Writer (背景)
echo ""
echo "[1] 啟動 Writer..."
LD_LIBRARY_PATH=. ./shm_writer &
WPID=$!
sleep 1

# 2) 執行 Reader
echo ""
echo "[2] 啟動 Reader..."
./shm_reader
READER_EXIT=$?

# 3) 等待 Writer 結束
kill $WPID 2>/dev/null || true
wait $WPID 2>/dev/null || true

echo ""
if [ $READER_EXIT -eq 0 ]; then
    echo "============================="
    echo "  全部測試通過！Shared Memory 通訊正常！"
    echo "============================="
else
    echo "============================="
    echo "  測試失敗！Shared Memory 通訊異常。"
    echo "============================="
fi

exit $READER_EXIT
