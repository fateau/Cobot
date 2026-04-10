#!/bin/bash
# run_full_test.sh — 一鍵全面測試 ShmAPI shared memory
set -e

DIR="$(cd "$(dirname "$0")" && pwd)"
cd "$DIR"

echo "===== 清理環境 ====="
rm -f /dev/shm/DataSpace /tmp/shm_writer_ready /tmp/shm_reader_done

echo "===== 啟動 Writer ====="
LD_LIBRARY_PATH="$DIR" "$DIR/shm_test_writer" &
WPID=$!

# 等待 Writer 就緒
for i in $(seq 1 30); do
    [ -f /tmp/shm_writer_ready ] && break
    sleep 0.2
done

echo ""
echo "===== 啟動 Reader ====="
"$DIR/shm_test_reader"
RCODE=$?

# 等 Writer 自然結束
wait $WPID 2>/dev/null || true

rm -f /tmp/shm_writer_ready /tmp/shm_reader_done

exit $RCODE
