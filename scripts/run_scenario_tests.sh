#!/bin/bash
# ================================================================
# 模拟场景测试脚本 —— 在容器内运行
# 用法:  docker exec -t <容器> bash /root/ws/scripts/run_scenario_tests.sh
# ================================================================
set -e

WS=/root/ws
cd $WS

echo "============================================"
echo "  宇树 Go2 Bridge 场景测试"
echo "============================================"
echo ""

# ---- source 环境 ----
source /opt/ros/humble/setup.bash
source $WS/src/unitree_ros2/cyclonedds_ws/install/setup.bash
source $WS/install/setup.bash

RESULTS_DIR=/tmp/scenario_tests
rm -rf $RESULTS_DIR
mkdir -p $RESULTS_DIR

run_scenario() {
    local name=$1
    local timeout_sec=$2
    local desc=$3
    local outfile="$RESULTS_DIR/${name}.log"

    echo ""
    echo "=== [$name] $desc ==="
    echo "    运行 ${timeout_sec}s ..."

    # 后台启动 launch（nohup 彻底脱离 TTY，避免卡死）
    nohup ros2 launch dog_nav_step56 closed_loop_demo.launch.py \
        use_bridge:=true scenario:=$name \
        > "$outfile" 2>&1 &
    local lpid=$!

    # 等待指定时间
    sleep $timeout_sec

    # 优雅结束
    kill -INT $lpid 2>/dev/null || true
    wait $lpid 2>/dev/null || true
    sleep 2

    # 确保所有残留进程被杀
    pkill -f "ros2 launch" 2>/dev/null || true
    sleep 1

    # ---- 分析结果 ----
    echo ""
    echo "--- 统计数据 ---"
    local total_lines=$(wc -l < "$outfile")
    local move_count=$(grep -c 'Move(1008)' "$outfile" || true)
    local damp_count=$(grep -c 'Damp(1001)' "$outfile" || true)
    local stop_count=$(grep -c 'StopMove(1003)' "$outfile" || true)
    local walk_count=$(grep -c 'api_id=1061' "$outfile" || true)
    local trot_count=$(grep -c 'api_id=1062' "$outfile" || true)
    local crawl_count=$(grep -c 'api_id=1063' "$outfile" || true)
    local speed_count=$(grep -c 'SpeedLevel' "$outfile" || true)
    local error_count=$(grep -ci 'error\|traceback\|exception' "$outfile" || true)

    echo "    总行数:     $total_lines"
    echo "    Move(1008): $move_count"
    echo "    Damp(1001): $damp_count"
    echo "    StopMove:   $stop_count"
    echo "    WALK:       $walk_count"
    echo "    TROT:       $trot_count"
    echo "    CRAWL:      $crawl_count"
    echo "    SpeedLevel: $speed_count"
    echo "    错误行:     $error_count"

    echo ""
    echo "--- 桥接输出 (最后 25 条) ---"
    grep '→' "$outfile" | tail -25 || echo "    (无桥接输出)"

    echo ""
    echo "--- 安全事件 ---"
    # 时间戳只取前段用于对比
    grep '→' "$outfile" | grep -E 'Damp|StopMove' | while read -r line; do
        # 提取时间戳（秒/微秒）
        ts=$(echo "$line" | grep -oP '\d+\.\d+' | head -1)
        echo "    t=${ts}s  $line" | sed 's/\[.*\]//g'
    done || echo "    (无安全事件)"

    echo ""
    echo "=== [$name] 完成 ==="
}

# ================================================================
# 场景 1: 蛇形巡航
# ================================================================
run_scenario "curved_normal" 15 \
    "蛇形路径，安全地形，验证 TROT 巡航 + 步态自动切换"

# ================================================================
# 场景 2: 地形恶化链
# ================================================================
run_scenario "terrain_escalation" 22 \
    "直线路径，4s/8s/12s 地形逐级恶化，验证 NORMAL→SLOW→PAUSE→STOP"

# ================================================================
# 场景 3: Z字巡逻 + 倾斜
# ================================================================
run_scenario "zigzag_with_tilt" 15 \
    "Z字折返路径，5-7s倾斜10度，验证大角度转向CRAWL + 倾斜SLOW"

# ================================================================
# 汇总
# ================================================================
echo ""
echo "============================================"
echo "  测试汇总"
echo "============================================"

for logfile in $RESULTS_DIR/*.log; do
    name=$(basename "$logfile" .log)
    errors=$(grep -ci 'error\|traceback\|exception' "$logfile" || true)
    apis=$(grep -c '→' "$logfile" || true)
    if [ "$errors" -gt 0 ]; then
        status="❌ 有错误"
    elif [ "$apis" -gt 0 ]; then
        status="✅ 通过 ($apis 条API调用)"
    else
        status="⚠️  无API输出"
    fi
    printf "  %-25s  %s\n" "$name" "$status"
done

echo ""
echo "详细日志: $RESULTS_DIR/"
echo "============================================"
echo "  全部完成"
echo "============================================"
