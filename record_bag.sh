#!/bin/bash
# ============================================================
# record_bag.sh — ros2 bag 录制脚本
# ============================================================
# 用法:
#   ./record_bag.sh                    # 录制全部话题
#   ./record_bag.sh -o my_session      # 指定输出文件名
#   ./record_bag.sh -t scan,detections # 只录制指定话题
#   ./record_bag.sh -c                 # 压缩模式 (仅图像用压缩话题)
#   ./record_bag.sh -l                 # 轻量模式 (不录图像)
#
# 录制后离线查看:
#   ros2 bag play <bag目录>
#   ros2 bag info <bag目录>
# ============================================================

set -e

# ---- 默认配置 ----
OUTPUT_DIR="${HOME}/bags"
BAG_NAME="record_$(date +%Y%m%d_%H%M%S)"

# ---- 话题分组 ----
# 核心话题 (无人机状态 + 检测结果)
CORE_TOPICS=(
    /fmu/out/vehicle_local_position_v1
    /fmu/out/vehicle_attitude
    /detections
    /detection/camera_coordinates
    /detection/world_coordinates_raw
    /detection/world_coordinates
)

# SLAM 话题
SLAM_TOPICS=(
    /scan
    /imu
    /tf
    /tf_static
)

# 图像话题 (数据量大，按需录制)
IMAGE_TOPICS=(
    /image_raw
    /image_raw/compressed
)

# ---- 参数解析 ----
COMPRESSED=false
LIGHT=false
CUSTOM_TOPICS=""

usage() {
    echo "用法: $0 [选项]"
    echo ""
    echo "选项:"
    echo "  -o NAME    输出文件名 (默认: record_YYYYMMDD_HHMMSS)"
    echo "  -d DIR     输出目录 (默认: ~/bags)"
    echo "  -t TOPICS  指定话题，逗号分隔 (覆盖默认)"
    echo "  -c         压缩图像模式 (录制 compressed 而非 raw，省空间)"
    echo "  -l         轻量模式 (不录图像，只录核心+SLAM话题)"
    echo "  -a         全部话题 (含原始图像)"
    echo "  -h         显示帮助"
    echo ""
    echo "示例:"
    echo "  $0                         # 录制默认话题 (含压缩图像)"
    echo "  $0 -l                      # 轻量模式，不录图像"
    echo "  $0 -a                      # 全部话题含原始图像"
    echo "  $0 -t /scan,/detections    # 只录指定话题"
    echo "  $0 -o test_flight          # 输出为 test_flight"
    exit 0
}

while getopts "o:d:t:clah" opt; do
    case $opt in
        o) BAG_NAME="$OPTARG" ;;
        d) OUTPUT_DIR="$OPTARG" ;;
        t) CUSTOM_TOPICS="$OPTARG" ;;
        c) COMPRESSED=true ;;
        l) LIGHT=true ;;
        a) LIGHT=false ; COMPRESSED=false ;;  # -a 覆盖 -l/-c
        h) usage ;;
        *) usage ;;
    esac
done

# ---- 构建话题列表 ----
if [ -n "$CUSTOM_TOPICS" ]; then
    # 自定义话题
    IFS=',' read -ra TOPICS <<< "$CUSTOM_TOPICS"
else
    TOPICS=("${CORE_TOPICS[@]}" "${SLAM_TOPICS[@]}")

    if $LIGHT; then
        echo "[info] 轻量模式: 跳过图像话题"
    elif $COMPRESSED; then
        # 只录压缩图像
        TOPICS+=("/image_raw/compressed")
        echo "[info] 压缩模式: 仅录制 /image_raw/compressed"
    else
        # 默认: 录压缩图像
        TOPICS+=("/image_raw/compressed")
        echo "[info] 默认模式: 录制压缩图像 (用 -a 录原始图像, -l 跳过图像)"
    fi
fi

# ---- 创建输出目录 ----
mkdir -p "$OUTPUT_DIR"
BAG_PATH="${OUTPUT_DIR}/${BAG_NAME}"

# ---- 显示录制信息 ----
echo "========================================"
echo " ros2 bag 录制"
echo "========================================"
echo " 输出路径: ${BAG_PATH}"
echo " 话题数量: ${#TOPICS[@]}"
echo " 话题列表:"
for t in "${TOPICS[@]}"; do
    echo "   - $t"
done
echo "========================================"
echo ""
echo "按 Ctrl+C 停止录制"
echo ""

# ---- 开始录制 ----
ros2 bag record -o "$BAG_PATH" "${TOPICS[@]}"

# ---- 录制完成 ----
echo ""
echo "录制完成!"
echo "查看信息: ros2 bag info ${BAG_PATH}"
echo "回放数据: ros2 bag play ${BAG_PATH}"
echo "导出CSV:  ros2 bag info ${BAG_PATH} -v  # 查看消息类型后自行解析"
