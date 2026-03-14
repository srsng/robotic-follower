#!/bin/bash
# 感知模块快速启动脚本

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
WORKSPACE_ROOT="$(cd "$SCRIPT_DIR/.." && pwd)"

# 加载 ROS2 环境
source /opt/ros/humble/setup.bash
source "$WORKSPACE_ROOT/install/setup.bash"

# 配置文件路径
VOTENET_CONFIG="$WORKSPACE_ROOT/src/perception/config/votenet_config.yaml"
DENSITY_CONFIG="$WORKSPACE_ROOT/src/perception/config/density_fusion_config.yaml"

# 显示使用说明
show_usage() {
    echo "用法: $0 [模型类型] [选项]"
    echo ""
    echo "模型类型:"
    echo "  votenet        - 使用 VoteNet 预训练模型"
    echo "  density        - 使用密度融合模型（模拟检测器）"
    echo ""
    echo "选项:"
    echo "  --no-viz       - 禁用可视化"
    echo ""
    echo "示例:"
    echo "  $0 votenet              # 启动 VoteNet 模型（带可视化）"
    echo "  $0 votenet --no-viz     # 启动 VoteNet 模型（无可视化）"
    echo "  $0 density              # 启动密度融合模型"
}

# 检查参数
if [ $# -eq 0 ]; then
    show_usage
    exit 1
fi

MODEL_TYPE=$1
ENABLE_VIZ="true"

# 解析选项
shift
while [[ $# -gt 0 ]]; do
    case $1 in
        --no-viz)
            ENABLE_VIZ="false"
            shift
            ;;
        *)
            echo "❌ 错误: 未知选项 '$1'"
            show_usage
            exit 1
            ;;
    esac
done

case $MODEL_TYPE in
    votenet)
        echo "🚀 启动感知模块 - VoteNet 模型"
        echo "   可视化: $ENABLE_VIZ"
        ros2 launch perception perception.launch.py \
            config_file:="$VOTENET_CONFIG" \
            enable_visualization:="$ENABLE_VIZ"
        ;;
    density)
        echo "🚀 启动感知模块 - 密度融合模型"
        echo "   可视化: $ENABLE_VIZ"
        ros2 launch perception perception.launch.py \
            config_file:="$DENSITY_CONFIG" \
            enable_visualization:="$ENABLE_VIZ"
        ;;
    *)
        echo "❌ 错误: 未知的模型类型 '$MODEL_TYPE'"
        echo ""
        show_usage
        exit 1
        ;;
esac
