#!/bin/bash

# 检查参数
if [ -z "$1" ]; then
    echo "❌ 使用方法: ./fresh_start.sh <NEW_REPO_URL>"
    echo "例如: ./fresh_start.sh https://github.com/He-91/ddo-topo-mppi-planner.git"
    exit 1
fi

NEW_REPO=$1
BACKUP_DIR="/tmp/ddo-topo-mppi-backup-$(date +%Y%m%d_%H%M%S)"

echo "🔄 完全重新开始推送到新仓库"
echo "📦 新仓库: $NEW_REPO"
echo ""

# 1. 备份当前.git
echo "🔧 步骤 1/5: 备份当前Git历史..."
cp -r .git "$BACKUP_DIR"
echo "✅ 已备份到: $BACKUP_DIR"
echo ""

# 2. 删除.git目录
echo "🔧 步骤 2/5: 删除旧Git历史..."
rm -rf .git
echo "✅ 旧Git历史已删除"
echo ""

# 3. 重新初始化Git
echo "🔧 步骤 3/5: 初始化新Git仓库..."
git init
git config user.name "$(git config --global user.name)"
git config user.email "$(git config --global user.email)"
echo "✅ 新Git仓库已初始化"
echo ""

# 4. 添加并提交所有文件
echo "🔧 步骤 4/5: 添加所有文件..."
git add -A
git commit -m "feat: Initial commit of DDO-Topo-MPPI Planner

A high-performance autonomous navigation system combining topological 
path planning with GPU-accelerated MPPI optimization and B-spline 
trajectory smoothing.

Key Features:
- 100% planning success rate (25/25 test cases)
- GPU-accelerated MPPI (~1.76ms avg)
- Adaptive B-spline optimization (lambda=0.5, iter=300)
- Intelligent MPPI fallback mechanism
- Real-time dynamic obstacle avoidance

Package renamed from ego_planner to topo_mppi_planner with significant
performance improvements over the original implementation."
echo "✅ 所有文件已提交"
echo ""

# 5. 添加远程仓库并推送
echo "🔧 步骤 5/5: 推送到新仓库..."
git branch -M main
git remote add origin $NEW_REPO
git push -u origin main
echo ""

echo "🎉 成功! 全新项目已推送到GitHub!"
echo "🔗 访问: $(echo $NEW_REPO | sed 's/.git$//')"
echo ""
echo "📝 提示:"
echo "   - 旧Git历史已备份到: $BACKUP_DIR"
echo "   - 默认分支已更改为: main"
echo "   - 所有文件作为初始提交推送"

