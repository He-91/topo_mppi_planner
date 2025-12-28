#!/bin/bash

# 检查参数
if [ -z "$1" ]; then
    echo "❌ 使用方法: ./push_new_repo.sh <NEW_REPO_URL>"
    echo "例如: ./push_new_repo.sh https://github.com/He-91/ddo-topo-mppi-planner.git"
    exit 1
fi

NEW_REPO=$1

echo "📦 准备推送到新仓库: $NEW_REPO"
echo ""

# 1. 移除当前远程仓库
echo "🔧 步骤 1/4: 移除旧的远程仓库..."
git remote remove origin
echo "✅ 旧远程仓库已移除"
echo ""

# 2. 添加新远程仓库
echo "🔧 步骤 2/4: 添加新的远程仓库..."
git remote add origin $NEW_REPO
echo "✅ 新远程仓库已添加: $NEW_REPO"
echo ""

# 3. 推送所有分支和标签
echo "�� 步骤 3/4: 推送master分支..."
git push -u origin master
echo "✅ master分支推送完成"
echo ""

# 4. 推送所有标签
echo "🔧 步骤 4/4: 推送所有标签..."
git push origin --tags 2>/dev/null || echo "ℹ️  没有标签需要推送"
echo ""

echo "🎉 成功! 项目已推送到新仓库!"
echo "🔗 访问: $(echo $NEW_REPO | sed 's/.git$//')"
echo ""
echo "📝 下一步:"
echo "   1. 访问GitHub仓库页面"
echo "   2. 添加项目描述和主题标签"
echo "   3. 设置仓库为public（如果需要）"

