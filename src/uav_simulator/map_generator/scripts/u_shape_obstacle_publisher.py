#!/usr/bin/env python3
"""
U型静态障碍物发布器
用于 Env4: 在航线必经之路上放置 U 型障碍物，迫使无人机穿越其中
发布点云到 /map_generator/global_cloud (供 cloud_merger 合并)
"""

import rospy
import numpy as np
from sensor_msgs.msg import PointCloud2
import sensor_msgs.point_cloud2 as pc2
from std_msgs.msg import Header


def generate_pillar(cx, cy, radius=0.4, height=4.0, resolution=0.1):
    """生成一根圆柱的点云"""
    points = []
    n_theta = int(2 * np.pi * radius / resolution) + 1
    n_height = int(height / resolution) + 1
    n_r = int(radius / resolution) + 1
    
    for h_idx in range(n_height):
        z = h_idx * resolution
        for t_idx in range(n_theta):
            theta = 2 * np.pi * t_idx / n_theta
            # 圆柱表面
            px = cx + radius * np.cos(theta)
            py = cy + radius * np.sin(theta)
            points.append([px, py, z])
            # 填充内部
            for r_idx in range(n_r):
                r = r_idx * resolution
                px_inner = cx + r * np.cos(theta)
                py_inner = cy + r * np.sin(theta)
                points.append([px_inner, py_inner, z])
    
    return points


def generate_u_shape(center_x, center_y, opening_dir='right',
                     width=4.0, depth=3.0, wall_thickness=0.8,
                     pillar_spacing=0.9, pillar_radius=0.35, height=4.0):
    """
    生成U型障碍物 (由密集柱子拼成)
    
    center_x, center_y: U型中心坐标
    opening_dir: 开口方向 ('right', 'left', 'up', 'down')
    width: U型宽度 (开口方向的垂直方向)
    depth: U型深度 (开口方向)
    wall_thickness: 壁厚 (用几根柱子)
    pillar_spacing: 柱子间距
    pillar_radius: 柱子半径
    height: 柱子高度
    """
    all_points = []
    
    half_w = width / 2.0
    
    if opening_dir == 'right':
        # U型开口朝右: 左壁 + 上壁 + 下壁
        # 左壁 (底部)
        for dy in np.arange(-half_w, half_w + 0.01, pillar_spacing):
            all_points.extend(generate_pillar(center_x - depth/2, center_y + dy, pillar_radius, height))
        # 上壁
        for dx in np.arange(-depth/2, depth/2 + 0.01, pillar_spacing):
            all_points.extend(generate_pillar(center_x + dx, center_y + half_w, pillar_radius, height))
        # 下壁
        for dx in np.arange(-depth/2, depth/2 + 0.01, pillar_spacing):
            all_points.extend(generate_pillar(center_x + dx, center_y - half_w, pillar_radius, height))
    
    elif opening_dir == 'left':
        # U型开口朝左
        # 右壁
        for dy in np.arange(-half_w, half_w + 0.01, pillar_spacing):
            all_points.extend(generate_pillar(center_x + depth/2, center_y + dy, pillar_radius, height))
        # 上壁
        for dx in np.arange(-depth/2, depth/2 + 0.01, pillar_spacing):
            all_points.extend(generate_pillar(center_x + dx, center_y + half_w, pillar_radius, height))
        # 下壁
        for dx in np.arange(-depth/2, depth/2 + 0.01, pillar_spacing):
            all_points.extend(generate_pillar(center_x + dx, center_y - half_w, pillar_radius, height))
    
    elif opening_dir == 'up':
        # U型开口朝上
        # 底壁
        for dx in np.arange(-half_w, half_w + 0.01, pillar_spacing):
            all_points.extend(generate_pillar(center_x + dx, center_y - depth/2, pillar_radius, height))
        # 左壁
        for dy in np.arange(-depth/2, depth/2 + 0.01, pillar_spacing):
            all_points.extend(generate_pillar(center_x - half_w, center_y + dy, pillar_radius, height))
        # 右壁
        for dy in np.arange(-depth/2, depth/2 + 0.01, pillar_spacing):
            all_points.extend(generate_pillar(center_x + half_w, center_y + dy, pillar_radius, height))
    
    elif opening_dir == 'down':
        # U型开口朝下
        # 顶壁
        for dx in np.arange(-half_w, half_w + 0.01, pillar_spacing):
            all_points.extend(generate_pillar(center_x + dx, center_y + depth/2, pillar_radius, height))
        # 左壁
        for dy in np.arange(-depth/2, depth/2 + 0.01, pillar_spacing):
            all_points.extend(generate_pillar(center_x - half_w, center_y + dy, pillar_radius, height))
        # 右壁
        for dy in np.arange(-depth/2, depth/2 + 0.01, pillar_spacing):
            all_points.extend(generate_pillar(center_x + half_w, center_y + dy, pillar_radius, height))
    
    return all_points


def main():
    rospy.init_node('u_shape_obstacle_publisher', anonymous=True)
    
    pub = rospy.Publisher('/map_generator/global_cloud', PointCloud2, queue_size=10)
    rate = rospy.Rate(2)  # 2Hz 发布频率
    
    rospy.loginfo("=== U型静态障碍物发布器 ===")
    
    # ========= 障碍物布局 =========
    all_points = []
    
    # U型障碍物1: x=0附近，航线中间段必经之路
    # 开口朝右 — 无人机从左边来必须进入U型再出来
    rospy.loginfo("生成 U型1: center=(0, 0), 开口朝右, 在中间航线上")
    all_points.extend(generate_u_shape(
        center_x=0.0, center_y=0.0,
        opening_dir='right',
        width=5.0, depth=4.0,
        pillar_spacing=0.85, pillar_radius=0.35, height=4.0
    ))
    
    # U型障碍物2: 回程航线 (-8, 3.5) 附近
    # 开口朝左下 — 无人机从(0,7)回(-16,0)必须穿过
    rospy.loginfo("生成 U型2: center=(-8, 3), 开口朝左, 在回程航线上")
    all_points.extend(generate_u_shape(
        center_x=-8.0, center_y=3.0,
        opening_dir='left',
        width=4.5, depth=3.5,
        pillar_spacing=0.85, pillar_radius=0.35, height=4.0
    ))
    
    rospy.loginfo("  总点数: %d", len(all_points))
    
    # 创建点云消息
    header = Header()
    header.frame_id = 'world'
    
    while not rospy.is_shutdown():
        header.stamp = rospy.Time.now()
        cloud_msg = pc2.create_cloud_xyz32(header, all_points)
        pub.publish(cloud_msg)
        rate.sleep()


if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass
