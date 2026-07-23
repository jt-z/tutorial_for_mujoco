# 两阶段运动任务说明文档

## 修改概述

将原来的单阶段运动（直接移动到目标位置）改为两阶段运动任务：
1. **阶段1**: 移动到A位置（接近红色立方体但不接触）
2. **阶段2**: 移动到B位置（推动红色立方体）

## 主要修改内容

### 1. main.py 修改

#### 新增的目标位置定义
```python
target_A_pos = [-0.13, 0.5, 0.1]   # A位置：靠近立方体但不接触
target_B_pos = [-0.13, 0.65, 0.1]  # B位置：推动立方体
```

#### 两阶段轨迹管理
- `trajectory_to_A`: 从初始位置到A位置的轨迹（100步）
- `trajectory_to_B`: 从A位置到B位置的轨迹（150步，到达A后动态创建）
- `current_phase`: 跟踪当前执行阶段（"to_A" → "to_B" → "completed"）

#### 阶段切换逻辑
```python
if current_phase == "to_A":
    # 执行到A的轨迹
    # 检测是否到达A位置
    if 到达A位置:
        创建trajectory_to_B
        current_phase = "to_B"

elif current_phase == "to_B":
    # 执行到B的轨迹
    # 检测是否到达B位置
    if 到达B位置:
        current_phase = "completed"
```

### 2. scene.xml 修改

#### 新增目标位置可视化标记

**A位置标记（绿色半透明球体）**
```xml
<body name="target_A_marker" pos="-0.13 0.5 0.1">
    <geom name="target_A_geom" type="sphere" size="0.03" 
          rgba="0 1 0 0.5" contype="0" conaffinity="0"/>
</body>
```

**B位置标记（蓝色半透明球体）**
```xml
<body name="target_B_marker" pos="-0.13 0.65 0.1">
    <geom name="target_B_geom" type="sphere" size="0.03" 
          rgba="0 0 1 0.5" contype="0" conaffinity="0"/>
</body>
```

**属性说明：**
- `size="0.03"`: 球体半径3cm
- `rgba="0 1 0 0.5"`: 半透明绿色（alpha=0.5）
- `contype="0" conaffinity="0"`: 无碰撞（不影响物理仿真）

## 场景布局

```
坐标系统（从上往下看）：
       Y轴↑
          |
    B位置 ● (蓝色, y=0.65)
          |
          |  ← 15cm推动距离
          |
红色立方体 ■ (y=0.5)
          |
    A位置 ● (绿色, y=0.5, 与立方体同高度)
          |
          |  ← 20cm接近距离
          |
初始末端位置 (y=0.3)
          |
          |
        机器人基座
   ───────┼───────→ X轴
          |
```

## 运动过程详解

### 阶段1: 接近立方体
- **起点**: `[-0.13, 0.3, 0.1]`（机械臂初始末端位置）
- **终点**: `[-0.13, 0.5, 0.1]`（A位置，绿色球体）
- **距离**: 沿Y轴移动 20cm
- **步数**: 100步
- **目的**: 将末端移动到立方体附近，准备推动

### 阶段2: 推动立方体
- **起点**: `[-0.13, 0.5, 0.1]`（A位置）
- **终点**: `[-0.13, 0.65, 0.1]`（B位置，蓝色球体）
- **距离**: 沿Y轴移动 15cm
- **步数**: 150步
- **目的**: 推动红色立方体向前移动

## 控制台输出

程序运行时会打印以下信息：
```
目标A位置: [-0.13, 0.5, 0.1]
目标B位置: [-0.13, 0.65, 0.1]
阶段1: 移动到A位置（接近立方体）
已到达A位置！
阶段2: 移动到B位置（推动立方体）
已到达B位置！任务完成
```

## 可视化效果

运行程序后，MuJoCo 3D窗口中会看到：
1. **红色立方体**: 目标物体（可移动）
2. **绿色半透明球**: A位置标记（接近点）
3. **蓝色半透明球**: B位置标记（推动终点）
4. **机械臂末端绿球**: 力传感器（实时跟随末端）
5. **世界坐标系**: RGB圆柱体表示XYZ轴

## 技术要点

### 1. 逆运动学求解
对A和B位置分别进行IK求解：
```python
joint_angles_A = my_chain.inverse_kinematics(target_A_pos, ee_orientation, "all", ...)
joint_angles_B = my_chain.inverse_kinematics(target_B_pos, ee_orientation, "all", ...)
```

### 2. 动态轨迹创建
B阶段的轨迹在到达A位置后才创建，确保从实际到达的关节角出发：
```python
trajectory_to_B = JointSpaceTrajectory(data.qpos[:6], target_B_joints, steps=150)
```

### 3. 到达检测
使用`np.allclose()`检测关节角是否接近目标：
```python
if np.allclose(data.qpos[:6], target_A_joints, atol=0.02):  # 容差0.02弧度
```

### 4. 状态机设计
使用字符串变量管理三个状态：
- `"to_A"`: 正在前往A位置
- `"to_B"`: 正在前往B位置
- `"completed"`: 任务完成

## 参数调整建议

### 修改目标位置
在`main.py`中修改：
```python
target_A_pos = [x, y, z]  # 修改A位置坐标
target_B_pos = [x, y, z]  # 修改B位置坐标
```

同时需要同步修改`scene.xml`中的可视化标记位置：
```xml
<body name="target_A_marker" pos="x y z">
<body name="target_B_marker" pos="x y z">
```

### 修改运动速度
调整轨迹步数（步数越少速度越快）：
```python
trajectory_to_A = JointSpaceTrajectory(start_joints, target_A_joints, steps=200)  # 变慢
trajectory_to_B = JointSpaceTrajectory(..., steps=50)  # 变快
```

### 修改标记颜色和大小
在`scene.xml`中修改：
```xml
<geom name="target_A_geom" type="sphere" 
      size="0.05"              <!-- 改变大小 -->
      rgba="1 1 0 0.7"         <!-- 改变颜色（黄色）和透明度 -->
      contype="0" conaffinity="0"/>
```

## 扩展方向

### 1. 多阶段任务
添加更多中间位置：
```python
target_C_pos = [-0.13, 0.75, 0.1]
trajectory_to_C = JointSpaceTrajectory(...)
current_phase = "to_C"
```

### 2. 添加等待时间
在到达某个位置后暂停：
```python
if current_phase == "to_A":
    if 到达A位置:
        wait_counter += 1
        if wait_counter > 100:  # 等待100帧（约2秒）
            current_phase = "to_B"
```

### 3. 基于力反馈的切换
根据接触力大小切换阶段：
```python
force_magnitude = np.linalg.norm(filtered_force)
if force_magnitude > 5.0:  # 检测到接触
    current_phase = "to_B"
```

### 4. 圆弧轨迹
在笛卡尔空间规划圆弧路径，避免碰撞：
```python
def arc_trajectory(start, end, height, steps):
    # 生成弧形路径点
    path = []
    for i in range(steps):
        t = i / steps
        # 抛物线插值
        pos = start + t * (end - start)
        pos[2] += height * (1 - 4*(t-0.5)**2)  # 中间抬高
        path.append(pos)
    return path
```

## 运行方法

```bash
cd /home/zjt/dev/On_Git_Projects/tutorial_for_mujoco
python main.py
```

程序会打开两个窗口：
1. MuJoCo 3D仿真窗口（显示机械臂和标记）
2. Matplotlib力向量可视化窗口

## 故障排除

### 问题1: 标记不可见
- 检查标记位置是否在相机视野内
- 调整`viewer_init()`中的相机参数

### 问题2: 机械臂无法到达目标
- IK求解可能失败，检查目标位置是否在工作空间内
- 尝试调整`ee_euler`姿态角度

### 问题3: 立方体没有被推动
- 检查B位置是否足够接近立方体
- 确保力传感器球体与立方体有物理接触

---

**文档创建时间**: 2026-07-22  
**相关文件**: 
- [main.py](../main.py)
- [scene.xml](../model/universal_robots_ur5e/scene.xml)
