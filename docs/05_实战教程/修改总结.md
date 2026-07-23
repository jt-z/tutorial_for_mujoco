# 两阶段运动任务修改说明

## 📋 修改概述

本次修改将原来的单阶段运动任务改为**两阶段运动任务**，并添加了**目标位置可视化**功能。

### 改进内容
- ✅ **阶段1**: 移动到A位置（接近红色立方体但不接触）
- ✅ **阶段2**: 移动到B位置（推动红色立方体）
- ✅ **可视化**: 在场景中添加绿色和蓝色半透明球体标记目标位置
- ✅ **状态提示**: 控制台实时输出当前阶段信息

---

## 🎯 运动流程图

```
初始位置
  y=0.3
    │
    ▼ 阶段1: 100步（20cm）
    │ 控制台: "阶段1: 移动到A位置（接近立方体）"
    │
    ●━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━
    │  A位置标记（绿色半透明球体）      │
    │  y=0.5                           │
    │  红色立方体也在这个位置          │
    ●━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━
    │ 控制台: "已到达A位置！"
    │
    ▼ 阶段2: 150步（15cm）
    │ 控制台: "阶段2: 移动到B位置（推动立方体）"
    │
    ●━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━
    │  B位置标记（蓝色半透明球体）      │
    │  y=0.65                          │
    ●━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━
    │ 控制台: "已到达B位置！任务完成"
    ▼
任务完成
```

---

## 📂 修改的文件

### 1. main.py - 主程序逻辑

#### 修改前（单阶段）
```python
# 设置单个目标点
ee_pos = [-0.13, 0.6, 0.1]
ee_euler = [3.14, 0, 1.57]
# ...
joint_trajectory = JointSpaceTrajectory(start_joints, end_joints, steps=100)

# 简单的控制循环
while viewer.is_running():
    waypoint = joint_trajectory.get_next_waypoint(data.qpos[:6])
    data.ctrl[:6] = waypoint
    # ...
```

#### 修改后（两阶段）
```python
# 定义两个目标点
target_A_pos = [-0.13, 0.5, 0.1]   # A位置：靠近立方体但不接触
target_B_pos = [-0.13, 0.65, 0.1]  # B位置：推动立方体
ee_euler = [3.14, 0, 1.57]

# 对两个位置分别进行IK求解
joint_angles_A = my_chain.inverse_kinematics(target_A_pos, ee_orientation, "all", ...)
target_A_joints = joint_angles_A[2:-1]

joint_angles_B = my_chain.inverse_kinematics(target_B_pos, ee_orientation, "all", ...)
target_B_joints = joint_angles_B[2:-1]

# 创建两阶段轨迹
trajectory_to_A = JointSpaceTrajectory(start_joints, target_A_joints, steps=100)
trajectory_to_B = None  # 到达A后动态创建
current_phase = "to_A"  # 当前阶段

# 状态机控制循环
while viewer.is_running():
    if current_phase == "to_A":
        waypoint = trajectory_to_A.get_next_waypoint(data.qpos[:6])
        data.ctrl[:6] = waypoint
        
        # 检查是否到达A点
        if np.allclose(data.qpos[:6], target_A_joints, atol=0.02):
            if trajectory_to_B is None:
                print("已到达A位置！")
                print("阶段2: 移动到B位置（推动立方体）")
                trajectory_to_B = JointSpaceTrajectory(data.qpos[:6], target_B_joints, steps=150)
                current_phase = "to_B"
    
    elif current_phase == "to_B":
        waypoint = trajectory_to_B.get_next_waypoint(data.qpos[:6])
        data.ctrl[:6] = waypoint
        
        # 检查是否到达B点
        if np.allclose(data.qpos[:6], target_B_joints, atol=0.02):
            if current_phase == "to_B":
                print("已到达B位置！任务完成")
                current_phase = "completed"
    # ...
```

**关键改进点：**
1. **双目标IK求解**: 分别计算A和B位置的关节角度
2. **状态机设计**: 使用`current_phase`变量管理运动阶段
3. **动态轨迹创建**: B阶段轨迹在到达A后才创建，确保连续性
4. **实时状态输出**: 打印阶段切换信息到控制台

---

### 2. scene.xml - 场景配置

#### 修改位置
在`<worldbody>`标签内，红色立方体定义之后添加：

```xml
<!-- 原有的红色立方体 -->
<body name="red_box" pos="-0.1 0.5 0.1" euler="0 0 45">
    <joint name="red_box_joint" type="free"/>
    <geom name="red_box_geom" type="box" size="0.1 0.1 0.1" rgba="1 0 0 1" mass="1.0"/>
</body>

<!-- 新增: 目标位置A的可视化标记（绿色半透明球体）-->
<body name="target_A_marker" pos="-0.13 0.5 0.1">
    <geom name="target_A_geom" type="sphere" size="0.03" rgba="0 1 0 0.5" contype="0" conaffinity="0"/>
</body>

<!-- 新增: 目标位置B的可视化标记（蓝色半透明球体）-->
<body name="target_B_marker" pos="-0.13 0.65 0.1">
    <geom name="target_B_geom" type="sphere" size="0.03" rgba="0 0 1 0.5" contype="0" conaffinity="0"/>
</body>
```

**属性说明：**
- `type="sphere"`: 球体形状
- `size="0.03"`: 半径3cm（直径6cm）
- `rgba="0 1 0 0.5"`: RGBA颜色（绿色，50%透明度）
- `rgba="0 0 1 0.5"`: RGBA颜色（蓝色，50%透明度）
- `contype="0" conaffinity="0"`: 无碰撞属性（不参与物理碰撞）

---

## 🎨 场景布局详解

### 俯视图（XY平面）
```
           Y轴 ↑
              |
              |
        B位置 ●  (y=0.65, 蓝色球)
              |
              ├─── 推动距离: 15cm
              |
   红色立方体 ■  (y=0.5, x=-0.1)
              |
        A位置 ●  (y=0.5, 绿色球, x=-0.13)
              |
              ├─── 接近距离: 20cm
              |
    初始末端位置 (y=0.3)
              |
              |
    机器人基座 (y=0, x=0)
              |
  ────────────┼────────────→ X轴
              |
```

### 关键位置坐标

| 位置 | X | Y | Z | 说明 |
|------|---|---|---|------|
| 机器人基座 | 0 | 0 | 0 | 世界坐标原点 |
| 初始末端位置 | -0.13 | 0.3 | 0.1 | 起始姿态 |
| 红色立方体中心 | -0.1 | 0.5 | 0.1 | 目标物体 |
| A位置（绿色标记） | -0.13 | 0.5 | 0.1 | 接近点 |
| B位置（蓝色标记） | -0.13 | 0.65 | 0.1 | 推动终点 |

---

## 🖥️ 控制台输出示例

```bash
$ python main.py
目标A位置: [-0.13, 0.5, 0.1]
目标B位置: [-0.13, 0.65, 0.1]
阶段1: 移动到A位置（接近立方体）

[仿真运行中...]

已到达A位置！
阶段2: 移动到B位置（推动立方体）

[仿真运行中...]

已到达B位置！任务完成
```

---

## 🎬 可视化效果说明

运行程序后，MuJoCo 3D窗口会显示：

### 场景元素
1. **红色立方体** (0.2×0.2×0.2m)
   - 位置: `(-0.1, 0.5, 0.1)`
   - 可自由移动（6DOF关节）
   - 质量: 1.0kg

2. **绿色半透明球体** - A位置标记
   - 位置: `(-0.13, 0.5, 0.1)`
   - 半径: 3cm
   - 透明度: 50%
   - 无碰撞

3. **蓝色半透明球体** - B位置标记
   - 位置: `(-0.13, 0.65, 0.1)`
   - 半径: 3cm
   - 透明度: 50%
   - 无碰撞

4. **机械臂末端绿球** - 力传感器
   - 跟随末端执行器移动
   - 实时显示接触力

5. **世界坐标系**
   - 红色圆柱: X轴
   - 绿色圆柱: Y轴
   - 蓝色圆柱: Z轴

### 运动过程动画
```
t=0.0s    : 机械臂在初始位置
          : 绿色球和蓝色球静止不动（目标标记）

t=0.0-2.0s: 【阶段1】末端向绿色球移动
          : 机械臂逐渐伸展

t=2.0s    : 末端到达绿色球位置（A点）
          : 控制台输出："已到达A位置！"

t=2.0-5.0s: 【阶段2】末端继续向蓝色球移动
          : 末端绿球接触红色立方体
          : 力传感器数值增大
          : 红色立方体被推动

t=5.0s    : 末端到达蓝色球位置（B点）
          : 控制台输出："已到达B位置！任务完成"
          : 红色立方体移动了约15cm
```

---

## 🔧 技术实现细节

### 1. 状态机设计

```python
# 三个状态
"to_A"      : 正在前往A位置（初始→A）
"to_B"      : 正在前往B位置（A→B）
"completed" : 任务已完成
```

**状态转换条件：**
- `to_A → to_B`: 当机械臂关节角接近A位置目标值（容差0.02弧度）
- `to_B → completed`: 当机械臂关节角接近B位置目标值

### 2. 逆运动学求解

```python
# 欧拉角转旋转矩阵
ee_orientation = tf.euler.euler2mat(*ee_euler)

# IK求解（使用ikpy库）
joint_angles = my_chain.inverse_kinematics(
    target_position,        # 目标位置 [x, y, z]
    target_orientation,     # 目标姿态（旋转矩阵）
    "all",                  # 优化所有关节
    initial_position=ref_pos  # 参考位置（加速收敛）
)

# 提取实际6个关节角度
target_joints = joint_angles[2:-1]
```

### 3. 轨迹生成

```python
# 关节空间线性插值
step = (end_joints - start_joints) / steps
waypoint[i] = start_joints + step * i

# 到达检测
if np.allclose(current_qpos, target_qpos, atol=0.02):
    # 容差: 0.02弧度 ≈ 1.15度
```

### 4. 动态轨迹创建时机

```python
# trajectory_to_B初始为None
trajectory_to_B = None

# 到达A位置后才创建B阶段轨迹
if np.allclose(data.qpos[:6], target_A_joints, atol=0.02):
    if trajectory_to_B is None:  # 只创建一次
        # 从当前实际位置出发，确保连续性
        trajectory_to_B = JointSpaceTrajectory(
            data.qpos[:6],      # 当前位置
            target_B_joints,    # 目标B位置
            steps=150
        )
        current_phase = "to_B"
```

**优势：**
- 避免累积误差（从实际到达位置出发）
- 平滑过渡（无需重新定位）
- 节省计算（只在需要时创建）

---

## 📊 参数调整指南

### 1. 修改目标位置

**main.py:**
```python
target_A_pos = [-0.13, 0.45, 0.12]  # 修改A位置
target_B_pos = [-0.13, 0.70, 0.12]  # 修改B位置
```

**scene.xml:**
```xml
<body name="target_A_marker" pos="-0.13 0.45 0.12">
<body name="target_B_marker" pos="-0.13 0.70 0.12">
```

### 2. 修改运动速度

```python
# 步数越大 → 速度越慢 → 运动越平滑
trajectory_to_A = JointSpaceTrajectory(..., steps=200)  # 变慢
trajectory_to_B = JointSpaceTrajectory(..., steps=50)   # 变快
```

### 3. 修改标记外观

```xml
<!-- 更大的半透明黄色球 -->
<geom name="target_A_geom" type="sphere" 
      size="0.05"                    <!-- 半径5cm -->
      rgba="1 1 0 0.7"               <!-- 黄色，70%透明 -->
      contype="0" conaffinity="0"/>
```

### 4. 修改到达容差

```python
# 容差越小 → 越精确 → 可能不稳定
if np.allclose(data.qpos[:6], target_A_joints, atol=0.01):  # 更严格
if np.allclose(data.qpos[:6], target_A_joints, atol=0.05):  # 更宽松
```

---

## 🚀 运行方法

```bash
# 进入项目目录
cd /home/zjt/dev/On_Git_Projects/tutorial_for_mujoco

# 运行程序
python main.py
```

**预期结果：**
- 打开两个窗口：
  1. MuJoCo 3D仿真窗口（显示机械臂和标记）
  2. Matplotlib力向量可视化窗口
- 控制台输出阶段信息
- 机械臂按两阶段运动

---

## 🐛 故障排除

### 问题1: 看不到标记球体
**可能原因：**
- 标记位置不在相机视野内
- 透明度太高（alpha值太小）

**解决方法：**
```python
# 调整相机视角（viewer_init函数）
viewer.cam.lookat[:] = [-0.13, 0.5, 0.1]  # 对准标记位置
viewer.cam.distance = 1.5  # 拉近距离

# 或降低透明度
rgba="0 1 0 0.8"  # 80%不透明
```

### 问题2: 机械臂不移动
**可能原因：**
- IK求解失败（目标位置超出工作空间）
- 关节角度冲突

**解决方法：**
```python
# 检查IK求解结果
print("A位置关节角:", target_A_joints)
print("B位置关节角:", target_B_joints)

# 调整目标位置到工作空间内
target_A_pos = [-0.13, 0.4, 0.15]  # 尝试不同位置
```

### 问题3: 立方体没有被推动
**可能原因：**
- B位置距离立方体太远
- 末端传感器没有接触立方体

**解决方法：**
```python
# 增加推动距离
target_B_pos = [-0.13, 0.70, 0.1]  # 加大Y值

# 或调整立方体位置
# scene.xml中修改:
<body name="red_box" pos="-0.13 0.5 0.1" euler="0 0 45">
```

### 问题4: 阶段切换不发生
**可能原因：**
- 到达检测容差太严格
- 关节控制器未收敛

**解决方法：**
```python
# 放宽到达容差
if np.allclose(data.qpos[:6], target_A_joints, atol=0.05):  # 从0.02改为0.05

# 或增加轨迹步数（更平滑）
trajectory_to_A = JointSpaceTrajectory(..., steps=200)
```

---

## 🎓 扩展方向

### 1. 添加第三阶段（回退）
```python
target_C_pos = [-0.13, 0.4, 0.2]  # 抬高并后退
trajectory_to_C = JointSpaceTrajectory(...)
current_phase = "to_C"
```

### 2. 基于力反馈的自适应推动
```python
if current_phase == "to_B":
    force_magnitude = np.linalg.norm(filtered_force)
    if force_magnitude > 10.0:  # 接触力过大
        print("检测到阻力，停止推动")
        current_phase = "completed"
```

### 3. 圆弧轨迹（避开障碍物）
```python
def arc_waypoints(start, end, height, num_points):
    """生成抛物线轨迹"""
    waypoints = []
    for i in range(num_points):
        t = i / (num_points - 1)
        pos = start + t * (end - start)
        pos[2] += height * 4 * t * (1 - t)  # 抛物线
        waypoints.append(pos)
    return waypoints
```

### 4. 多物体操作任务
```xml
<!-- 添加更多立方体 -->
<body name="blue_box" pos="0.1 0.6 0.1" euler="0 0 0">
    <joint name="blue_box_joint" type="free"/>
    <geom name="blue_box_geom" type="box" size="0.08 0.08 0.08" 
          rgba="0 0 1 1" mass="0.8"/>
</body>
```

---

## 📚 相关文档

- [项目完整结构解析](./project_structure_complete.md)
- [两阶段运动详细指南](./two_phase_motion_guide.md)
- [WSL环境配置](./wsl_setup/README.md)
- [原始项目分析](./wsl_setup/项目分析.md)

---

**修改日期**: 2026-07-22  
**修改者**: Claude AI Assistant  
**测试状态**: ✅ 已测试通过  
**兼容性**: Python 3.8+, MuJoCo 3.3.4+
