# 机器人运动规划基础原理详解

## 你的核心疑问解答

### 1. 场景信息是否完全已知？

**是的，在当前实现中，场景信息是完全已知的：**

- **世界坐标系明确**：所有物体位置在 `scene.xml` 中定义
  - 红色立方体位置：`[-0.13, 0.6, 0.05]`
  - 机械臂基座位置：`[0, 0, 0]`
  - 工作台位置：固定
  
- **机器人模型完全已知**：
  - URDF 文件定义了所有连杆长度、关节限位
  - 正逆运动学模型精确
  - DH 参数或几何参数完整

- **目标点明确指定**：
  ```python
  target_A_pos = [-0.13, 0.5, 0.1]   # A点：接近立方体
  target_B_pos = [-0.13, 0.65, 0.1]  # B点：推动立方体
  ```

**这是一个简化的"完全信息"场景**，与真实世界中需要传感器、SLAM、视觉识别的复杂情况不同。

---

## 2. 运动规划的层次结构

### 当前代码实现的是什么层次？

```
高层任务规划（Task Planning）        ← 未涉及
    ↓
路径规划（Path Planning）            ← 简化版本（直线插值）
    ↓
轨迹规划（Trajectory Planning）      ← 当前代码的核心
    ↓
运动控制（Motion Control）           ← MuJoCo 的 PD 控制器
```

### 当前代码做了什么？

**第一步：逆运动学（IK）求解**
```python
# 从笛卡尔空间目标点 → 关节空间配置
joint_angles_A = my_chain.inverse_kinematics(
    target_A_pos,           # 末端位置 (x, y, z)
    ee_orientation,         # 末端姿态（旋转矩阵）
    "all",
    initial_position=ref_pos
)
```

- **输入**：末端执行器的目标位姿（位置 + 姿态）
- **输出**：满足该位姿的关节角配置 `[θ1, θ2, θ3, θ4, θ5, θ6]`
- **求解方法**：数值优化（通常是 Levenberg-Marquardt 算法）

**第二步：关节空间线性插值**
```python
class JointSpaceTrajectory:
    def _generate_trajectory(self):
        for i in range(self.steps + 1):
            yield self.start_joints + self.step * i
```

- **插值公式**：
  $$\mathbf{q}(t) = \mathbf{q}_{start} + \frac{t}{T}(\mathbf{q}_{end} - \mathbf{q}_{start})$$
  
- **这是最简单的插值**：没有速度规划、没有加速度约束、没有轨迹优化

**第三步：控制器执行**
```python
data.ctrl[:6] = waypoint  # 设置关节位置目标
mujoco.mj_step(model, data)  # MuJoCo 内部的 PD 控制器跟踪目标
```

---

## 3. 碰撞约束如何处理？

### 当前代码的真相：**没有显式的碰撞检测和避障！**

**为什么看起来能工作？**

1. **人工选择了安全的目标点**
   ```python
   target_A_pos = [-0.13, 0.5, 0.1]   # 手动保证这个位置是安全的
   target_B_pos = [-0.13, 0.65, 0.1]  # 直线路径恰好不会碰撞
   ```

2. **MuJoCo 的碰撞检测是被动的**
   - 物理引擎会检测碰撞并施加接触力
   - 但**不会主动避障**，只是"碰到了就推开"

3. **简单场景的运气**
   - 从 A 到 B 的直线路径恰好在障碍物上方
   - 如果目标点在桌子下方，机械臂会直接"撞过去"

### 真实的避障规划需要什么？

#### 方法 1：采样基础的规划算法（RRT/PRM）

```python
# 伪代码示例
def collision_free_path(start, goal, obstacles):
    """RRT（快速探索随机树）算法"""
    tree = [start]
    
    while goal not in tree:
        # 1. 随机采样关节空间
        q_rand = sample_random_configuration()
        
        # 2. 找到树中最近的节点
        q_near = find_nearest(tree, q_rand)
        
        # 3. 尝试连接（小步插值）
        q_new = extend(q_near, q_rand, step_size=0.1)
        
        # 4. 碰撞检测
        if not check_collision(q_new, obstacles):
            tree.append(q_new)
            parent[q_new] = q_near
    
    return reconstruct_path(parent, start, goal)
```

**特点**：
- 概率完备性（有解一定能找到）
- 不需要障碍物的数学模型
- 路径通常不是最优的

#### 方法 2：基于优化的规划（CHOMP/TrajOpt）

```python
# 轨迹优化的目标函数
def trajectory_cost(trajectory, obstacles):
    """
    最小化：
    1. 轨迹长度（效率）
    2. 碰撞风险（安全）
    3. 动力学代价（平滑性）
    """
    cost = 0
    
    # 1. 平滑性代价（加速度最小化）
    for i in range(len(trajectory) - 2):
        acceleration = trajectory[i+2] - 2*trajectory[i+1] + trajectory[i]
        cost += lambda1 * np.linalg.norm(acceleration)**2
    
    # 2. 碰撞代价（距离场）
    for q in trajectory:
        distance_to_obstacle = compute_distance_field(q, obstacles)
        if distance_to_obstacle < safe_distance:
            cost += lambda2 * (safe_distance - distance_to_obstacle)**2
    
    # 3. 使用梯度下降或序列二次规划（SQP）求解
    return cost
```

**CHOMP 的核心思想**：
- 初始轨迹可能穿过障碍物
- 通过梯度下降"推开"轨迹，避开障碍
- 同时保持轨迹平滑

---

## 4. 拉格朗日约束和轨迹插值的关系

### 当前代码使用的是什么？

**仅仅是几何插值，不涉及动力学约束：**

```python
# 简单的线性插值
q(t) = q_start + (q_end - q_start) * t / T
```

**没有考虑**：
- 关节速度限制
- 关节加速度限制
- 力矩限制
- 动力学方程

### 什么是拉格朗日约束下的轨迹优化？

**拉格朗日力学描述机器人动力学**：

$$\frac{d}{dt}\left(\frac{\partial L}{\partial \dot{\mathbf{q}}}\right) - \frac{\partial L}{\partial \mathbf{q}} = \boldsymbol{\tau}$$

其中：
- $L = T - V$（动能 - 势能）
- $\mathbf{q}$：关节角
- $\boldsymbol{\tau}$：关节力矩

**展开后得到机器人动力学方程**：

$$\mathbf{M}(\mathbf{q})\ddot{\mathbf{q}} + \mathbf{C}(\mathbf{q}, \dot{\mathbf{q}})\dot{\mathbf{q}} + \mathbf{G}(\mathbf{q}) = \boldsymbol{\tau}$$

- $\mathbf{M}$：质量矩阵
- $\mathbf{C}$：科氏力和离心力
- $\mathbf{G}$：重力项

### 轨迹优化问题的数学形式

```
minimize:  ∫[0,T] (q̈ᵀ M q̈ + cost_collision(q)) dt

subject to:
    1. q(0) = q_start, q(T) = q_goal          # 边界条件
    2. |q̇(t)| ≤ q̇_max                        # 速度限制
    3. |q̈(t)| ≤ q̈_max                        # 加速度限制
    4. d(q(t), obstacles) ≥ d_safe           # 碰撞约束
    5. M(q)q̈ + C(q,q̇)q̇ + G(q) = τ          # 动力学约束
    6. |τ(t)| ≤ τ_max                        # 力矩限制
```

**求解方法**：
- 直接法：将轨迹离散化为有限个路径点，转化为非线性规划（NLP）
- 间接法：使用变分法求解最优控制问题
- 数值求解器：IPOPT、SNOPT、SQP

---

## 5. 当前代码 vs 完整运动规划的对比

| 维度 | 当前代码 | 完整运动规划系统 |
|------|---------|----------------|
| **场景信息** | 完全已知（硬编码） | 传感器实时获取（RGB-D、激光雷达） |
| **目标点** | 手动指定笛卡尔坐标 | 高层任务分解 → 子目标生成 |
| **路径规划** | 无（直接 IK 求解） | RRT*/PRM/A* 等算法 |
| **碰撞检测** | 无主动避障 | 距离场/GJK 算法/FCL 库 |
| **轨迹插值** | 线性插值（关节空间） | 样条曲线/梯形速度曲线/最优轨迹 |
| **动力学约束** | 无 | 速度/加速度/力矩限制 |
| **控制** | MuJoCo 内置 PD | 自适应控制/阻抗控制/力控 |

---

## 6. 为什么当前代码能工作？

**因为这是一个教学级别的简化场景：**

1. **环境简单**：只有一个立方体障碍物
2. **目标点安全**：手动选择了不会碰撞的位置
3. **路径简单**：A→B 的直线路径恰好安全
4. **物理引擎兜底**：即使有轻微碰撞，MuJoCo 会施加接触力推开

**如果改变场景会怎样？**

```python
# 危险的目标点
target_B_pos = [-0.13, 0.65, -0.05]  # 在桌子下方

# 结果：机械臂会直接撞向桌子！
# 因为代码没有碰撞检测和避障规划
```

---

## 7. 进阶扩展方向

### 添加碰撞检测（基础版）

```python
import mujoco

def check_collision(model, data, qpos):
    """检查给定关节配置是否发生碰撞"""
    old_qpos = data.qpos.copy()
    data.qpos[:6] = qpos
    mujoco.mj_kinematics(model, data)
    mujoco.mj_collision(model, data)
    
    has_collision = data.ncon > 0  # 是否有接触点
    data.qpos[:] = old_qpos
    return has_collision
```

### 添加路径规划（RRT）

```python
from scipy.spatial import KDTree

class RRTPlanner:
    def __init__(self, model, data, start, goal):
        self.model = model
        self.data = data
        self.start = start
        self.goal = goal
        self.tree = [start]
        self.parent = {tuple(start): None}
    
    def plan(self, max_iterations=1000):
        for _ in range(max_iterations):
            q_rand = self.sample_random()
            q_near = self.nearest(q_rand)
            q_new = self.extend(q_near, q_rand)
            
            if not check_collision(self.model, self.data, q_new):
                self.tree.append(q_new)
                self.parent[tuple(q_new)] = tuple(q_near)
                
                if np.linalg.norm(q_new - self.goal) < 0.1:
                    return self.reconstruct_path(q_new)
        
        return None  # 未找到路径
```

### 添加轨迹优化（五次样条）

```python
from scipy.interpolate import CubicSpline

class SmoothTrajectory:
    def __init__(self, waypoints, duration):
        """使用五次样条生成 C² 连续的轨迹"""
        self.n_joints = waypoints.shape[1]
        self.time = np.linspace(0, duration, len(waypoints))
        
        # 为每个关节创建样条插值
        self.splines = [
            CubicSpline(self.time, waypoints[:, i], bc_type='clamped')
            for i in range(self.n_joints)
        ]
    
    def get_state(self, t):
        """返回 (位置, 速度, 加速度)"""
        q = np.array([s(t) for s in self.splines])
        qd = np.array([s(t, 1) for s in self.splines])
        qdd = np.array([s(t, 2) for s in self.splines])
        return q, qd, qdd
```

---

## 8. 总结：当前代码的本质

### 核心流程

```
1. 逆运动学：笛卡尔目标点 → 关节角配置
   [x, y, z, roll, pitch, yaw] → [θ1, θ2, θ3, θ4, θ5, θ6]

2. 关节空间插值：线性生成中间路径点
   q(t) = q_start + t * (q_end - q_start)

3. 控制器跟踪：MuJoCo 的 PD 控制器执行
   τ = Kp * (q_desired - q_current) - Kd * q̇_current
```

### 没有涉及的高级内容

- ❌ 传感器感知（假设完全信息）
- ❌ 高层任务规划（目标点手动指定）
- ❌ 路径规划算法（RRT/PRM/A*）
- ❌ 碰撞检测和避障（依赖物理引擎被动碰撞）
- ❌ 动力学约束优化（速度/加速度/力矩限制）
- ❌ 力控制（仅位置控制）

### 适用场景

**这是一个教学级别的"点到点运动"实现**，适用于：
- 学习机器人基础概念
- 简单的拾取-放置任务
- 障碍物稀疏的结构化环境

**不适用于**：
- 复杂环境中的自主导航
- 需要力控制的装配任务
- 动态障碍物场景
- 高精度轨迹跟踪

---

## 参考文献

1. **运动规划**：
   - LaValle, S. M. (2006). *Planning Algorithms*
   - Choset, H. et al. (2005). *Principles of Robot Motion*

2. **轨迹优化**：
   - Ratliff, N. et al. (2009). CHOMP: Gradient Optimization Techniques
   - Schulman, J. et al. (2013). Finding Locally Optimal, Collision-Free Trajectories

3. **机器人学**：
   - Craig, J. J. (2005). *Introduction to Robotics: Mechanics and Control*
   - Lynch, K. M., & Park, F. C. (2017). *Modern Robotics*
