# MuJoCo UR5e 机械臂仿真教程

基于MuJoCo物理引擎的UR5e机械臂仿真项目，实现两阶段运动控制、力传感器反馈和实时可视化。

[![Python Version](https://img.shields.io/badge/python-3.8%2B-blue)](https://www.python.org/)
[![MuJoCo Version](https://img.shields.io/badge/mujoco-3.3.4%2B-green)](https://mujoco.org/)
[![License](https://img.shields.io/badge/license-MIT-orange)](LICENSE)

---

## ✨ 功能特性

- 🤖 **UR5e机械臂仿真** - 6自由度机械臂完整运动学模型
- 🎯 **两阶段运动控制** - 接近→推动的多阶段任务编排
- 📊 **力传感器反馈** - 实时三维力向量测量与可视化
- 🎨 **3D可视化** - MuJoCo原生渲染 + Matplotlib数据图表
- 📐 **逆运动学求解** - 基于ikpy的IK求解器
- 🔄 **关节空间规划** - 平滑轨迹插值算法

---

## 🚀 快速开始

### 环境要求

- Python 3.8+
- 支持OpenGL的系统（WSL2需启用WSLg）

### 安装步骤

#### 1. 克隆项目
```bash
git clone https://github.com/yourusername/tutorial_for_mujoco.git
cd tutorial_for_mujoco
```

#### 2. 创建虚拟环境（推荐）
```bash
# 使用conda
conda create -n tutorial_for_mujoco python=3.9
conda activate tutorial_for_mujoco

# 或使用venv
python -m venv venv
source venv/bin/activate  # Linux/Mac
venv\Scripts\activate     # Windows
```

#### 3. 安装依赖
```bash
pip install -r requirements.txt
```

#### 4. 运行程序
```bash
python main.py
```

程序将打开两个窗口：
- **MuJoCo 3D仿真窗口** - 显示机械臂和场景
- **Matplotlib可视化窗口** - 显示力传感器数据

---

## 📚 完整文档

完善的中文文档体系，适合不同水平的开发者：

### 📖 入门教程
- [**快速入门**](docs/02_快速入门.md) - 5分钟运行第一个示例
- [**环境配置**](docs/01_环境配置/README.md) - WSL2/Linux/Windows/macOS配置指南
- [**项目结构**](docs/03_项目结构概览.md) - 代码组织和文件说明

### 🎓 核心知识
- [**坐标系统详解**](docs/04_基础知识/坐标系统详解.md) - MuJoCo坐标系、关节配置和位姿表示
- [**运动规划原理**](docs/04_基础知识/运动规划原理.md) - 逆运动学、轨迹规划和碰撞检测

### 💡 实战教程
- [**两阶段运动控制**](docs/05_实战教程/两阶段运动控制.md) - 实现接近→推动任务
- [**修改总结**](docs/05_实战教程/修改总结.md) - 代码变更详解

### 🔧 参考手册
- [**参数调整指南**](docs/06_参考手册/参数调整指南.md) - 修改位置、速度、姿态等参数
- [**故障排除**](docs/06_参考手册/故障排除.md) - 17个常见问题解决方案

### 📑 更多资源
- [**变更日志**](docs/附录/CHANGELOG.md) - 版本历史和计划功能
- [**文档索引**](docs/README.md) - 完整文档导航

---

## 🎬 运行效果

### 两阶段运动过程

```
阶段1: 接近立方体
机械臂末端移动到A位置（绿色球标记）
├─ 距离: 20cm
├─ 时间: 约2秒
└─ 控制台: "已到达A位置！"

阶段2: 推动立方体
机械臂末端移动到B位置（蓝色球标记）
├─ 距离: 15cm
├─ 时间: 约3秒
├─ 效果: 红色立方体被推动
└─ 控制台: "已到达B位置！任务完成"
```

### 场景元素
- 🟥 红色立方体 - 可移动目标物体
- 🟢 绿色半透明球 - A位置标记（接近点）
- 🔵 蓝色半透明球 - B位置标记（推动终点）
- 🟢 末端绿球 - 力传感器
- 🌈 RGB坐标轴 - 世界坐标系

---

## 🛠️ 技术栈

| 组件 | 技术 | 版本 |
|------|------|------|
| 物理引擎 | MuJoCo | 3.3.4+ |
| 逆运动学 | ikpy | 3.3.0+ |
| 坐标变换 | transforms3d | 0.4.1+ |
| 数值计算 | NumPy | 1.21.0+ |
| 数据可视化 | Matplotlib | 3.5.0+ |

---

## 📂 项目结构

```
tutorial_for_mujoco/
├── main.py                    # 主程序（155行）
├── requirements.txt           # 依赖列表
├── model/                     # 模型文件
│   ├── ur5e.urdf             # URDF机器人描述（IK求解用）
│   └── universal_robots_ur5e/
│       ├── scene.xml         # 完整仿真场景
│       ├── ur5e.xml          # 机械臂MuJoCo模型
│       └── assets/           # 3D网格文件
└── docs/                      # 完整中文文档
    ├── 01_环境配置/          # 安装配置指南
    ├── 02_快速入门.md        # 入门教程
    ├── 03_项目结构概览.md    # 技术架构
    ├── 04_基础知识/          # 理论基础
    ├── 05_实战教程/          # 实践案例
    ├── 06_参考手册/          # API和参数
    └── 附录/                  # 变更日志等
```

---

## 🎯 学习路径

### 路径1：快速上手（1-2小时）
```
环境配置 → 快速入门 → 运行示例 → 修改参数
```

### 路径2：深入理解（1-2天）
```
项目结构 → 坐标系统 → 运动规划原理 → 两阶段运动 → 参数调整
```

### 路径3：扩展开发（1周+）
```
完整阅读文档 → 实战教程 → 自定义功能 → 贡献代码
```

---

## 🔧 常见问题

### WSL2环境问题

**Q: Qt platform plugin "xcb" 错误**
```bash
cd docs/01_环境配置/wsl_setup
./install_qt_deps.sh
```

**Q: OpenGL错误**
```bash
unset LIBGL_ALWAYS_INDIRECT
export DISPLAY=:0
```

更多问题请查看 [故障排除文档](docs/06_参考手册/故障排除.md)

---

## 🤝 贡献

欢迎提交Issue和Pull Request！

### 贡献指南
1. Fork本项目
2. 创建特性分支 (`git checkout -b feature/AmazingFeature`)
3. 提交变更 (`git commit -m '添加某个功能'`)
4. 推送到分支 (`git push origin feature/AmazingFeature`)
5. 创建Pull Request

---

## 📄 许可证

本项目采用 MIT 许可证 - 详见 [LICENSE](LICENSE) 文件

---

## 🙏 致谢

- [MuJoCo](https://mujoco.org/) - 强大的物理仿真引擎
- [Universal Robots](https://www.universal-robots.com/) - UR5e机械臂
- [ikpy](https://github.com/Phylliade/ikpy) - Python逆运动学库
- [mujoco_menagerie](https://github.com/deepmind/mujoco_menagerie) - 机器人模型库

---

## 📮 联系方式

- 项目主页: [GitHub Repository](https://github.com/yourusername/tutorial_for_mujoco)
- 问题反馈: [Issues](https://github.com/yourusername/tutorial_for_mujoco/issues)
- 原始博客: CSDN《MuJoCo 全流程实战教程：从零搭建一个仿真实验》

---

**⭐ 如果这个项目对您有帮助，请给一个Star！**

**最后更新**: 2026-07-23 | **版本**: v2.0.0 | **文档**: 完整中文
