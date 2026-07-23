# 环境配置指南

本目录包含在不同环境下配置和运行 MuJoCo 可视化所需的所有文档和脚本。

---

## 📋 文档列表

- **[WSL2环境配置](./环境配置总结.md)** - WSL2环境配置完整指南，包含问题排查和解决方案
- **[项目技术分析](./项目分析.md)** - MuJoCo教程项目的详细技术分析

---

## 🚀 快速开始

### 方式1：WSL2环境（推荐）

#### 1. 确认WSL2版本
```bash
wsl --version
# 需要WSL 2.0+，支持WSLg（图形界面）
```

#### 2. 安装系统依赖
```bash
cd docs/01_环境配置
chmod +x install_qt_deps.sh
./install_qt_deps.sh
```

#### 3. 安装Python依赖
```bash
cd ~/dev/On_Git_Projects/tutorial_for_mujoco
pip install -r requirements.txt
```

#### 4. 运行仿真
```bash
# 方式A: 使用便捷脚本
cd docs/01_环境配置
chmod +x run_with_wslg.sh
./run_with_wslg.sh

# 方式B: 手动运行
cd ~/dev/On_Git_Projects/tutorial_for_mujoco
unset LIBGL_ALWAYS_INDIRECT
export DISPLAY=:0
python main.py
```

---

### 方式2：原生Linux环境

#### 1. 安装系统依赖
```bash
# Ubuntu/Debian
sudo apt update
sudo apt install -y \
    python3-pip \
    libgl1-mesa-glx \
    libglib2.0-0 \
    libxcb-xinerama0 \
    libxcb-cursor0

# Fedora/RHEL
sudo dnf install -y \
    python3-pip \
    mesa-libGL \
    glib2 \
    libxcb
```

#### 2. 安装Python依赖
```bash
pip3 install -r requirements.txt
```

#### 3. 运行程序
```bash
python3 main.py
```

---

### 方式3：Windows原生环境

#### 1. 安装Python
- 从 [python.org](https://www.python.org/downloads/) 下载Python 3.8+
- 安装时勾选 "Add Python to PATH"

#### 2. 安装依赖
```cmd
cd C:\path\to\tutorial_for_mujoco
pip install -r requirements.txt
```

#### 3. 运行程序
```cmd
python main.py
```

---

### 方式4：macOS环境

#### 1. 安装Homebrew（如未安装）
```bash
/bin/bash -c "$(curl -fsSL https://raw.githubusercontent.com/Homebrew/install/HEAD/install.sh)"
```

#### 2. 安装Python
```bash
brew install python@3.11
```

#### 3. 安装依赖
```bash
pip3 install -r requirements.txt
```

#### 4. 运行程序
```bash
python3 main.py
```

---

## 📦 依赖包说明

### requirements.txt 内容
```txt
mujoco>=3.3.0          # MuJoCo物理引擎
numpy>=1.21.0          # 数值计算
ikpy>=3.3.0            # 逆运动学求解
transforms3d>=0.4.1    # 坐标变换
matplotlib>=3.5.0      # 数据可视化
```

### 版本兼容性
- **Python**: 3.8 - 3.11（推荐3.10）
- **MuJoCo**: 3.3.0+
- **操作系统**: Linux, Windows, macOS

---

## 🐛 常见问题

### Q1: Qt platform plugin "xcb" 错误

**错误信息：**
```
qt.qpa.plugin: Could not load the Qt platform plugin "xcb"
```

**解决方法：**
```bash
# 运行安装脚本
cd docs/01_环境配置
./install_qt_deps.sh

# 或手动安装
sudo apt install -y \
    libxcb-xinerama0 \
    libxcb-cursor0 \
    libxkbcommon-x11-0
```

### Q2: OpenGL相关错误

**错误信息：**
```
libGL error: failed to load driver: swrast
```

**解决方法（WSL2）：**
```bash
# 取消间接渲染
unset LIBGL_ALWAYS_INDIRECT

# 设置正确的DISPLAY
export DISPLAY=:0
```

### Q3: 找不到mujoco模块

**错误信息：**
```
ModuleNotFoundError: No module named 'mujoco'
```

**解决方法：**
```bash
pip install mujoco --upgrade
```

### Q4: GLFW窗口无法创建

**解决方法（WSL2）：**
```bash
# 确认WSLg运行
ps aux | grep Xwayland

# 如未运行，重启WSL
wsl --shutdown
# 重新打开WSL终端
```

---

## 🔧 脚本说明

### install_qt_deps.sh
自动安装Qt xcb平台插件所需的所有系统依赖。

**用法：**
```bash
chmod +x install_qt_deps.sh
./install_qt_deps.sh
```

### run_with_wslg.sh
配置环境变量并启动MuJoCo仿真程序的便捷脚本。

**用法：**
```bash
chmod +x run_with_wslg.sh
./run_with_wslg.sh
```

**脚本内容：**
```bash
#!/bin/bash
cd ~/dev/On_Git_Projects/tutorial_for_mujoco
unset LIBGL_ALWAYS_INDIRECT
export DISPLAY=:0
python main.py
```

---

## ✅ 验证安装

运行以下命令验证环境配置是否成功：

### 1. 检查Python版本
```bash
python --version
# 应显示：Python 3.8.x 或更高
```

### 2. 检查依赖包
```bash
python -c "import mujoco; print(mujoco.__version__)"
# 应显示：3.3.x

python -c "import ikpy; print(ikpy.__version__)"
# 应显示：3.3.x 或更高
```

### 3. 测试MuJoCo渲染
```bash
python -c "import mujoco.viewer; print('MuJoCo viewer OK')"
# 应显示：MuJoCo viewer OK
```

### 4. 运行完整程序
```bash
cd ~/dev/On_Git_Projects/tutorial_for_mujoco
python main.py
# 应打开两个窗口（3D仿真 + 力向量图）
```

---

## 🌐 网络代理配置（可选）

如果您在国内且遇到下载速度慢的问题：

### pip使用国内镜像
```bash
pip install -r requirements.txt -i https://pypi.tuna.tsinghua.edu.cn/simple
```

### 配置永久镜像源
```bash
pip config set global.index-url https://pypi.tuna.tsinghua.edu.cn/simple
```

---

## 📚 相关文档

- **完整配置说明**: [环境配置总结.md](./环境配置总结.md)
- **技术分析**: [项目分析.md](./项目分析.md)
- **快速入门**: [../02_快速入门.md](../02_快速入门.md)

---

## 🆘 获取帮助

如果遇到本文档未涵盖的问题：

1. 查看 [环境配置总结.md](./环境配置总结.md) 中的详细故障排查
2. 查看 [../06_参考手册/故障排除.md](../06_参考手册/故障排除.md)
3. 在GitHub项目中提交Issue

---

**最后更新**: 2026-07-23  
**适用系统**: WSL2, Linux, Windows, macOS  
**测试通过**: Ubuntu 22.04, WSL2 (Windows 11)
