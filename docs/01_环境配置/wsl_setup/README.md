# WSL 环境配置指南

本目录包含在 WSL2 环境下配置和运行 MuJoCo 可视化所需的所有文档和脚本。

## 文档列表

- **[项目分析.md](./项目分析.md)** - MuJoCo 教程项目的详细分析，包括代码结构、功能说明和技术架构
- **[环境配置总结.md](./环境配置总结.md)** - WSL2 环境配置完整指南，包含问题排查和解决方案

## 脚本列表

- **[install_qt_deps.sh](./install_qt_deps.sh)** - 安装 Qt xcb 平台插件所需的系统依赖
- **[run_with_wslg.sh](./run_with_wslg.sh)** - 使用 WSLg 运行 MuJoCo 仿真的便捷脚本

## 快速开始

### 1. 安装系统依赖

```bash
cd docs/wsl_setup
./install_qt_deps.sh
```

### 2. 运行仿真

```bash
# 方式 1: 使用脚本
./run_with_wslg.sh

# 方式 2: 手动运行
cd ~/dev/On_Git_Projects/tutorial_for_mujoco
unset LIBGL_ALWAYS_INDIRECT
export DISPLAY=:0
python main.py
```

## 相关问题

如果遇到问题，请参考 [环境配置总结.md](./环境配置总结.md) 中的故障排查章节。
