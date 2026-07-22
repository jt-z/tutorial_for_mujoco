#!/bin/bash
# Install Qt xcb dependencies for MuJoCo GUI

echo "Installing Qt xcb platform plugin dependencies..."

sudo apt-get update

sudo apt-get install -y \
    libxcb-xinerama0 \
    libxcb-cursor0 \
    libxkbcommon-x11-0 \
    libxcb-icccm4 \
    libxcb-image0 \
    libxcb-keysyms1 \
    libxcb-render-util0 \
    libxcb-shape0 \
    libdbus-1-3 \
    libxcb-xkb1 \
    libxkbcommon0 \
    libgl1-mesa-glx \
    libglib2.0-0

echo "Installation complete!"
echo ""
echo "Now try running:"
echo "  export DISPLAY=172.26.64.1:0.0"
echo "  export LIBGL_ALWAYS_INDIRECT=1"
echo "  python main.py"
