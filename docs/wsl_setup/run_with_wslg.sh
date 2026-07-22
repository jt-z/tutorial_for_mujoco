#!/bin/bash
# Test and run MuJoCo with WSLg

echo "Installing mesa-utils to check OpenGL..."
sudo apt-get install -y mesa-utils

echo ""
echo "Checking OpenGL version..."
glxinfo | grep "OpenGL version"

echo ""
echo "Running MuJoCo simulation..."
unset LIBGL_ALWAYS_INDIRECT
export DISPLAY=:0
python main.py
