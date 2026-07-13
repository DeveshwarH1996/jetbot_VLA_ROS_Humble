#!/bin/bash
set -e

echo "--- JetBot Vision Setup: TensorRT & YOLOv8 ---"

# 1. Verify JetPack installations
if [ -f /etc/nv_tegra_release ]; then
    echo "✅ Jetson Hardware detected: $(cat /etc/nv_tegra_release)"
else
    echo "❌ Error: This script must be run on an NVIDIA Jetson device."
    exit 1
fi

# 2. Install basic dependencies
echo "Installing system dependencies..."
sudo apt-get update
sudo apt-get install -y python3-pip libopencv-dev

# 3. Install Ultralytics (YOLOv8)
echo "Installing Ultralytics..."
pip3 install ultralytics

# 4. TensorRT Check
# Note: TensorRT is usually installed via JetPack (SDK Manager/Nvidia-l4t).
# If missing, the user needs to run 'sudo apt install nvidia-tensorrt' or use SDK Manager.
if python3 -c "import tensorrt" &> /dev/null; then
    echo "✅ TensorRT Python module found."
else
    echo "⚠️ TensorRT module not found. Please ensure JetPack is fully installed."
    echo "Try: sudo apt-get install nvidia-tensorrt"
fi

echo "----------------------------------------------------------------"
echo "SETUP COMPLETE: To optimize the model for your GPU, run:"
echo "python3 -c \"from ultralytics import YOLO; model = YOLO('yolov8n.pt'); model.export(format='engine')\""
echo "This will create 'yolov8n.engine' in your current directory."
echo "----------------------------------------------------------------"
