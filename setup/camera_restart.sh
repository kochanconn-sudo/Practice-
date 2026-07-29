#!/bin/bash
# カメラサービスのリスタートスクリプト

echo "Restarting nvargus-daemon..."
sudo systemctl restart nvargus-daemon
sleep 2

echo "Checking daemon status..."
sudo systemctl status nvargus-daemon | head -10

echo "Resetting camera modules..."
# カメラモジュールのリセット
sudo modprobe -r imx219
sleep 1
sudo modprobe imx219

echo "Testing cameras..."
for i in 0 1; do
    echo "Testing camera $i..."
    timeout 3 gst-launch-1.0 nvarguscamerasrc sensor-id=$i num-buffers=1 ! fakesink 2>&1 | grep -E "(Setting pipeline|Error)"
done