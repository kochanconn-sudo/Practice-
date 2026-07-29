# monitor.py

import tornado.ioloop
import tornado.web
import tornado.websocket
import tornado.gen
import tornado.concurrent
import asyncio
import time
import cv2
import config  # config.pyの定数を更新するため
import threading
from datetime import datetime
from pytz import timezone as _tz
_jst = _tz('Asia/Tokyo')
import logging
import os
import numpy as np
import json
import base64
from concurrent.futures import ThreadPoolExecutor
import webbrowser
import math
import platform

# 走行中に取得されるセンサーデータやステアリング値などを保持する辞書
# run.py のメインループから update_data() を介して書き込まれる想定
realtime_data = {
    "mode": None,
    "steering_value": 0.0,
    "throttle_value": 0.0,
    "ranges": {},  # 例: {"Fr": 100.0, "FrLH": 50.0, ...}
    "imu_data": None,
    "timestamp": None,
    # 画像フレーム (numpy配列) はここに入る
    "camera_image_0": None,
    "camera_image_1": None,
    "camera_image_2": None,
    "camera_image_3": None,

    # LiDAR点群・FTG表示用
    "lidar_measurements": None,
    "ftg_info": None,
    "lidar_config": None,

    # 追加センサー
    "imu_yaw_rate": None,
    "imu_accel": None,
    "rpm": None,
    "optical_flow_speed": None,

    # 走行一時停止用のフラグ例
    "pause_drive": False,

    # ループ情報
    "record_count": 0,
    "fps": None,
}

# setconfigでの再ロード用
set_config_reload = False

# 終了シグナル用フラグ
shutdown_signal = False

# 単独起動時のカメラインスタンス
camera_instances = {}
camera_update_thread = None

# 単独起動時のセンサーインスタンス
active_sensor_instances = {}
data_aggregator_instance = None

# WebSocketクライアント管理
websocket_clients = set()

# スレッドプール
executor = ThreadPoolExecutor(max_workers=4)

# 画像エンコード用のロック
image_lock = threading.Lock()

# 画像キャッシュ（フレーム変更検出用）
last_frame_hash = None
last_encoded_image = None

# 高速化用の設定
FAST_MODE = True  # 高速モードフラグ
SKIP_FRAME_COUNT = 0  # フレームスキップ数（0=全フレーム処理）
frame_skip_counter = 0


#------------------------------------------------------------------------------#
# HTMLテンプレートを動的に生成（templatesフォルダが無い場合の対応）
#------------------------------------------------------------------------------#
def create_template_if_needed():
    """templatesフォルダとmonitor.htmlを作成（存在しない場合）"""
    templates_dir = "templates"
    if not os.path.exists(templates_dir):
        os.makedirs(templates_dir)
        print(f"Created {templates_dir} directory")

    index_html_path = os.path.join(templates_dir, "monitor.html")
    if not os.path.exists(index_html_path):
        # センサー可視化対応のHTMLテンプレートを作成
        html_content = '''<!DOCTYPE html>
<html>
<head>
    <meta charset="utf-8">
    <title>Togikaidrive Monitor</title>
    <style>
        body {
            font-family: 'Arial', sans-serif;
            margin: 0;
            padding: 0;
            background-color: #f4f4f9;
            color: #333;
            width: 100vw;
            height: 100vh;
            overflow: hidden;
        }
        .main-wrapper {
            display: flex;
            flex-direction: column;
            width: 100%;
            height: 100vh;
        }
        header {
            background-color: #3b5998;
            color: white;
            padding: 16px;
            text-align: center;
        }
        h1 {
            margin: 0;
            font-size: 24px;
        }
        .container {
            display: flex;
            flex-wrap: nowrap;
            padding: 8px;
            flex: 7;
            min-height: 0;
            position: relative;
        }
        .camera-container {
            flex: 7;
            display: flex;
            align-items: center;
            justify-content: center;
            position: relative;
            background-color: #000;
            border-radius: 8px;
            overflow: hidden;
            min-width: 200px;
        }
        #cameraFeed {
            width: 100%;
            height: 100%;
            object-fit: contain;
            border-radius: 8px;
        }
        #overlayCanvas {
            position: absolute;
            top: 0;
            left: 0;
            width: 100%;
            height: 100%;
            pointer-events: none;
        }
        .control-container {
            flex: 3;
            background: white;
            border: 1px solid #ddd;
            border-radius: 8px;
            padding: 6px 8px;
            box-shadow: 0 4px 6px rgba(0, 0, 0, 0.1);
            overflow-y: auto;
            min-width: 220px;
        }
        .form-grid {
            display: grid;
            grid-template-columns: 1fr 1fr;
            gap: 3px 6px;
        }
        .form-grid-full {
            grid-column: 1 / -1;
        }
        .control-container h2 {
            font-size: 13px;
            border-bottom: 2px solid #3b5998;
            margin: 0 0 4px;
            padding-bottom: 2px;
        }
        .buttons {
            display: flex;
            justify-content: space-between;
            margin-bottom: 4px;
        }
        button {
            padding: 4px 8px;
            font-size: 11px;
            border: none;
            border-radius: 4px;
            cursor: pointer;
            transition: background 0.3s, color 0.3s;
        }
        #setConfigButton {
            background-color: #f9c74f;
            color: black;
            width: 100%;
            margin-top: 4px;
            padding: 4px 8px;
            font-size: 11px;
        }
        #setConfigButton:disabled {
            background-color: #f4f4f9;
            color: #999;
            cursor: not-allowed;
        }
        button.active {
            background-color: #3b5998;
            color: white;
        }
        button.inactive {
            background-color: #ddd;
            color: #666;
        }
        .form-row {
            display: flex;
            align-items: center;
            gap: 4px;
            margin-bottom: 0;
        }
        .form-row label {
            flex: 0 0 72px;
            font-size: 10px;
            margin: 0;
            color: #555;
            text-align: right;
            overflow: hidden;
            text-overflow: ellipsis;
            white-space: nowrap;
        }
        .form-row input,
        .form-row select {
            flex: 1;
            min-width: 0;
        }
        label {
            font-size: 10px;
            display: block;
            margin-bottom: 1px;
            color: #555;
        }
        input, select {
            width: 100%;
            padding: 2px 4px;
            font-size: 11px;
            border: 1px solid #ccc;
            border-radius: 3px;
        }
        .realtime-data {
            padding: 8px;
            background: white;
            border: 1px solid #ddd;
            border-radius: 8px;
            box-shadow: 0 4px 6px rgba(0, 0, 0, 0.1);
            margin: 8px;
            flex: 3 1 80px;
            overflow: hidden;
            display: flex;
            justify-content: center;
            align-items: stretch;
            gap: 8px;
            min-height: 80px;
            width: 100%;
            box-sizing: border-box;
        }
        .realtime-data-left {
            flex: 1 1 0;
            min-width: 100px;
        }
        .realtime-data-right {
            flex: 1 1 0;
        }
        .realtime-data h2 {
            margin: 0 0 8px;
            font-size: 16px;
            border-bottom: 2px solid #3b5998;
            display: inline-block;
            padding-bottom: 4px;
        }
        .realtime-data div {
            margin-bottom: 4px;
            font-size: 12px;
        }
        #ultrasonicChart {
            margin-top: 5px;
            height: 60px !important;
        }
        #configUpdateResult {
            margin-top: 2px;
            font-size: 10px;
            color: green;
        }
        /* Resizer styles */
        .resizer-horizontal {
            width: 8px;
            cursor: col-resize;
            background-color: #ddd;
            position: relative;
            user-select: none;
            transition: background-color 0.2s;
        }
        .resizer-horizontal:hover {
            background-color: #3b5998;
        }
        .resizer-vertical {
            height: 8px;
            cursor: row-resize;
            background-color: #ddd;
            position: relative;
            user-select: none;
            transition: background-color 0.2s;
        }
        .resizer-vertical:hover {
            background-color: #3b5998;
        }
    </style>
</head>
<body>
    <div class="main-wrapper">
        <div class="container">
            <!-- カメラ映像 -->
            <div class="camera-container">
                <img id="cameraFeed" alt="Camera feed" style="display: none;">
                <canvas id="overlayCanvas"></canvas> <!-- This canvas will hold the arrows -->
            </div>

            <!-- 水平リサイザー -->
            <div class="resizer-horizontal" id="horizontalResizer"></div>

            <!-- Control & Config -->
            <div class="control-container">
            <h2>Control & Config</h2>
            <div class="buttons">
                <button id="pauseButton" class="inactive" onclick="toggleDriveControl('pause')">Pause</button>
                <button id="resumeButton" class="inactive" onclick="toggleDriveControl('resume')">Resume</button>
            </div>
            <form id="configForm">
                <div class="form-grid">
                    <div class="form-row form-grid-full">
                        <label title="走行プラン">PLAN</label>
                        <select name="PLAN" id="planSelect" onchange="updateModelOptions(this.value)"></select>
                    </div>
                    <div class="form-row form-grid-full">
                        <label title="使用するAIモデルファイル">MODEL_NAME</label>
                        <select id="modelSelect" name="MODEL_NAME"></select>
                    </div>
                    <div class="form-row form-grid-full">
                        <label title="wall_follow時の壁沿い方向">HAND_SIDE</label>
                        <select name="HAND_SIDE" id="handSideSelect">
                            <option value="right">Right</option>
                            <option value="left">Left</option>
                        </select>
                    </div>
                    <div class="form-row">
                        <label title="直進時のスロットル値 (0~1)">FWD_STRAIGHT</label>
                        <input type="text" name="FORWARD_STRAIGHT">
                    </div>
                    <div class="form-row">
                        <label title="カーブ時のスロットル値 (0~1)">FWD_CORNER</label>
                        <input type="text" name="FORWARD_CORNER">
                    </div>
                    <div class="form-row">
                        <label title="停止判定距離 (mm)">STOP_RANGE</label>
                        <input type="text" name="STOP_RANGE">
                    </div>
                    <div class="form-row">
                        <label title="後退判定距離 (mm)">BACKWARD_R</label>
                        <input type="text" name="BACKWARD_RANGE">
                    </div>
                    <div class="form-row">
                        <label title="障害物検知開始距離 (mm)">DETECTION_R</label>
                        <input type="text" name="DETECTION_RANGE">
                    </div>
                    <div class="form-row">
                        <label title="右左折判定基準距離 (mm)">RL_RANGE</label>
                        <input type="text" name="RIGHT_LEFT_RANGE">
                    </div>
                    <div class="form-row">
                        <label title="壁沿い走行の目標距離 (mm)">TARGET_R</label>
                        <input type="text" name="TARGET_RANGE">
                    </div>
                    <div class="form-row">
                        <label title="目標距離の許容誤差 (±mm)">TARGET_ADJ</label>
                        <input type="text" name="TARGET_RANGE_ADJUSTMENT">
                    </div>
                    <div class="form-row">
                        <label title="PID比例ゲイン">K_P</label>
                        <input type="text" name="K_P">
                    </div>
                    <div class="form-row">
                        <label title="PID積分ゲイン">K_I</label>
                        <input type="text" name="K_I">
                    </div>
                    <div class="form-row">
                        <label title="PID微分ゲイン">K_D</label>
                        <input type="text" name="K_D">
                    </div>
                </div>
                <button id="setConfigButton" type="button" onclick="setConfig()" disabled>Set Config</button>
            </form>
            <div id="configUpdateResult"></div>
        </div>
    </div>

    <!-- 垂直リサイザー -->
    <div class="resizer-vertical" id="verticalResizer"></div>

    <!-- Realtime Data -->
    <div class="realtime-data">
        <!-- 左側のデータ表示 -->
        <div class="realtime-data-left">
            <h2>Realtime Data</h2>
            <div>Mode: <span id="mode"></span></div>
            <div>Steering Value: <span id="steering_value"></span></div>
            <div>Throttle Value: <span id="throttle_value"></span></div>
            <div>Timestamp: <span id="timestamp"></span></div>
            <div>Records: <span id="record_count">0</span></div>
            <div>FPS: <span id="fps_value">-</span></div>
        </div>

        <!-- 中央のコントロール表示（横並び） -->
        <div class="realtime-data-center" style="flex: 0 0 auto; display: flex; flex-direction: row; align-items: center; gap: 4px;">
            <div style="text-align: center;">
                <div style="font-size: 12px; font-weight: bold;">Steering</div>
                <canvas id="steeringCanvas" width="100" height="100" style="border: 1px solid #ccc; border-radius: 50%;"></canvas>
                <div id="steeringValueLabel" style="font-size: 12px; font-family: monospace; color: #333;">0.00</div>
            </div>
            <div style="text-align: center;">
                <div style="font-size: 12px; font-weight: bold;">Throttle</div>
                <canvas id="throttleCanvas" width="60" height="100" style="border: 1px solid #ccc; border-radius: 5px;"></canvas>
                <div id="throttleValueLabel" style="font-size: 12px; font-family: monospace; color: #333;">0.00</div>
            </div>
            <div style="text-align: center;">
                <div style="font-size: 12px; font-weight: bold;">IMU</div>
                <canvas id="imuCanvas" width="100" height="100" style="border: 1px solid #ccc; border-radius: 50%;"></canvas>
                <div id="imuValueLabel" style="font-size: 12px; font-family: monospace; color: #333; min-width:60px; text-align:center;">NA</div>
            </div>
            <div style="display: flex; flex-direction: column; justify-content: center; gap: 4px; font-family: monospace; font-size: 12px; color: #333; min-width: 50px;">
                <div style="text-align: center;">
                    <div style="font-weight: bold;">RPM</div>
                    <div id="rpmValueLabel" style="color: #666;">NA</div>
                </div>
                <div style="text-align: center;">
                    <div style="font-weight: bold;">Flow</div>
                    <div id="flowValueLabel" style="color: #666;">NA</div>
                </div>
            </div>
        </div>

        <!-- 右側のセンサー可視化表示（2列レイアウト） -->
        <div class="realtime-data-right" style="display:flex; flex-direction:row; gap:0; align-items:stretch; background:#000;">
            <!-- 左列: センサー値 -->
            <div id="sensorInfoColumn" style="flex:0 0 auto; min-width:90px; display:flex; flex-direction:column; font-size:10px; gap:2px; background:#000; padding:4px 0 4px 4px; color:#ccc; overflow:hidden;">
                <div style="font-weight:bold; font-size:10px; color:#999; margin-bottom:1px;">Ranges</div>
                <div id="sensorValuesPanel" style="font-family:monospace; font-size:11px; line-height:1.4; display:flex; flex-wrap:wrap; gap:0 8px;"></div>
            </div>
            <!-- 中列: センサー可視化キャンバス（1:1アスペクト比） -->
            <div id="sensorCanvasWrap" style="flex:1; min-width:0; display:flex; align-items:center; justify-content:center; background:#000;">
                <canvas id="sensorCanvas" width="200" height="200"></canvas>
            </div>
            <!-- 右列: Zoomスライダー（縦向き） -->
            <div id="lidarZoomColumn" style="display:none; flex:0 0 24px; background:#000; padding:2px 2px; color:#ccc; font-size:9px; align-items:center; justify-content:center; flex-direction:column; gap:0;">
                <div style="white-space:nowrap; font-family:monospace; text-align:center; line-height:1.2;">Zoom<br><span id="lidarZoomLabel">85</span></div>
                <input type="range" id="lidarZoomSlider" min="20" max="1000" value="85" orient="vertical" style="writing-mode:vertical-lr; direction:rtl; height:100%; min-height:40px; width:18px; flex:1;">
            </div>
        </div>
    </div> <!-- end of main-wrapper -->

    <script>
        // Fetch models (Promiseを返す)
        function fetchModels() {
            return fetch('/get_models')
                .then(res => {
                    if (!res.ok) {
                        throw new Error('Failed to fetch models');
                    }
                    return res.json();
                })
                .then(data => {
                    // data.models を返す
                    return data.models;
                })
                .catch(err => {
                    console.error('Error fetching models:', err);
                    throw err;
                });
        }

        // Update MODEL_NAME options when PLAN changes
        function updateModelOptions(plan) {
            fetchModels()
                .then(models => {
                    const modelSelect = document.getElementById('modelSelect');
                    modelSelect.innerHTML = ''; // Clear existing options

                    // modelsが取れなかった場合のフォールバック
                    if (!models || !Array.isArray(models)) {
                        const noModelOption = document.createElement('option');
                        noModelOption.value = '';
                        noModelOption.textContent = 'No models available';
                        modelSelect.appendChild(noModelOption);
                        return;
                    }

                    // PLANに応じて絞り込み（ファイル名プレフィックスで判定）
                    const planPrefixMap = {
                        'nn': 'nn_',
                        'donkeycar': 'donkeycar_',
                        'resnet18': 'resnet18_',
                        'mobilevit_xxs': 'mobilevit_xxs_',
                        'edgenext_xx_small': 'edgenext_xx_small_',
                        'gru': 'gru_',
                        'tcn': 'tcn_',
                        'causal_cnn': 'causal_cnn_',
                    };
                    const prefix = planPrefixMap[plan];
                    const filteredModels = prefix
                        ? models.filter(model => model.startsWith(prefix))
                        : [];

                    // Populate MODEL_NAME options
                    const currentModel = document.querySelector('select[name="MODEL_NAME"]')?.dataset.current || '';
                    if (filteredModels.length > 0) {
                        filteredModels.forEach(model => {
                            const option = document.createElement('option');
                            option.value = model;
                            option.textContent = model;
                            if (model === currentModel) option.selected = true;
                            modelSelect.appendChild(option);
                        });
                    } else {
                        const noModelOption = document.createElement('option');
                        noModelOption.value = '';
                        noModelOption.textContent = 'No models available';
                        modelSelect.appendChild(noModelOption);
                    }
                })
                .catch(err => console.error('updateModelOptions error:', err));
        }

        // Handle PLAN changes and reload the system (現在未使用のようなので残す)
        function handlePlanChange(plan) {
            console.log('PLAN changed to:', plan);
            reloadPlan(plan);
        }

        // Reload the system with the selected PLAN (現在未使用のようなので残す)
        function reloadPlan(plan) {
            fetch('/reload_system', {
                method: 'POST',
                headers: { 'Content-Type': 'application/json' },
                body: JSON.stringify({ plan: plan })
            })
                .then(res => {
                    if (!res.ok) {
                        throw new Error('Failed to reload system');
                    }
                    return res.json();
                })
                .then(data => {
                    console.log('System reloaded:', data);
                    alert('System reloaded with new plan: ' + plan);
                })
                .catch(err => console.error('Error reloading system:', err));
        }

        // WebSocket 接続とハンドリング
        let websocket = null;
        let reconnectInterval = null;
        let isConnecting = false;
        let reconnectAttempts = 0;
        const maxReconnectAttempts = 5;

        function connectWebSocket() {
            if (isConnecting || (websocket && websocket.readyState === WebSocket.CONNECTING)) {
                console.log('WebSocket connection already in progress');
                return;
            }

            if (websocket && websocket.readyState === WebSocket.OPEN) {
                console.log('WebSocket already connected');
                return;
            }

            isConnecting = true;
            const protocol = window.location.protocol === 'https:' ? 'wss:' : 'ws:';
            const wsUrl = `${protocol}//${window.location.host}/ws`;

            console.log(`Connecting to WebSocket: ${wsUrl}`);
            websocket = new WebSocket(wsUrl);

            websocket.onopen = function() {
                console.log('WebSocket connected successfully');
                isConnecting = false;
                reconnectAttempts = 0;
                if (reconnectInterval) {
                    clearInterval(reconnectInterval);
                    reconnectInterval = null;
                }
            };

            websocket.onmessage = function(event) {
                try {
                    const data = JSON.parse(event.data);
                    handleWebSocketMessage(data);
                } catch (error) {
                    console.error('Error parsing WebSocket message:', error);
                }
            };

            websocket.onclose = function(event) {
                console.log(`WebSocket disconnected. Code: ${event.code}, Reason: ${event.reason}`);
                websocket = null;
                isConnecting = false;

                // 自動再接続（最大試行回数まで）
                if (reconnectAttempts < maxReconnectAttempts && !reconnectInterval) {
                    reconnectAttempts++;
                    console.log(`Attempting to reconnect... (${reconnectAttempts}/${maxReconnectAttempts})`);
                    reconnectInterval = setTimeout(() => {
                        reconnectInterval = null;
                        connectWebSocket();
                    }, 3000);
                } else if (reconnectAttempts >= maxReconnectAttempts) {
                    console.error('Max reconnection attempts reached. Please refresh the page.');
                }
            };

            websocket.onerror = function(error) {
                console.error('WebSocket error:', error);
                isConnecting = false;
            };
        }

        function handleWebSocketMessage(data) {
            switch (data.type) {
                case 'image':
                    updateCameraImage(data.data);
                    break;
                case 'sensor_data':
                    updateSensorData(data.data);
                    break;
                case 'control_response':
                    console.log('Control response:', data);
                    break;
                case 'config_response':
                    if (data.config) {
                        handleGetConfigResponse(data);
                    } else {
                        handleConfigResponse(data);
                    }
                    break;
                case 'error':
                    console.error('WebSocket error:', data.message);
                    break;
            }
        }

        function updateCameraImage(base64Data) {
            const img = document.getElementById('cameraFeed');
            img.src = 'data:image/jpeg;base64,' + base64Data;
            img.style.display = 'block';
        }

        function updateSensorData(data) {
            // デバッグ出力（最初の10回のみ）
            if (!updateSensorData.debugCount) updateSensorData.debugCount = 0;
            if (updateSensorData.debugCount < 10) {
                console.log(`[DEBUG ${updateSensorData.debugCount}] updateSensorData called:`, {
                    steering: data.steering_value,
                    throttle: data.throttle_value,
                    ultrasonicKeys: data.ultrasonic_ranges ? Object.keys(data.ultrasonic_ranges) : 'null/undefined',
                    ultrasonicValues: data.ultrasonic_ranges,
                    rangesKeys: data.ranges ? Object.keys(data.ranges) : 'null/undefined',
                    rangesValues: data.ranges
                });
                updateSensorData.debugCount++;
            }

            document.getElementById('mode').textContent = data.mode || 'N/A';
            document.getElementById('steering_value').textContent = (data.steering_value !== undefined && data.steering_value !== null) ? data.steering_value : 'N/A';
            document.getElementById('throttle_value').textContent = (data.throttle_value !== undefined && data.throttle_value !== null) ? data.throttle_value : 'N/A';
            document.getElementById('timestamp').textContent = data.timestamp || 'N/A';
            document.getElementById('record_count').textContent = (data.record_count !== undefined && data.record_count !== null) ? data.record_count : '0';
            document.getElementById('fps_value').textContent = (data.fps !== undefined && data.fps !== null) ? data.fps : '-';

            // ステアリングハンドルの更新
            const steeringValue = (data.steering_value !== undefined && data.steering_value !== null) ? data.steering_value : 0;
            drawSteeringWheel(steeringValue);
            var stLabel = document.getElementById('steeringValueLabel');
            if (stLabel) stLabel.textContent = Number(steeringValue).toFixed(2);

            // スロットルゲージの更新
            const throttleValue = (data.throttle_value !== undefined && data.throttle_value !== null) ? data.throttle_value : 0;
            drawThrottleGauge(throttleValue);
            var thLabel = document.getElementById('throttleValueLabel');
            if (thLabel) thLabel.textContent = Number(throttleValue).toFixed(2);

            // IMU/RPM/OpticalFlow の更新
            var imuYawRate = (data.imu_yaw_rate !== undefined && data.imu_yaw_rate !== null) ? data.imu_yaw_rate : null;
            var imuAccel = (data.imu_accel !== undefined && data.imu_accel !== null) ? data.imu_accel : null;
            drawImuGauge(imuYawRate, imuAccel);
            var imuLabel = document.getElementById('imuValueLabel');
            if (imuLabel) imuLabel.textContent = imuYawRate !== null ? imuYawRate + '°/s' : 'NA';
            var rpmLabel = document.getElementById('rpmValueLabel');
            if (rpmLabel) rpmLabel.textContent = (data.rpm !== undefined && data.rpm !== null) ? data.rpm : 'NA';
            var flowLabel = document.getElementById('flowValueLabel');
            if (flowLabel) flowLabel.textContent = (data.optical_flow_speed !== undefined && data.optical_flow_speed !== null) ? data.optical_flow_speed + 'mm/s' : 'NA';

            // センサー可視化の更新（ultrasonic_rangesまたはrangesを使用）
            const sensorRanges = data.ultrasonic_ranges || data.ranges || {};
            drawSensorVisualization(
                sensorRanges,
                data.lidar_measurements || null,
                data.lidar_config || null,
                data.ftg_info || null
            );
        }

        function sendWebSocketMessage(message) {
            if (websocket && websocket.readyState === WebSocket.OPEN) {
                websocket.send(JSON.stringify(message));
            } else {
                console.error('WebSocket is not connected');
            }
        }

        // Toggle Drive Control buttons
        function toggleDriveControl(action) {
            sendWebSocketMessage({
                type: 'control',
                action: action
            });

            const pauseButton = document.getElementById('pauseButton');
            const resumeButton = document.getElementById('resumeButton');
            const setConfigButton = document.getElementById('setConfigButton');

            if (action === 'pause') {
                pauseButton.classList.add('active');
                pauseButton.classList.remove('inactive');
                resumeButton.classList.add('inactive');
                resumeButton.classList.remove('active');
                setConfigButton.disabled = false;
            } else if (action === 'resume') {
                resumeButton.classList.add('active');
                resumeButton.classList.remove('inactive');
                pauseButton.classList.add('inactive');
                pauseButton.classList.remove('active');
                setConfigButton.disabled = true;
            }
        }

        // Set Config
        function setConfig() {
            const form = document.getElementById("configForm");
            const formData = {};

            Array.from(form.elements).forEach(elem => {
                if (elem.name) formData[elem.name] = elem.value;
            });

            sendWebSocketMessage({
                type: 'set_config',
                config: formData
            });
        }

        function handleConfigResponse(data) {
            if (data.status === "ok" || data.status === "partial_success") {
                document.getElementById("configUpdateResult").textContent
                    = "Updated keys: " + data.updated_keys.join(", ");
            } else {
                document.getElementById("configUpdateResult").textContent
                    = "Error: " + JSON.stringify(data);
            }
        }

        // Fetch config and set defaults (WebSocket経由で取得)
        function fetchConfigAndSetDefaults() {
            sendWebSocketMessage({
                type: 'get_config'
            });
        }

        function handleGetConfigResponse(data) {
            if (data.status === "ok" && data.config) {
                const config = data.config;

                // PLAN_LISTからselectのoptionを動的生成
                var planSelect = document.getElementById('planSelect');
                if (config.PLAN_LIST && planSelect) {
                    planSelect.innerHTML = '';
                    config.PLAN_LIST.forEach(function(p) {
                        var opt = document.createElement('option');
                        opt.value = p;
                        opt.textContent = p;
                        planSelect.appendChild(opt);
                    });
                }

                // 全inputフィールドにconfigの値を一括設定
                var inputKeys = ['FORWARD_STRAIGHT', 'FORWARD_CORNER', 'STOP_RANGE', 'BACKWARD_RANGE',
                    'DETECTION_RANGE', 'RIGHT_LEFT_RANGE', 'TARGET_RANGE', 'TARGET_RANGE_ADJUSTMENT',
                    'K_P', 'K_I', 'K_D'];
                inputKeys.forEach(function(key) {
                    var el = document.querySelector('input[name="' + key + '"]');
                    if (el && config[key] !== undefined && config[key] !== null) {
                        el.value = config[key];
                    }
                });
                if (config.PLAN && planSelect) {
                    planSelect.value = config.PLAN;
                    // 現在のMODEL_NAMEをdata属性に保存（updateModelOptionsで選択状態にするため）
                    var modelSelect = document.getElementById('modelSelect');
                    if (modelSelect && config.MODEL_NAME) {
                        modelSelect.dataset.current = config.MODEL_NAME;
                    }
                    updateModelOptions(config.PLAN);
                }
                if (config.HAND_SIDE) {
                    document.querySelector('select[name="HAND_SIDE"]').value = config.HAND_SIDE;
                }
            } else {
                console.error('Error fetching config:', data.message);
            }
        }


        // センサー可視化キャンバス（グローバル変数）
        let sensorCanvas = null;
        let sensorCtx = null;
        var lidarZoom = 85;

        // Zoomスライダーのイベント
        (function() {
            document.addEventListener('DOMContentLoaded', function() {
                var slider = document.getElementById('lidarZoomSlider');
                var label = document.getElementById('lidarZoomLabel');
                if (slider) {
                    slider.oninput = function() {
                        lidarZoom = parseInt(this.value);
                        label.textContent = lidarZoom;
                    };
                }
            });
        })();

        // センサー配置定義（configのZONE_INDEXから動的に設定、フォールバック用デフォルト）
        // canvas arc: 0=右, 時計回りが正, -PI/2=上=前方
        var sensorConfig = {
            FrFR: { angle: -Math.PI / 2, arc: Math.PI / 6, x: 0, y: 0 },
            FrLH: { angle: -Math.PI / 2 - Math.PI / 4, arc: Math.PI / 6, x: 0, y: 0 },
            FrRH: { angle: -Math.PI / 2 + Math.PI / 4, arc: Math.PI / 6, x: 0, y: 0 },
            RrLH: { angle: -Math.PI, arc: Math.PI / 6, x: 0, y: 0 },
            RrRH: { angle: 0, arc: Math.PI / 6, x: 0, y: 0 }
        };

        function updateSensorConfigFromLidar(lidarConfig) {
            if (!lidarConfig || !lidarConfig.zone_angles) return;
            var za = lidarConfig.zone_angles;
            Object.keys(za).forEach(function(name) {
                sensorConfig[name] = {
                    angle: za[name].center_rad,
                    arc: za[name].arc_rad,
                    x: 0, y: 0
                };
            });
        }

        function drawVehicle(centerX, centerY) {
            sensorCtx.save();
            sensorCtx.translate(centerX, centerY);

            // 三角形のみ（進行方向を示す）
            sensorCtx.fillStyle = '#ffffff';
            sensorCtx.beginPath();
            sensorCtx.moveTo(0, -8);
            sensorCtx.lineTo(-6, 5);
            sensorCtx.lineTo(6, 5);
            sensorCtx.closePath();
            sensorCtx.fill();

            sensorCtx.restore();
        }

        function getDistanceColor(distance) {
            if (distance < 300) {
                return '#ff3333'; // 赤（危険）
            } else if (distance < 600) {
                return '#ffcc00'; // 黄（警告）
            } else {
                return '#33ff66'; // 緑（安全）
            }
        }

        function drawSensorFan(centerX, centerY, sensorKey, distance, scaleOverride) {
            const config = sensorConfig[sensorKey];
            if (!config) return;

            sensorCtx.save();
            sensorCtx.translate(centerX + config.x, centerY + config.y);

            // scaleOverride: LiDARスケール(px/mm)が渡された場合はそれを使用
            const maxRange = scaleOverride ? (distance * scaleOverride) : Math.min(distance / 10, 80);

            // 扇型を描画
            sensorCtx.fillStyle = getDistanceColor(distance);
            sensorCtx.globalAlpha = 0.6;
            sensorCtx.beginPath();
            sensorCtx.moveTo(0, 0);
            sensorCtx.arc(0, 0, maxRange,
                   config.angle - config.arc / 2,
                   config.angle + config.arc / 2);
            sensorCtx.closePath();
            sensorCtx.fill();

            // 扇の輪郭
            sensorCtx.globalAlpha = 1.0;
            sensorCtx.strokeStyle = getDistanceColor(distance);
            sensorCtx.lineWidth = 1;
            sensorCtx.stroke();

            // 距離テキスト（小さなフォント）
            sensorCtx.fillStyle = '#cccccc';
            sensorCtx.font = '8px Arial';
            sensorCtx.textAlign = 'center';
            const textX = Math.cos(config.angle) * (maxRange + 10);
            const textY = Math.sin(config.angle) * (maxRange + 10);
            sensorCtx.fillText(`${distance}mm`, textX, textY);

            sensorCtx.restore();
        }

        function drawGrid(centerX, centerY, maxDistanceMm, maxRadiusPx) {
            sensorCtx.strokeStyle = '#444444';
            sensorCtx.lineWidth = 1;

            if (maxDistanceMm && maxRadiusPx) {
                // LiDARモード: 実距離リングを表示
                var stepMm = 500;
                if (maxDistanceMm > 5000) stepMm = 2000;
                else if (maxDistanceMm > 2000) stepMm = 1000;
                var scale = maxRadiusPx / maxDistanceMm;
                for (var d = stepMm; d <= maxDistanceMm; d += stepMm) {
                    var r = d * scale;
                    if (r > maxRadiusPx) break;
                    sensorCtx.beginPath();
                    sensorCtx.arc(centerX, centerY, r, 0, 2 * Math.PI);
                    sensorCtx.stroke();
                    sensorCtx.fillStyle = '#999999';
                    sensorCtx.font = '8px Arial';
                    sensorCtx.fillText((d >= 1000 ? (d/1000).toFixed(1)+'m' : d+'mm'), centerX + r + 3, centerY - 2);
                }
                // 十字線
                sensorCtx.beginPath();
                sensorCtx.moveTo(centerX - maxRadiusPx, centerY);
                sensorCtx.lineTo(centerX + maxRadiusPx, centerY);
                sensorCtx.moveTo(centerX, centerY - maxRadiusPx);
                sensorCtx.lineTo(centerX, centerY + maxRadiusPx);
                sensorCtx.stroke();
            } else {
                // 既存モード: 同心円グリッド（調整された間隔）
                for (let r = 20; r <= 100; r += 20) {
                    sensorCtx.beginPath();
                    sensorCtx.arc(centerX, centerY, r, 0, 2 * Math.PI);
                    sensorCtx.stroke();
                }
                // 十字線
                sensorCtx.beginPath();
                sensorCtx.moveTo(centerX - 100, centerY);
                sensorCtx.lineTo(centerX + 100, centerY);
                sensorCtx.moveTo(centerX, centerY - 100);
                sensorCtx.lineTo(centerX, centerY + 100);
                sensorCtx.stroke();
            }
        }

        function updateSensorValuesHTML(sensorData) {
            var panel = document.getElementById('sensorValuesPanel');
            var column = document.getElementById('sensorInfoColumn');
            if (!panel || !column) return;
            var sensorOrder = ['RrLH', 'FrLH', 'FrFR', 'FrRH', 'RrRH'];
            var items = [];
            sensorOrder.forEach(function(key) {
                var value = sensorData[key];
                if (value !== undefined) {
                    items.push({key: key, value: value});
                }
            });

            // 利用可能な縦幅を推定
            var availH = column.clientHeight - 8; // padding分
            var itemH = 16; // 1行あたりの高さ概算
            var needTwoCols = (items.length * itemH) > availH;

            if (needTwoCols) {
                column.style.minWidth = '170px';
                panel.style.cssText = 'font-family:monospace; font-size:11px; line-height:1.4; display:grid; grid-template-columns:1fr 1fr; gap:0 6px;';
            } else {
                column.style.minWidth = '90px';
                panel.style.cssText = 'font-family:monospace; font-size:11px; line-height:1.4; display:flex; flex-direction:column;';
            }

            var html = '';
            items.forEach(function(item) {
                var color = getDistanceColor(item.value);
                html += '<div style="color:' + color + '; white-space:nowrap;">' + item.key + ':' + item.value + '</div>';
            });
            panel.innerHTML = html;
        }

        function drawLidarPointCloud(centerX, centerY, measurements, lidarConfig, maxRadius) {
            var numPoints = measurements.length;
            var angleStart = lidarConfig.angle_start;
            var angleEnd = lidarConfig.angle_end;
            var angleOffset = lidarConfig.angle_offset || 0;
            var clockwise = lidarConfig.clockwise || false;
            var maxDist = lidarConfig.max_distance;
            var scale = maxRadius / maxDist;

            for (var i = 0; i < numPoints; i++) {
                var dist = measurements[i];
                if (dist <= 0 || dist > maxDist) continue;

                // 角度を生成（ドライバと同じロジック）
                var angle;
                if (clockwise) {
                    angle = angleEnd - (angleEnd - angleStart) * i / (numPoints - 1);
                } else {
                    angle = angleStart + (angleEnd - angleStart) * i / (numPoints - 1);
                }
                // オフセット適用
                angle = angle + angleOffset;
                // -180〜180に正規化
                angle = ((angle + 180) % 360 + 360) % 360 - 180;
                var angleRad = angle * Math.PI / 180;

                // 座標変換（上が前方: x=sin, y=-cos）— 0°=front
                // 標準数学座標(cos,sin)からの変換: 90°=front=up
                var px = centerX + dist * scale * Math.cos(angleRad);
                var py = centerY - dist * scale * Math.sin(angleRad);

                // 距離に応じた色（HSL: 赤0→黄60→緑120）
                var ratio = Math.min(dist / maxDist, 1.0);
                var hue = ratio * 120;
                sensorCtx.fillStyle = 'hsl(' + hue + ', 100%, 50%)';
                sensorCtx.fillRect(px - 1, py - 1, 2, 2);
            }
        }

        // FTG角度 → 標準数学角度(0=右,90=上=前方)への変換
        function ftgAngleToStdRad(ftgAngle, lidarConfig) {
            var offset = lidarConfig.angle_offset || 0;
            var cw = lidarConfig.clockwise || false;
            var stdDeg = cw ? (offset - ftgAngle) : (ftgAngle + offset);
            return stdDeg * Math.PI / 180;
        }

        function drawFtgOverlay(centerX, centerY, ftgInfo, lidarConfig, maxRadius) {
            var maxDist = lidarConfig.max_distance;
            var scale = maxRadius / maxDist;

            // 1. ギャップ領域（半透明シアン扇形）
            // 標準数学角度 → canvas arc角度（Y反転のため符号反転）
            var arcA = -ftgAngleToStdRad(ftgInfo.gap_start_angle, lidarConfig);
            var arcB = -ftgAngleToStdRad(ftgInfo.gap_end_angle, lidarConfig);
            var arcStart = Math.min(arcA, arcB);
            var arcEnd = Math.max(arcA, arcB);
            sensorCtx.save();
            sensorCtx.globalAlpha = 0.2;
            sensorCtx.fillStyle = '#00ffff';
            sensorCtx.beginPath();
            sensorCtx.moveTo(centerX, centerY);
            sensorCtx.arc(centerX, centerY, maxRadius, arcStart, arcEnd);
            sensorCtx.closePath();
            sensorCtx.fill();
            sensorCtx.restore();

            // 2. 目標矢印（黒アウトライン＋オレンジ塗り＋ターゲットドット）
            var targetAngleRad = ftgAngleToStdRad(ftgInfo.target_angle, lidarConfig);
            var arrowLen = Math.min(ftgInfo.target_distance * scale, maxRadius * 0.9);
            if (arrowLen < 20) arrowLen = 20;
            var endX = centerX + arrowLen * Math.cos(targetAngleRad);
            var endY = centerY - arrowLen * Math.sin(targetAngleRad);
            var baseAngle = Math.atan2(endY - centerY, endX - centerX);
            var headLen = 12;

            sensorCtx.save();
            // 矢印の線（黒アウトライン）
            sensorCtx.strokeStyle = '#000000';
            sensorCtx.lineWidth = 5;
            sensorCtx.lineCap = 'round';
            sensorCtx.beginPath();
            sensorCtx.moveTo(centerX, centerY);
            sensorCtx.lineTo(endX, endY);
            sensorCtx.stroke();
            // 矢印の線（オレンジ内側）
            sensorCtx.strokeStyle = '#ff6600';
            sensorCtx.lineWidth = 3;
            sensorCtx.beginPath();
            sensorCtx.moveTo(centerX, centerY);
            sensorCtx.lineTo(endX, endY);
            sensorCtx.stroke();

            // 矢じり（黒アウトライン＋オレンジ塗り）
            sensorCtx.beginPath();
            sensorCtx.moveTo(endX, endY);
            sensorCtx.lineTo(endX - headLen * Math.cos(baseAngle - 0.45), endY - headLen * Math.sin(baseAngle - 0.45));
            sensorCtx.lineTo(endX - headLen * Math.cos(baseAngle + 0.45), endY - headLen * Math.sin(baseAngle + 0.45));
            sensorCtx.closePath();
            sensorCtx.fillStyle = '#ff6600';
            sensorCtx.fill();
            sensorCtx.strokeStyle = '#000000';
            sensorCtx.lineWidth = 2;
            sensorCtx.stroke();

            // ターゲットドット（先端に白丸＋黒縁）
            sensorCtx.beginPath();
            sensorCtx.arc(endX, endY, 5, 0, 2 * Math.PI);
            sensorCtx.fillStyle = '#ffffff';
            sensorCtx.fill();
            sensorCtx.strokeStyle = '#ff6600';
            sensorCtx.lineWidth = 2;
            sensorCtx.stroke();
            sensorCtx.restore();

            // 3. 情報テキスト（右上）
            sensorCtx.save();
            sensorCtx.fillStyle = '#cccccc';
            sensorCtx.font = '9px Arial';
            sensorCtx.textAlign = 'right';
            var tx = sensorCanvas.width - 5;
            var ty = 12;
            sensorCtx.fillText('Tgt: ' + ftgInfo.target_angle.toFixed(1) + ' / ' + ftgInfo.target_distance.toFixed(0) + 'mm', tx, ty);
            sensorCtx.fillText('Near: ' + ftgInfo.closest_dist.toFixed(0) + 'mm  ' + ftgInfo.steering_method, tx, ty + 12);
            sensorCtx.restore();
        }

        function drawSensorVisualization(ultrasonicRanges, lidarMeasurements, lidarConfig, ftgInfo) {
            if (!sensorCanvas || !sensorCtx) {
                console.error('Sensor canvas not initialized');
                return;
            }

            // キャンバスを黒で塗りつぶし
            sensorCtx.fillStyle = '#000000';
            sensorCtx.fillRect(0, 0, sensorCanvas.width, sensorCanvas.height);

            var centerX = sensorCanvas.width / 2;
            var centerY = sensorCanvas.height / 2;

            // lidarConfigからセンサー配置を動的更新
            if (lidarConfig) updateSensorConfigFromLidar(lidarConfig);

            // Zoomスライダーの表示制御
            var zoomCol = document.getElementById('lidarZoomColumn');
            var hasLidar = !!(lidarMeasurements && lidarConfig);
            if (zoomCol) zoomCol.style.display = hasLidar ? 'flex' : 'none';

            // センサー値をHTMLパネルに表示
            var sensorData = (ultrasonicRanges && Object.keys(ultrasonicRanges).length > 0) ? ultrasonicRanges : null;
            if (sensorData) {
                updateSensorValuesHTML(sensorData);
            }

            if (hasLidar) {
                // LiDAR点群モード: キャンバス中央にセンタリング
                var maxRadius = lidarZoom;

                drawGrid(centerX, centerY, lidarConfig.max_distance, maxRadius);
                drawLidarPointCloud(centerX, centerY, lidarMeasurements, lidarConfig, maxRadius);

                // 超音波扇形も重ねて表示（LiDARと同じスケール）
                var fanScale = maxRadius / lidarConfig.max_distance;
                if (sensorData) {
                    Object.entries(sensorData).forEach(function(entry) {
                        var key = entry[0], value = entry[1];
                        if (sensorConfig[key] && value !== undefined) {
                            drawSensorFan(centerX, centerY, key, value, fanScale);
                        }
                    });
                }

                if (ftgInfo && lidarConfig.plan === 'follow_the_gap') {
                    drawFtgOverlay(centerX, centerY, ftgInfo, lidarConfig, maxRadius);
                }
                drawVehicle(centerX, centerY);
            } else {
                // 既存ゾーン扇形モード
                drawGrid(centerX, centerY);

                var defaultSensorValues = {
                    RrLH: 850, FrLH: 450, FrFR: 250, FrRH: 650, RrRH: 1200
                };
                var drawData = sensorData || defaultSensorValues;

                Object.entries(drawData).forEach(function(entry) {
                    var key = entry[0], value = entry[1];
                    if (sensorConfig[key] && value !== undefined) {
                        drawSensorFan(centerX, centerY, key, value);
                    }
                });

                drawVehicle(centerX, centerY);

                // HTMLパネルにデフォルト値も表示
                if (!sensorData) updateSensorValuesHTML(drawData);
            }
        }

        // センサーキャンバスを初期化・リサイズする関数
        function resizeSensorCanvas() {
            var wrap = document.getElementById('sensorCanvasWrap');
            var canvas = document.getElementById('sensorCanvas');
            if (!wrap || !canvas) return;
            var w = wrap.clientWidth;
            var h = wrap.clientHeight;
            if (w < 50) w = 50;
            if (h < 50) h = 50;
            canvas.width = w;
            canvas.height = h;
            canvas.style.width = w + 'px';
            canvas.style.height = h + 'px';
        }

        function initializeSensorCanvas() {
            sensorCanvas = document.getElementById('sensorCanvas');
            if (!sensorCanvas) {
                console.error('Sensor canvas element not found!');
                return;
            }
            resizeSensorCanvas();
            sensorCtx = sensorCanvas.getContext('2d');
            console.log('Sensor canvas initialized:', sensorCanvas.width, 'x', sensorCanvas.height);

            window.addEventListener('resize', function() {
                resizeSensorCanvas();
                sensorCtx = sensorCanvas.getContext('2d');
                drawSensorVisualization();
            });

            // 初期描画
            drawSensorVisualization();
        }

        // ステアリングハンドルを描画する関数
        function drawSteeringWheel(steeringValue) {
            const canvas = document.getElementById('steeringCanvas');
            if (!canvas) return;

            const ctx = canvas.getContext('2d');
            const centerX = canvas.width / 2;
            const centerY = canvas.height / 2;
            const radius = Math.min(canvas.width, canvas.height) / 2 - 2;

            // キャンバスをクリア
            ctx.clearRect(0, 0, canvas.width, canvas.height);

            // 回転角度を計算（-1 to 1 -> -90度 to 90度）
            const rotation = steeringValue * Math.PI / 2;

            ctx.save();
            ctx.translate(centerX, centerY);
            ctx.rotate(rotation);

            // ハンドル外枠
            ctx.beginPath();
            ctx.arc(0, 0, radius, 0, 2 * Math.PI);
            ctx.strokeStyle = '#333';
            ctx.lineWidth = 3;
            ctx.stroke();

            // ハンドル内部
            ctx.beginPath();
            ctx.arc(0, 0, radius - 5, 0, 2 * Math.PI);
            ctx.fillStyle = '#f0f0f0';
            ctx.fill();

            // 中央のハブ
            ctx.beginPath();
            ctx.arc(0, 0, 8, 0, 2 * Math.PI);
            ctx.fillStyle = '#666';
            ctx.fill();

            // スポーク（上下左右）
            ctx.strokeStyle = '#666';
            ctx.lineWidth = 2;
            ctx.beginPath();
            ctx.moveTo(0, -radius + 5);
            ctx.lineTo(0, -8);
            ctx.moveTo(0, 8);
            ctx.lineTo(0, radius - 5);
            ctx.moveTo(-radius + 5, 0);
            ctx.lineTo(-8, 0);
            ctx.moveTo(8, 0);
            ctx.lineTo(radius - 5, 0);
            ctx.stroke();

            // 上方向インジケーター（12時の位置）
            ctx.beginPath();
            ctx.arc(0, -radius + 12, 4, 0, 2 * Math.PI);
            ctx.fillStyle = '#ff4444';
            ctx.fill();

            ctx.restore();
        }

        // スロットルゲージを描画する関数
        function drawThrottleGauge(throttleValue) {
            const canvas = document.getElementById('throttleCanvas');
            if (!canvas) return;

            const ctx = canvas.getContext('2d');
            const width = canvas.width;
            const height = canvas.height;

            // キャンバスをクリア
            ctx.clearRect(0, 0, width, height);

            // F/Rラベル領域を確保してゲージを描画
            const centerX = width / 2;
            var labelH = 10; // ラベル用の上下余白
            var gaugeTop = labelH + 2;
            var gaugeBottom = height - labelH - 2;
            var gaugeH = gaugeBottom - gaugeTop;
            const centerY = gaugeTop + gaugeH / 2;
            var barW = Math.min(Math.round(width * 0.6), 20);
            var barX = centerX - barW / 2;

            // Fラベル（上）
            ctx.fillStyle = '#333';
            ctx.font = '9px Arial';
            ctx.textAlign = 'center';
            ctx.textBaseline = 'top';
            ctx.fillText('F', centerX, 1);

            // ゲージ背景
            ctx.fillStyle = '#f0f0f0';
            ctx.fillRect(barX, gaugeTop, barW, gaugeH);
            ctx.strokeStyle = '#333';
            ctx.lineWidth = 1;
            ctx.strokeRect(barX, gaugeTop, barW, gaugeH);

            // 中央線（0の位置）
            ctx.strokeStyle = '#666';
            ctx.lineWidth = 1;
            ctx.beginPath();
            ctx.moveTo(barX - 2, centerY);
            ctx.lineTo(barX + barW + 2, centerY);
            ctx.stroke();

            // スケール線
            for (let i = 1; i <= 4; i++) {
                const y1 = centerY - (i * gaugeH / 8);
                const y2 = centerY + (i * gaugeH / 8);
                ctx.beginPath();
                ctx.moveTo(barX - 1, y1);
                ctx.lineTo(barX + 2, y1);
                ctx.moveTo(barX - 1, y2);
                ctx.lineTo(barX + 2, y2);
                ctx.stroke();
            }

            // ゲージバー
            let barHeight = Math.abs(throttleValue) * gaugeH / 2;
            let barY, barColor;

            if (throttleValue > 0) {
                barY = centerY - barHeight;
                barColor = '#44ff44';
            } else if (throttleValue < 0) {
                barY = centerY;
                barColor = '#ff4444';
            } else {
                barHeight = 0;
                barY = centerY;
                barColor = '#666';
            }

            if (barHeight > 0) {
                ctx.fillStyle = barColor;
                ctx.fillRect(barX + 1, barY, barW - 2, barHeight);
            }

            // Rラベル（下）
            ctx.fillStyle = '#333';
            ctx.font = '9px Arial';
            ctx.textAlign = 'center';
            ctx.textBaseline = 'bottom';
            ctx.fillText('R', centerX, height - 1);
        }


        // IMUヨーレートゲージを描画する関数
        // accel: {x: 前後m/s², y: 左右m/s²} x正=前進加速, y正=右加速
        function drawImuGauge(yawRate, accel) {
            const canvas = document.getElementById('imuCanvas');
            if (!canvas) return;
            const ctx = canvas.getContext('2d');
            const w = canvas.width, h = canvas.height;
            const cx = w / 2, cy = h / 2;
            const outerR = Math.min(w, h) / 2 - 1;  // 円弧用外径
            const arcW = 4;                           // 円弧の太さ
            const radius = outerR - arcW - 1;         // 内側の円（加速度表示用）

            ctx.clearRect(0, 0, w, h);

            if (yawRate === null || yawRate === undefined) {
                ctx.beginPath();
                ctx.arc(cx, cy, outerR, 0, 2 * Math.PI);
                ctx.strokeStyle = '#999';
                ctx.lineWidth = 1;
                ctx.stroke();
                ctx.fillStyle = '#ccc';
                ctx.font = '10px Arial';
                ctx.textAlign = 'center';
                ctx.textBaseline = 'middle';
                ctx.fillText('NA', cx, cy);
                return;
            }

            // --- ヨーレート円弧 ---
            // 背景トラック（グレー）
            ctx.beginPath();
            ctx.arc(cx, cy, outerR - arcW / 2, 0, 2 * Math.PI);
            ctx.strokeStyle = '#ddd';
            ctx.lineWidth = arcW;
            ctx.stroke();

            // 値の円弧: 12時（上）を0として、右=正、左=負
            var maxRate = 180;
            var clampedRate = Math.max(-maxRate, Math.min(maxRate, yawRate));
            var sweepAngle = (clampedRate / maxRate) * Math.PI; // ±πにマッピング
            var startAngle = -Math.PI / 2; // 12時位置

            var absRate = Math.abs(clampedRate);
            var arcColor = absRate < 30 ? '#33ff66' : absRate < 90 ? '#ffcc00' : '#ff3333';

            ctx.beginPath();
            if (sweepAngle >= 0) {
                ctx.arc(cx, cy, outerR - arcW / 2, startAngle, startAngle + sweepAngle);
            } else {
                ctx.arc(cx, cy, outerR - arcW / 2, startAngle + sweepAngle, startAngle);
            }
            ctx.strokeStyle = arcColor;
            ctx.lineWidth = arcW;
            ctx.lineCap = 'round';
            ctx.stroke();

            // 12時位置マーカー（小さいティック）
            ctx.beginPath();
            ctx.moveTo(cx, cy - outerR + 1);
            ctx.lineTo(cx, cy - outerR + arcW + 2);
            ctx.strokeStyle = '#666';
            ctx.lineWidth = 1;
            ctx.stroke();

            // --- 内側の円（加速度表示） ---
            ctx.beginPath();
            ctx.arc(cx, cy, radius, 0, 2 * Math.PI);
            ctx.strokeStyle = '#bbb';
            ctx.lineWidth = 1;
            ctx.stroke();

            ctx.beginPath();
            ctx.arc(cx, cy, radius - 0.5, 0, 2 * Math.PI);
            ctx.fillStyle = '#f5f5f5';
            ctx.fill();

            // --- 加速度三角形（上下左右）+ 数値 ---
            if (accel) {
                var maxAccel = 10.0;
                var triSize = 4;
                var triOffset = radius - 2;

                function accelColor(val) {
                    var absVal = Math.abs(val);
                    if (absVal < 1.0) return '#66cc66';
                    if (absVal < 5.0) return '#ffcc00';
                    return '#ff4444';
                }
                function accelAlpha(val) {
                    return Math.min(1.0, 0.3 + (Math.abs(val) / maxAccel) * 0.7);
                }

                var ax = accel.x || 0;
                var ay = accel.y || 0;
                var fs = Math.max(6, Math.round(radius * 0.38));
                var lf = fs + 'px monospace';
                var textOff = triSize * 1.5 + 1; // 三角底辺から内側へ

                ctx.save();
                ctx.translate(cx, cy);

                // 上（前進加速）
                ctx.globalAlpha = ax > 0 ? accelAlpha(ax) : 0.15;
                ctx.fillStyle = ax > 0 ? accelColor(ax) : '#999';
                ctx.beginPath();
                ctx.moveTo(0, -triOffset);
                ctx.lineTo(-triSize, -triOffset + triSize * 1.5);
                ctx.lineTo(triSize, -triOffset + triSize * 1.5);
                ctx.closePath(); ctx.fill();
                ctx.globalAlpha = 1.0; ctx.fillStyle = '#333'; ctx.font = lf;
                ctx.textAlign = 'center'; ctx.textBaseline = 'top';
                ctx.fillText(Math.abs(ax).toFixed(1), 0, -triOffset + textOff);

                // 下（減速）
                ctx.globalAlpha = ax < 0 ? accelAlpha(ax) : 0.15;
                ctx.fillStyle = ax < 0 ? accelColor(ax) : '#999';
                ctx.beginPath();
                ctx.moveTo(0, triOffset);
                ctx.lineTo(-triSize, triOffset - triSize * 1.5);
                ctx.lineTo(triSize, triOffset - triSize * 1.5);
                ctx.closePath(); ctx.fill();
                ctx.globalAlpha = 1.0; ctx.fillStyle = '#333'; ctx.font = lf;
                ctx.textAlign = 'center'; ctx.textBaseline = 'bottom';
                ctx.fillText(Math.abs(ax).toFixed(1), 0, triOffset - textOff);

                // 右
                ctx.globalAlpha = ay > 0 ? accelAlpha(ay) : 0.15;
                ctx.fillStyle = ay > 0 ? accelColor(ay) : '#999';
                ctx.beginPath();
                ctx.moveTo(triOffset, 0);
                ctx.lineTo(triOffset - triSize * 1.5, -triSize);
                ctx.lineTo(triOffset - triSize * 1.5, triSize);
                ctx.closePath(); ctx.fill();
                ctx.globalAlpha = 1.0; ctx.fillStyle = '#333'; ctx.font = lf;
                ctx.textAlign = 'right'; ctx.textBaseline = 'middle';
                ctx.fillText(Math.abs(ay).toFixed(1), triOffset - textOff, 0);

                // 左
                ctx.globalAlpha = ay < 0 ? accelAlpha(ay) : 0.15;
                ctx.fillStyle = ay < 0 ? accelColor(ay) : '#999';
                ctx.beginPath();
                ctx.moveTo(-triOffset, 0);
                ctx.lineTo(-triOffset + triSize * 1.5, -triSize);
                ctx.lineTo(-triOffset + triSize * 1.5, triSize);
                ctx.closePath(); ctx.fill();
                ctx.globalAlpha = 1.0; ctx.fillStyle = '#333'; ctx.font = lf;
                ctx.textAlign = 'left'; ctx.textBaseline = 'middle';
                ctx.fillText(Math.abs(ay).toFixed(1), -triOffset + textOff, 0);

                ctx.restore();
            }
        }

        // リサイズ機能の実装
        let isResizingHorizontal = false;
        let isResizingVertical = false;

        function initResizers() {
            const horizontalResizer = document.getElementById('horizontalResizer');
            const verticalResizer = document.getElementById('verticalResizer');
            const cameraContainer = document.querySelector('.camera-container');
            const controlContainer = document.querySelector('.control-container');
            const mainWrapper = document.querySelector('.main-wrapper');
            const container = document.querySelector('.container');
            const realtimeData = document.querySelector('.realtime-data');

            // 水平リサイザー (カメラとコントロールパネルの間)
            horizontalResizer.addEventListener('mousedown', (e) => {
                isResizingHorizontal = true;
                document.body.style.cursor = 'col-resize';
                e.preventDefault();
            });

            // 垂直リサイザー (メインコンテナとリアルタイムデータの間)
            verticalResizer.addEventListener('mousedown', (e) => {
                isResizingVertical = true;
                document.body.style.cursor = 'row-resize';
                e.preventDefault();
            });

            document.addEventListener('mousemove', (e) => {
                if (isResizingHorizontal) {
                    const containerRect = container.getBoundingClientRect();
                    const newControlWidth = containerRect.right - e.clientX - 8; // 8px for resizer

                    if (newControlWidth >= 200 && newControlWidth <= 800) {
                        controlContainer.style.width = newControlWidth + 'px';
                        resizeSensorCanvas();
                    }
                } else if (isResizingVertical) {
                    const wrapperRect = mainWrapper.getBoundingClientRect();
                    const newRealtimeHeight = wrapperRect.bottom - e.clientY - 8; // 8px for resizer

                    if (newRealtimeHeight >= 80) {
                        realtimeData.style.flex = '0 0 ' + newRealtimeHeight + 'px';
                        resizeSensorCanvas();
                    }
                }
            });

            document.addEventListener('mouseup', () => {
                if (isResizingHorizontal || isResizingVertical) {
                    resizeSensorCanvas();
                    if (sensorCtx) {
                        sensorCtx = sensorCanvas.getContext('2d');
                        drawSensorVisualization();
                    }
                }
                isResizingHorizontal = false;
                isResizingVertical = false;
                document.body.style.cursor = 'default';
            });
        }

        // Initialize the page
        document.addEventListener('DOMContentLoaded', () => {
            // WebSocket接続を開始
            connectWebSocket();

            // センサーキャンバスの初期化
            initializeSensorCanvas();

            // ステアリングハンドルとスロットルゲージの初期化
            drawSteeringWheel(0);
            drawThrottleGauge(0);
            drawImuGauge(null, null);

            // リサイザーの初期化
            initResizers();

            // WebSocket接続後に設定を取得
            setTimeout(() => {
                fetchConfigAndSetDefaults();
            }, 1000);
        });
    </script>
</body>
</html>'''
        
        with open(index_html_path, 'w', encoding='utf-8') as f:
            f.write(html_content)
        print(f"Created {index_html_path} with sensor visualization")


#------------------------------------------------------------------------------#
# Tornado WebSocket ハンドラー
#------------------------------------------------------------------------------#
class WebSocketHandler(tornado.websocket.WebSocketHandler):
    def check_origin(self, origin):
        return True
    
    def open(self):
        websocket_clients.add(self)
        print(f"[DEBUG] WebSocket client connected. Total clients: {len(websocket_clients)}")
    
    def on_close(self):
        websocket_clients.discard(self)
        print(f"WebSocket client disconnected. Total clients: {len(websocket_clients)}")
    
    async def on_message(self, message):
        try:
            data = json.loads(message)
            if data.get('type') == 'control':
                response = await self.handle_control(data)
                await self.write_message(json.dumps(response))
            elif data.get('type') == 'get_config':
                response = await self.handle_get_config()
                await self.write_message(json.dumps(response))
            elif data.get('type') == 'set_config':
                response = await self.handle_set_config(data)
                await self.write_message(json.dumps(response))
        except Exception as e:
            await self.write_message(json.dumps({
                'type': 'error',
                'message': str(e)
            }))
    
    async def handle_control(self, data):
        action = data.get('action')
        if action == 'pause':
            realtime_data['pause_drive'] = True
            return {'type': 'control_response', 'status': 'paused'}
        elif action == 'resume':
            realtime_data['pause_drive'] = False
            return {'type': 'control_response', 'status': 'resumed'}
        else:
            return {'type': 'control_response', 'status': 'unknown command'}
    
    async def handle_get_config(self):
        try:
            config_values = {
                key: getattr(config, key, None)
                for key in ALLOWED_CONFIG_KEYS
            }
            config_values['PLAN_LIST'] = getattr(config, 'PLAN_LIST', [])
            return {'type': 'config_response', 'status': 'ok', 'config': config_values}
        except Exception as e:
            return {'type': 'config_response', 'status': 'error', 'message': str(e)}
    
    async def handle_set_config(self, data):
        global set_config_reload
        config_data = data.get('config', {})
        updated_keys = []
        errors = []
        
        for key, value in config_data.items():
            if key not in ALLOWED_CONFIG_KEYS:
                errors.append(f"{key} is not an allowed config key.")
                continue
            
            # 型チェック
            if key == "PLAN":
                if value not in config.PLAN_LIST:
                    errors.append(f"Invalid PLAN: '{value}'. Must be in {config.PLAN_LIST}.")
                    continue
            
            if key == "HAND_SIDE":
                if value not in ["right", "left"]:
                    errors.append(f"Invalid HAND_SIDE: '{value}'. Must be 'right' or 'left'.")
                    continue

            # boolean変換
            if key == "WALL_FOLLOW_USE_ALIGNMENT":
                value = str(value).lower() in ("true", "1", "yes", "on")

            # 数値系の変換
            if key in ["FORWARD_STRAIGHT", "FORWARD_CORNER", "STOP_RANGE", "BACKWARD_RANGE",
                       "DETECTION_RANGE", "RIGHT_LEFT_RANGE", "TARGET_RANGE", "TARGET_RANGE_ADJUSTMENT",
                       "K_P", "K_I", "K_D", "RECOVERY_TIME", "WALL_FOLLOW_K_ANGLE"]:
                try:
                    value = float(value)
                except ValueError:
                    errors.append(f"{key} must be float, got {value}")
                    continue

            if key in ["RIGHT_LEFT_RECORD_NUMBER", "RECOVERY_BRAKING"]:
                try:
                    value = int(value)
                except ValueError:
                    errors.append(f"{key} must be int, got {value}")
                    continue

            setattr(config, key, value)
            updated_keys.append(key)

        set_config_reload = True

        return {
            'type': 'config_response',
            'status': 'partial_success' if errors else 'ok',
            'updated_keys': updated_keys,
            'errors': errors
        }

#------------------------------------------------------------------------------#
# 非同期画像処理
#------------------------------------------------------------------------------#
def encode_image_to_base64(frame):
    """画像をbase64にエンコードする関数（超高速版）"""
    try:
        # RGB→BGR変換（cv2.imencodeはBGR形式を期待）
        frame = frame[:, :, ::-1]

        if FAST_MODE:
            # 超高速モード: 品質50、より小さなサイズに
            height, width = frame.shape[:2]
            if width > 320:  # 大きすぎる場合はリサイズ
                scale = 320 / width
                new_width = int(width * scale)
                new_height = int(height * scale)
                frame = cv2.resize(frame, (new_width, new_height), interpolation=cv2.INTER_LINEAR)

            # 品質を大幅に下げて高速化
            encode_params = [cv2.IMWRITE_JPEG_QUALITY, 50, cv2.IMWRITE_JPEG_OPTIMIZE, 1]
            ret, buffer = cv2.imencode('.jpg', frame, encode_params)
        else:
            # 通常モード
            ret, buffer = cv2.imencode('.jpg', frame, [cv2.IMWRITE_JPEG_QUALITY, 70])

        if ret:
            return base64.b64encode(buffer).decode('utf-8')
    except Exception as e:
        print(f"Image encoding error: {e}")
    return None

def get_combined_frame():
    """カメラ画像を結合して返す（camera_0〜3対応）"""
    frames = []
    for i in range(4):
        img = realtime_data.get(f"camera_image_{i}")
        if img is not None:
            frames.append(img)

    if not frames:
        return None
    if len(frames) == 1:
        return frames[0]

    vertical = hasattr(config, 'IMAGE_CONCAT_DIRECTION') and config.IMAGE_CONCAT_DIRECTION == "vertical"
    # 高さ（横結合時）または幅（縦結合時）を揃える
    if vertical:
        target_w = frames[0].shape[1]
        resized = []
        for f in frames:
            if f.shape[1] != target_w:
                h = int(f.shape[0] * target_w / f.shape[1])
                f = cv2.resize(f, (target_w, h))
            resized.append(f)
        return np.vstack(resized)
    else:
        target_h = frames[0].shape[0]
        resized = []
        for f in frames:
            if f.shape[0] != target_h:
                w = int(f.shape[1] * target_h / f.shape[0])
                f = cv2.resize(f, (w, target_h))
            resized.append(f)
        return np.hstack(resized)

async def broadcast_image():
    """画像をWebSocket経由でブロードキャストする（超高速版）"""
    global websocket_clients, last_frame_hash, last_encoded_image, frame_skip_counter
    
    if not websocket_clients:
        return
    
    try:
        # フレームスキップによる負荷軽減
        if FAST_MODE:
            frame_skip_counter += 1
            if frame_skip_counter <= SKIP_FRAME_COUNT:
                return  # このフレームはスキップ
            frame_skip_counter = 0  # カウンターリセット
        
        frame = get_combined_frame()
        if frame is None:
            return
        
        # より高速なフレーム変更検出（サイズベース + 簡易チェック）
        if FAST_MODE:
            # 高速モード: 簡易な変更検出
            frame_signature = frame.shape + (frame[::50, ::50].mean(),)  # サンプリングベース
        else:
            # 通常モード: ハッシュベース
            frame_signature = hash(frame.tobytes())
        
        if frame_signature == last_frame_hash and last_encoded_image is not None:
            # フレームが変更されていない場合はキャッシュを使用
            base64_image = last_encoded_image
        else:
            # 新しいフレームをエンコード
            loop = tornado.ioloop.IOLoop.current()
            base64_image = await loop.run_in_executor(executor, encode_image_to_base64, frame)
            if base64_image:
                last_frame_hash = frame_signature
                last_encoded_image = base64_image
        
        if base64_image:
            # より軽量なメッセージ形式
            if FAST_MODE:
                message = {
                    'type': 'image',
                    'data': base64_image
                    # timestampを省略して軽量化
                }
            else:
                message = {
                    'type': 'image',
                    'data': base64_image,
                    'timestamp': datetime.now(_jst).isoformat()
                }
            
            # バイナリ送信準備（JSONエンコードを最適化）
            message_json = json.dumps(message, separators=(',', ':'))  # 空白を削除
            
            # 並列でWebSocketクライアントに送信
            tasks = []
            for client in list(websocket_clients):  # コピーを作成
                tasks.append(_send_raw_message_to_client(client, message_json))
            
            # 並列実行
            if tasks:
                results = await asyncio.gather(*tasks, return_exceptions=True)
                
                # 失敗したクライアントを削除
                for i, result in enumerate(results):
                    if isinstance(result, Exception):
                        client = list(websocket_clients)[i] if i < len(websocket_clients) else None
                        if client:
                            websocket_clients.discard(client)
            
    except Exception as e:
        print(f"[ERROR] broadcast_image error: {e}")

async def _send_message_to_client(client, message):
    """個別クライアントへのメッセージ送信"""
    try:
        await client.write_message(json.dumps(message))
    except Exception:
        raise  # エラーを上位に伝播

async def _send_raw_message_to_client(client, message_json):
    """個別クライアントへの生JSON送信（高速版）"""
    try:
        await client.write_message(message_json)
    except Exception:
        raise  # エラーを上位に伝播

async def broadcast_sensor_data():
    """センサーデータをWebSocket経由でブロードキャストする"""
    global websocket_clients
    if websocket_clients:
        data_to_send = {}
        for k, v in realtime_data.items():
            if k.startswith("camera_image_"):
                continue
            # numpy配列はJSON化できないのでリストに変換
            if isinstance(v, np.ndarray):
                data_to_send[k] = v.tolist()
            else:
                data_to_send[k] = v

        message_json = json.dumps({'type': 'sensor_data', 'data': data_to_send}, separators=(',', ':'))

        disconnected_clients = set()
        for client in websocket_clients:
            try:
                await client.write_message(message_json)
            except Exception:
                disconnected_clients.add(client)

        websocket_clients -= disconnected_clients

#------------------------------------------------------------------------------#
# Tornado HTTP ハンドラー
#------------------------------------------------------------------------------#
class MainHandler(tornado.web.RequestHandler):
    def get(self):
        self.render("monitor.html")

class GetDataHandler(tornado.web.RequestHandler):
    def get(self):
        data_to_return = {}
        for k, v in realtime_data.items():
            if k.startswith("camera_image_"):
                continue
            if isinstance(v, np.ndarray):
                data_to_return[k] = v.tolist()
            else:
                data_to_return[k] = v
        self.write(json.dumps(data_to_return))
        self.set_header("Content-Type", "application/json")

class ControlHandler(tornado.web.RequestHandler):
    def post(self):
        try:
            data = json.loads(self.request.body)
            action = data.get("action")
            
            if action == "pause":
                realtime_data["pause_drive"] = True
                self.write(json.dumps({"status": "paused"}))
            elif action == "resume":
                realtime_data["pause_drive"] = False
                self.write(json.dumps({"status": "resumed"}))
            else:
                self.set_status(400)
                self.write(json.dumps({"status": "unknown command"}))
        except Exception as e:
            self.set_status(400)
            self.write(json.dumps({"status": "error", "message": str(e)}))
        
        self.set_header("Content-Type", "application/json")

#------------------------------------------------------------------------------#
# 5) config.py の定数を変更
#------------------------------------------------------------------------------#
ALLOWED_CONFIG_KEYS = {
    # 変更を許可するキー名のセット
    "FORWARD_STRAIGHT",
    "FORWARD_CORNER",
    "STOP",
    "REVERSE",
    "LEFT",
    "NEUTRAL",
    "RIGHT",
    "STOP_RANGE",
    "BACKWARD_RANGE",
    "DETECTION_RANGE",
    "RIGHT_LEFT_RANGE",
    "TARGET_RANGE",
    "TARGET_RANGE_ADJUSTMENT",
    "K_P",
    "K_I",
    "K_D",
    "PLAN",
    "HAND_SIDE",
    "RIGHT_LEFT_RECORD_NUMBER",
    "RECOVERY_MODE",
    "RECOVERY_STEERING",
    "RECOVERY_TIME",
    "RECOVERY_BRAKING",
    "USE_PLOTTER",
    "MODEL_NAME",
    "WALL_FOLLOW_USE_ALIGNMENT",
    "WALL_FOLLOW_K_ANGLE",
}

class GetConfigHandler(tornado.web.RequestHandler):
    def get(self):
        try:
            config_values = {
                key: getattr(config, key, None)
                for key in ALLOWED_CONFIG_KEYS
            }
            config_values['PLAN_LIST'] = getattr(config, 'PLAN_LIST', [])
            self.write(json.dumps({"status": "ok", "config": config_values}))
        except Exception as e:
            self.set_status(500)
            self.write(json.dumps({"status": "error", "message": str(e)}))
        
        self.set_header("Content-Type", "application/json")


class SetConfigHandler(tornado.web.RequestHandler):
    def post(self):
        global set_config_reload
        
        try:
            data = json.loads(self.request.body)
            if not data:
                self.set_status(400)
                self.write(json.dumps({"status": "error", "message": "No JSON data"}))
                return
            
            updated_keys = []
            errors = []
            
            for key, value in data.items():
                if key not in ALLOWED_CONFIG_KEYS:
                    errors.append(f"{key} is not an allowed config key.")
                    continue
                
                # 型チェック
                if key == "PLAN":
                    if value not in config.PLAN_LIST:
                        errors.append(f"Invalid PLAN: '{value}'. Must be in {config.PLAN_LIST}.")
                        continue
                
                if key == "HAND_SIDE":
                    if value not in ["right", "left"]:
                        errors.append(f"Invalid HAND_SIDE: '{value}'. Must be 'right' or 'left'.")
                        continue

                # boolean変換
                if key == "WALL_FOLLOW_USE_ALIGNMENT":
                    value = str(value).lower() in ("true", "1", "yes", "on")

                # 数値系
                if key in ["FORWARD_STRAIGHT", "FORWARD_CORNER", "STOP_RANGE", "BACKWARD_RANGE",
                           "DETECTION_RANGE", "RIGHT_LEFT_RANGE", "TARGET_RANGE", "TARGET_RANGE_ADJUSTMENT",
                           "K_P", "K_I", "K_D", "RECOVERY_TIME", "WALL_FOLLOW_K_ANGLE"]:
                    try:
                        value = float(value)
                    except ValueError:
                        errors.append(f"{key} must be float, got {value}")
                        continue

                if key in ["RIGHT_LEFT_RECORD_NUMBER", "RECOVERY_BRAKING"]:
                    try:
                        value = int(value)
                    except ValueError:
                        errors.append(f"{key} must be int, got {value}")
                        continue

                setattr(config, key, value)
                updated_keys.append(key)
            
            set_config_reload = True
            
            if errors:
                self.set_status(400)
                response = {
                    "status": "partial_success",
                    "updated_keys": updated_keys,
                    "errors": errors
                }
            else:
                response = {
                    "status": "ok",
                    "updated_keys": updated_keys
                }
            
            self.write(json.dumps(response))
            
        except Exception as e:
            self.set_status(400)
            self.write(json.dumps({"status": "error", "message": str(e)}))
        
        self.set_header("Content-Type", "application/json")

#------------------------------------------------------------------------------#
# モデル管理
#------------------------------------------------------------------------------#
class GetModelsHandler(tornado.web.RequestHandler):
    def get(self):
        # config.MODEL_DIRを使用、なければプロジェクトルートのmodelsフォルダ
        base_dir = os.path.dirname(os.path.abspath(__file__))
        models_folder = os.path.join(base_dir, getattr(config, 'MODEL_DIR', 'models'))
        if not os.path.exists(models_folder):
            self.write(json.dumps({"models": []}))
            return
        
        models = [
            f for f in os.listdir(models_folder)
            if os.path.isfile(os.path.join(models_folder, f)) and not f.endswith('.png')
        ]
        
        self.write(json.dumps({"models": models}))
        self.set_header("Content-Type", "application/json")


#------------------------------------------------------------------------------#
# run.py からデータを更新するための関数
#------------------------------------------------------------------------------#
def update_data(mode=None,
                steering_value=None,
                throttle_value=None,
                ranges=None,
                imu_data=None,
                timestamp=None,
                camera_image_0=None,
                camera_image_1=None,
                camera_image_2=None,
                camera_image_3=None,
                lidar_measurements=None,
                ftg_info=None,
                imu_yaw_rate=None,
                imu_accel=None,
                rpm=None,
                optical_flow_speed=None,
                record_count=None,
                fps=None):
    """
    run.pyのメインループなどから呼び出されてリアルタイムデータを更新する。
    ここで受け取った値をrealtime_dataに格納し、/get_data で参照できるようにする。
    """
    if mode is not None:
        realtime_data["mode"] = mode
    if steering_value is not None:
        realtime_data["steering_value"] = steering_value
    if throttle_value is not None:
        realtime_data["throttle_value"] = throttle_value
    # rangesが渡された場合は優先、なければrangesを使用（後方互換性）
    sensor_ranges = ranges if ranges is not None else ranges
    if sensor_ranges is not None:
        realtime_data["ranges"] = sensor_ranges
    if imu_data is not None:
        realtime_data["imu_data"] = imu_data
    if timestamp is not None:
        realtime_data["timestamp"] = convert_timestamp(timestamp)
    if camera_image_0 is not None:
        realtime_data["camera_image_0"] = camera_image_0
    if camera_image_1 is not None:
        realtime_data["camera_image_1"] = camera_image_1
    if camera_image_2 is not None:
        realtime_data["camera_image_2"] = camera_image_2
    if camera_image_3 is not None:
        realtime_data["camera_image_3"] = camera_image_3
    # LiDAR点群データ（numpy配列のまま保持、JSON変換はbroadcast時に実施）
    realtime_data["lidar_measurements"] = lidar_measurements
    # FTG診断情報
    realtime_data["ftg_info"] = ftg_info
    # 追加センサー
    realtime_data["imu_yaw_rate"] = imu_yaw_rate
    realtime_data["imu_accel"] = imu_accel
    realtime_data["rpm"] = rpm
    realtime_data["optical_flow_speed"] = optical_flow_speed
    # ループ情報
    if record_count is not None:
        realtime_data["record_count"] = record_count
    if fps is not None:
        realtime_data["fps"] = fps
    # LiDAR設定（PLANが変わった時のみ更新）
    current_plan = getattr(config, 'PLAN', '')
    cached = realtime_data.get("lidar_config")
    if cached is None or cached.get('plan') != current_plan:
        realtime_data["lidar_config"] = {
            'angle_start': getattr(config, 'LIDAR_ANGLE_START', -135),
            'angle_end': getattr(config, 'LIDAR_ANGLE_END', 135),
            'angle_offset': getattr(config, 'LIDAR_ANGLE_OFFSET', 0),
            'clockwise': getattr(config, 'LIDAR_CLOCKWISE', False),
            'max_distance': getattr(config, 'LIDAR_MAX_DISTANCE', 4000),
            'plan': current_plan,
            'zone_angles': _compute_zone_angles(),
        }

def _compute_zone_angles():
    """ZONE_INDEXからゾーンの中心角度とアーク幅を計算し、canvas arc座標で返す"""
    zone_index = getattr(config, 'ZONE_INDEX', [])
    zone_names = getattr(config, 'ZONE_NAMES', [])
    data_points = getattr(config, 'LIDAR_DATA_POINTS', 0)
    angle_start = getattr(config, 'LIDAR_ANGLE_START', 0)
    angle_end = getattr(config, 'LIDAR_ANGLE_END', 0)
    angle_offset = getattr(config, 'LIDAR_ANGLE_OFFSET', 0)
    clockwise = getattr(config, 'LIDAR_CLOCKWISE', False)
    total_range = angle_end - angle_start
    if data_points <= 0 or total_range == 0:
        return {}
    result = {}
    for i, indices in enumerate(zone_index):
        if not indices or i >= len(zone_names):
            continue
        name = zone_names[i]
        n = len(indices)
        # アーク幅（点数に基づく）
        arc_deg = n / data_points * total_range
        # 中心インデックス（円環平均でラップアラウンド対応）
        idx_angles = [2 * math.pi * idx / data_points for idx in indices]
        sin_sum = sum(math.sin(a) for a in idx_angles)
        cos_sum = sum(math.cos(a) for a in idx_angles)
        center_idx = math.atan2(sin_sum, cos_sum) * data_points / (2 * math.pi)
        if center_idx < 0:
            center_idx += data_points
        # インデックスを物理角度に変換（standard math: 0=右, CCW正）
        if clockwise:
            raw = angle_end - total_range * center_idx / (data_points - 1)
        else:
            raw = angle_start + total_range * center_idx / (data_points - 1)
        center_deg = raw + angle_offset
        # -180..180に正規化
        center_deg = ((center_deg + 180) % 360 + 360) % 360 - 180
        # canvas arc座標（standard mathを反転）: 0=右, CW正, -PI/2=上=前方
        result[name] = {
            'center_rad': round(-center_deg * math.pi / 180, 4),
            'arc_rad': round(arc_deg * math.pi / 180, 4),
        }
    return result

#------------------------------------------------------------------------------#
# 表示のための補助関数
#------------------------------------------------------------------------------#
def convert_timestamp(timestamp):
    # タイムスタンプ文字列の長さチェック
    if len(timestamp) != 20:
        raise ValueError("タイムスタンプの形式が正しくありません。20桁である必要があります。")

    # 年月日、時分秒、ミリ秒を抽出
    year = int(timestamp[0:4])
    month = int(timestamp[4:6])
    day = int(timestamp[6:8])
    hour = int(timestamp[8:10])
    minute = int(timestamp[10:12])
    second = int(timestamp[12:14])
    microsecond = int(timestamp[14:20])  # ミリ秒以下は6桁で解釈

    # datetimeオブジェクトを作成
    dt = datetime(year, month, day, hour, minute, second, microsecond)

    # 見やすいフォーマットに変換
    formatted_time = dt.strftime("%Y-%m-%d %H:%M:%S.%f")[:-3]  # ミリ秒まで表示
    return formatted_time


#------------------------------------------------------------------------------#
# Tornado アプリケーション起動
#------------------------------------------------------------------------------#
def make_app():
    # templatesフォルダとindex.htmlを作成（存在しない場合）
    create_template_if_needed()
    
    return tornado.web.Application([
        (r"/", MainHandler),
        (r"/ws", WebSocketHandler),
        (r"/get_data", GetDataHandler),
        (r"/control", ControlHandler),
        (r"/get_config", GetConfigHandler),
        (r"/set_config", SetConfigHandler),
        (r"/get_models", GetModelsHandler),
    ], template_path="templates", static_path="static")

async def periodic_broadcast():
    """定期的なデータブロードキャスト"""
    while not shutdown_signal:
        try:
            # 画像とセンサーデータを並行で送信
            await asyncio.gather(
                broadcast_image(),
                broadcast_sensor_data(),
                return_exceptions=True
            )
            if FAST_MODE:
                await asyncio.sleep(0.02)  # 高速モード
            else:
                await asyncio.sleep(0.05)  # 通常モード
        except Exception as e:
            print(f"Broadcast error: {e}")
            await asyncio.sleep(0.1)

def open_browser(url, delay=1.5):
    """ブラウザを開く関数（遅延実行）"""
    time.sleep(delay)
    try:
        # プラットフォームに応じて適切な方法でブラウザを開く
        if platform.system() == 'Linux':
            # Jetson (ARM Linux) の場合、環境変数DISPLAYをチェック
            if 'DISPLAY' in os.environ:
                webbrowser.open(url)
            else:
                print(f"No display found. Please open browser manually: {url}")
        else:
            webbrowser.open(url)
    except Exception as e:
        print(f"Could not open browser automatically: {e}")
        print(f"Please open browser manually: {url}")

def run(host="0.0.0.0", port=8000, debug=False, open_browser_on_start=True):
    """
    Tornadoアプリケーションを起動
    """
    app = make_app()
    app.listen(port, address=host)
    
    # 定期ブロードキャストタスクを開始
    tornado.ioloop.IOLoop.current().spawn_callback(periodic_broadcast)
    
    # print(f"Tornado server started on {host}:{port}")
    import socket
    import subprocess
    # 実際のネットワークIPを取得（hostname -Iは割当済みIPを返す）
    try:
        ip_addr = subprocess.check_output(['hostname', '-I'], text=True).strip().split()[0]
    except Exception:
        ip_addr = "localhost"
    print(f"Sensor visualization available at: http://{ip_addr}:{port}\n")

    # ブラウザを自動的に開く
    if open_browser_on_start:
        url = f"http://{ip_addr}:{port}"
        browser_thread = threading.Thread(target=open_browser, args=(url,), daemon=True)
        browser_thread.start()
    
    tornado.ioloop.IOLoop.current().start()

def init_cameras_standalone():
    """単独起動時のカメラ初期化（config.pyとcamera.pyを使用、camera_0〜3対応）"""
    global camera_instances
    import camera

    print(f"Initializing cameras with config: {config.IMAGE_W}x{config.IMAGE_H}@{config.CAMERA_FRAMERATE}fps")
    print(f"Active sensors: {config.ACTIVE_SENSORS}")

    # config.pyのACTIVE_SENSORS設定に基づいてカメラを初期化
    for cam_idx in range(4):
        cam_name = f"camera_{cam_idx}"
        if cam_name in config.ACTIVE_SENSORS:
            try:
                camera_instances[cam_name] = camera.create_camera(
                    device_id=getattr(config, f'CAMERA_{cam_idx}_DEVICE_ID', cam_idx),
                    camera_type=getattr(config, f'CAMERA_{cam_idx}_TYPE', None))
                print(f"Camera {cam_idx} initialized: {config.IMAGE_W}x{config.IMAGE_H}")
            except Exception as e:
                print(f"Failed to initialize camera {cam_idx}: {e}")

def init_sensors_standalone():
    """単独起動時のセンサー初期化（ultrasonic/lidarをconfig.pyに基づいて初期化）"""
    global active_sensor_instances
    active_sensor_instances = {}
    initialized = False

    try:
        # ultrasonicセンサー初期化
        if "ultrasonic" in config.ACTIVE_SENSORS and hasattr(config, 'ULTRASONIC_SENSOR_LIST'):
            try:
                import ultrasonic
                for sensor_name in config.ULTRASONIC_SENSOR_LIST:
                    active_sensor_instances[sensor_name] = ultrasonic.Ultrasonic(sensor_name=sensor_name)
                print(f"Ultrasonic sensors initialized: {config.ULTRASONIC_SENSOR_LIST}")
                initialized = True
            except Exception as e:
                print(f"Failed to initialize ultrasonic sensors: {e}")

        # LiDARセンサー初期化
        if "lidar" in config.ACTIVE_SENSORS:
            try:
                import lidar as lidar_module
                # デバイス検出
                from device_detection import detect_device
                device_info = detect_device()
                config.I2C_BUS = device_info.i2c_bus

                detected = lidar_module.detect_lidar(config)
                if detected is not None:
                    print(f"LiDAR detected: {detected}")
                if config.LIDAR_TYPE != "NONE":
                    print("--- LiDAR初期化開始 ---")
                    lidar_instance = lidar_module.create_lidar(lidar_type=config.LIDAR_TYPE)
                    active_sensor_instances["lidar"] = lidar_instance
                    print("--- LiDAR初期化完了 ---")
                    print("LiDARのスキャン開始を待機中...")
                    time.sleep(3)
                    print("--- LiDAR準備完了 ---")
                    initialized = True
                else:
                    print("LiDAR type is NONE — skipping")
            except Exception as e:
                print(f"Failed to initialize LiDAR: {e}")
                import traceback
                traceback.print_exc()

        # DataAggregatorの初期化（センサーがある場合）
        if active_sensor_instances:
            try:
                from data_aggregator import DataAggregator
                global data_aggregator_instance
                data_aggregator_instance = DataAggregator(sensor_instances=active_sensor_instances)
                print(f"DataAggregator initialized with: {list(active_sensor_instances.keys())}")
            except Exception as e:
                print(f"Failed to initialize DataAggregator: {e}")
                initialized = False
        else:
            print("No distance sensors configured in ACTIVE_SENSORS")

        return initialized
    except Exception as e:
        print(f"Error initializing sensors: {e}")
        return False

def update_cameras_standalone():
    """単独起動時のカメラ更新ループ（config.py設定に基づく）"""
    global camera_instances, shutdown_signal, data_aggregator_instance
    
    # config.pyのフレームレートに基づいたスリープ時間を計算
    frame_interval = 1.0 / config.CAMERA_FRAMERATE if hasattr(config, 'CAMERA_FRAMERATE') else 0.033
    
    while not shutdown_signal:
        try:
            # config.pyのACTIVE_SENSORSに基づいてカメラを更新（camera_0〜3対応）
            for cam_idx in range(4):
                cam_name = f"camera_{cam_idx}"
                if cam_name in config.ACTIVE_SENSORS and cam_name in camera_instances:
                    ret, frame = camera_instances[cam_name].read()
                    if ret and frame is not None:
                        realtime_data[f"camera_image_{cam_idx}"] = frame
            
            # センサー値を取得（run.pyと同様のフォーマット）
            ranges = {}
            lidar_data_dict = None
            if 'data_aggregator_instance' in globals() and data_aggregator_instance is not None:
                try:
                    # LiDAR画像生成フラグ
                    data_aggregator_instance.lidar_generate_image = True
                    # センサー値を更新
                    data_aggregator_instance.update_sensors()

                    # ultrasonicセンサー値を取得
                    if "ultrasonic" in config.ACTIVE_SENSORS and hasattr(config, 'ULTRASONIC_SENSOR_LIST'):
                        for us_name in config.ULTRASONIC_SENSOR_LIST:
                            value = data_aggregator_instance.get_latest_sensor_value(us_name)
                            if value is not None:
                                ranges[us_name] = value

                    # LiDARセンサー値を取得（run.pyと同じゾーン距離マッピング）
                    if "lidar" in config.ACTIVE_SENSORS:
                        lidar_data_latest = data_aggregator_instance.get_latest_sensor_value("lidar")
                        if lidar_data_latest and isinstance(lidar_data_latest, dict):
                            lidar_data_dict = lidar_data_latest
                            zone_distances = lidar_data_latest.get('zone_distances', [])
                            for i, zone_name in enumerate(config.ULTRASONIC_SENSOR_LIST):
                                if i < len(zone_distances):
                                    ranges[zone_name] = int(zone_distances[i])
                                else:
                                    ranges[zone_name] = 0
                except Exception as e:
                    print(f"Failed to update sensors: {e}")

            # データの更新
            realtime_data["mode"] = "standalone_test"
            realtime_data["steering_value"] = 0.0
            realtime_data["throttle_value"] = 0.0

            if ranges:
                realtime_data["ranges"] = ranges
            elif "ranges" not in realtime_data or not realtime_data["ranges"]:
                realtime_data["ranges"] = {name: 0 for name in config.ULTRASONIC_SENSOR_LIST}

            # LiDAR点群データ
            if lidar_data_dict:
                realtime_data["lidar_measurements"] = lidar_data_dict.get('measurements')
            
            realtime_data["timestamp"] = convert_timestamp(datetime.now(_jst).strftime("%Y%m%d%H%M%S%f"))
            
            time.sleep(frame_interval)  # config.pyのフレームレートに合わせる
            
        except Exception as e:
            print(f"[ERROR] Camera update error: {e}")
            import traceback
            traceback.print_exc()
            time.sleep(0.1)

def cleanup_cameras_standalone():
    """単独起動時のカメラ・センサークリーンアップ"""
    global camera_instances, active_sensor_instances

    for name, cam in camera_instances.items():
        if cam:
            try:
                cam.cleanup()
                print(f"{name} cleaned up")
            except Exception as e:
                print(f"Failed to cleanup {name}: {e}")
    camera_instances.clear()

    # センサーのクリーンアップ
    for name, sensor in active_sensor_instances.items():
        if hasattr(sensor, 'shutdown') and callable(sensor.shutdown):
            try:
                sensor.shutdown()
                print(f"{name} shutdown")
            except Exception as e:
                print(f"Failed to shutdown {name}: {e}")
        elif hasattr(sensor, 'cleanup') and callable(sensor.cleanup):
            try:
                sensor.cleanup()
                print(f"{name} cleaned up")
            except Exception as e:
                print(f"Failed to cleanup {name}: {e}")
    active_sensor_instances.clear()

    # リソースの完全な解放
    import gc
    time.sleep(0.2)
    gc.collect()

if __name__ == '__main__':
    print("Monitor starting in standalone mode...")
    print(f"Using config.py settings:")
    print(f"  Image size: {config.IMAGE_W}x{config.IMAGE_H}")
    print(f"  Framerate: {config.CAMERA_FRAMERATE}fps")
    print(f"  Active sensors: {config.ACTIVE_SENSORS}")
    print(f"  Image concat direction: {getattr(config, 'IMAGE_CONCAT_DIRECTION', 'horizontal')}")
    print(f"  Sensor visualization: Enabled")
    
    # config.pyの設定に基づいてカメラを初期化
    init_cameras_standalone()
    
    # センサーの初期化を試行
    sensors_initialized = init_sensors_standalone()
    if sensors_initialized:
        print(f"Sensors initialized: {list(active_sensor_instances.keys())}")
    else:
        print("No distance sensors available")
    
    # カメラ更新スレッドを開始
    camera_update_thread = threading.Thread(target=update_cameras_standalone, daemon=True)
    camera_update_thread.start()
    
    try:
        # Tornadoアプリを起動（ブラウザ自動起動あり）
        run(debug=False, open_browser_on_start=True)
    except KeyboardInterrupt:
        print("\nShutting down...")
    finally:
        # 終了処理
        shutdown_signal = True
        if camera_update_thread:
            camera_update_thread.join(timeout=2.0)
        cleanup_cameras_standalone()
        print("Cleanup complete")
