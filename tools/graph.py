#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
走行データ可視化Webアプリケーション
CSVまたはNDJSON形式の走行データをインタラクティブに表示
"""

import os
import sys
import json
import pandas as pd
import numpy as np
from datetime import datetime
from flask import Flask, render_template_string, jsonify, request, send_file
import plotly.graph_objs as go
import plotly.utils
from PIL import Image
import base64
from io import BytesIO
import config

app = Flask(__name__)

# グローバル変数
current_data = None
current_file = None

# HTMLテンプレート
HTML_TEMPLATE = '''
<!DOCTYPE html>
<html>
<head>
    <title>走行データビューア</title>
    <meta charset="utf-8">
    <script src="https://cdn.plot.ly/plotly-latest.min.js"></script>
    <style>
        body {
            font-family: Arial, sans-serif;
            margin: 0;
            padding: 20px;
            background-color: #f5f5f5;
        }
        .container {
            max-width: 1400px;
            margin: 0 auto;
            background-color: white;
            padding: 20px;
            border-radius: 10px;
            box-shadow: 0 0 10px rgba(0,0,0,0.1);
        }
        .header {
            text-align: center;
            margin-bottom: 30px;
        }
        .controls {
            margin-bottom: 20px;
            padding: 15px;
            background-color: #f0f0f0;
            border-radius: 5px;
        }
        .graph-container {
            margin-bottom: 20px;
        }
        .image-viewer {
            text-align: center;
            margin: 20px 0;
            padding: 20px;
            background-color: #f9f9f9;
            border-radius: 5px;
        }
        .image-viewer img {
            max-width: 100%;
            height: auto;
            border: 1px solid #ddd;
        }
        select, button {
            padding: 8px 15px;
            margin: 5px;
            border: 1px solid #ddd;
            border-radius: 4px;
            background-color: white;
            cursor: pointer;
        }
        button:hover {
            background-color: #e0e0e0;
        }
        .info-panel {
            display: grid;
            grid-template-columns: repeat(auto-fit, minmax(200px, 1fr));
            gap: 15px;
            margin: 20px 0;
        }
        .info-card {
            padding: 15px;
            background-color: #f8f8f8;
            border-radius: 5px;
            border-left: 4px solid #007bff;
        }
        .info-card h3 {
            margin-top: 0;
            color: #333;
        }
        #timeSlider {
            width: 100%;
            margin: 10px 0;
        }
    </style>
</head>
<body>
    <div class="container">
        <div class="header">
            <h1>🚗 走行データビューア</h1>
            <p>記録された走行データをインタラクティブに確認できます</p>
        </div>
        
        <div class="controls">
            <label>データファイル選択:</label>
            <select id="fileSelect" onchange="loadData()">
                <option value="">-- ファイルを選択 --</option>
                {% for file in files %}
                <option value="{{ file }}">{{ file }}</option>
                {% endfor %}
            </select>
            
            <button onclick="refreshFiles()">🔄 更新</button>
        </div>
        
        <div id="infoPanel" class="info-panel" style="display:none;">
            <div class="info-card">
                <h3>📊 データ数</h3>
                <p id="recordCount">-</p>
            </div>
            <div class="info-card">
                <h3>⏱️ 記録時間</h3>
                <p id="duration">-</p>
            </div>
            <div class="info-card">
                <h3>🎯 モード</h3>
                <p id="mode">-</p>
            </div>
            <div class="info-card">
                <h3>📷 画像数</h3>
                <p id="imageCount">-</p>
            </div>
        </div>
        
        <div class="graph-container">
            <h2>センサーデータ</h2>
            <div id="sensorGraph"></div>
        </div>
        
        <div class="graph-container">
            <h2>制御値（ステアリング・スロットル）</h2>
            <div id="controlGraph"></div>
        </div>
        
        <div class="image-viewer" id="imageViewer" style="display:none;">
            <h2>カメラ画像</h2>
            <input type="range" id="timeSlider" min="0" max="100" value="0" oninput="updateImage(this.value)">
            <p>タイムスタンプ: <span id="timestamp">-</span></p>
            <img id="cameraImage" src="" alt="カメラ画像">
            <p>ステアリング: <span id="steeringValue">-</span> | スロットル: <span id="throttleValue">-</span></p>
        </div>
    </div>
    
    <script>
        let currentData = null;
        let imageData = [];
        
        function refreshFiles() {
            location.reload();
        }
        
        async function loadData() {
            const fileSelect = document.getElementById('fileSelect');
            const filename = fileSelect.value;
            
            if (!filename) return;
            
            try {
                const response = await fetch(`/api/load_data?file=${filename}`);
                const data = await response.json();
                
                if (data.error) {
                    alert('エラー: ' + data.error);
                    return;
                }
                
                currentData = data;
                updateInfo(data);
                plotSensorData(data);
                plotControlData(data);
                
                if (data.has_images) {
                    document.getElementById('imageViewer').style.display = 'block';
                    imageData = data.image_data || [];
                    const slider = document.getElementById('timeSlider');
                    slider.max = imageData.length - 1;
                    slider.value = 0;
                    updateImage(0);
                } else {
                    document.getElementById('imageViewer').style.display = 'none';
                }
                
                document.getElementById('infoPanel').style.display = 'grid';
                
            } catch (error) {
                alert('データの読み込みに失敗しました: ' + error);
            }
        }
        
        function updateInfo(data) {
            document.getElementById('recordCount').textContent = data.record_count + ' 件';
            document.getElementById('duration').textContent = data.duration + ' 秒';
            document.getElementById('mode').textContent = data.mode || '不明';
            document.getElementById('imageCount').textContent = data.image_count + ' 枚';
        }
        
        function plotSensorData(data) {
            const traces = [];
            
            // 超音波センサーデータ
            if (data.ultrasonic_columns.length > 0) {
                data.ultrasonic_columns.forEach(col => {
                    traces.push({
                        x: data.timestamps,
                        y: data.sensor_data[col],
                        name: col,
                        type: 'scatter',
                        mode: 'lines'
                    });
                });
            }
            
            const layout = {
                title: '超音波センサー測定値',
                xaxis: { title: '時間 (秒)' },
                yaxis: { title: '距離 (mm)' },
                hovermode: 'x unified'
            };
            
            Plotly.newPlot('sensorGraph', traces, layout);
        }
        
        function plotControlData(data) {
            const traces = [
                {
                    x: data.timestamps,
                    y: data.steering,
                    name: 'ステアリング',
                    type: 'scatter',
                    mode: 'lines',
                    line: { color: 'blue' },
                    yaxis: 'y'
                },
                {
                    x: data.timestamps,
                    y: data.throttle,
                    name: 'スロットル',
                    type: 'scatter',
                    mode: 'lines',
                    line: { color: 'red' },
                    yaxis: 'y2'
                }
            ];
            
            const layout = {
                title: '制御値の時系列',
                xaxis: { title: '時間 (秒)' },
                yaxis: {
                    title: 'ステアリング',
                    side: 'left',
                    range: [-1.2, 1.2]
                },
                yaxis2: {
                    title: 'スロットル',
                    side: 'right',
                    overlaying: 'y',
                    range: [-1.2, 1.2]
                },
                hovermode: 'x unified'
            };
            
            Plotly.newPlot('controlGraph', traces, layout);
        }
        
        function updateImage(index) {
            if (!imageData || imageData.length === 0) return;
            
            index = parseInt(index);
            const imgInfo = imageData[index];
            
            if (imgInfo) {
                document.getElementById('timestamp').textContent = imgInfo.timestamp.toFixed(3) + ' 秒';
                document.getElementById('steeringValue').textContent = imgInfo.steering.toFixed(3);
                document.getElementById('throttleValue').textContent = imgInfo.throttle.toFixed(3);
                
                // 画像を読み込む
                fetch(`/api/get_image?path=${encodeURIComponent(imgInfo.path)}`)
                    .then(response => response.json())
                    .then(data => {
                        if (data.image) {
                            document.getElementById('cameraImage').src = 'data:image/jpeg;base64,' + data.image;
                        }
                    });
            }
        }
    </script>
</body>
</html>
'''

def find_data_files():
    """記録データファイルを検索"""
    files = []
    
    # CSV形式
    if os.path.exists(config.RECORDS_DIRECTORY):
        csv_files = [f for f in os.listdir(config.RECORDS_DIRECTORY) if f.endswith('.csv')]
        files.extend([os.path.join(config.RECORDS_DIRECTORY, f) for f in csv_files])
    
    # NDJSON形式
    if os.path.exists(config.RECORDS_DIRECTORY):
        ndjson_files = [f for f in os.listdir(config.RECORDS_DIRECTORY) if f.endswith('.ndjson')]
        files.extend([os.path.join(config.RECORDS_DIRECTORY, f) for f in ndjson_files])
    
    return sorted(files)

def load_csv_data(filepath):
    """CSVデータを読み込む"""
    df = pd.read_csv(filepath)
    return df

def load_ndjson_data(filepath):
    """NDJSONデータを読み込む"""
    records = []
    with open(filepath, 'r') as f:
        for line in f:
            records.append(json.loads(line))
    return pd.DataFrame(records)

def process_data(df):
    """データを処理して可視化用に整形"""
    result = {
        'record_count': len(df),
        'has_images': False,
        'image_count': 0,
        'timestamps': [],
        'steering': [],
        'throttle': [],
        'sensor_data': {},
        'ultrasonic_columns': [],
        'image_data': []
    }
    
    # タイムスタンプ処理
    if 'timestamp' in df.columns:
        timestamps = pd.to_numeric(df['timestamp'], errors='coerce')
        result['timestamps'] = (timestamps - timestamps.min()).tolist()
        result['duration'] = round((timestamps.max() - timestamps.min()), 2)
    else:
        result['timestamps'] = list(range(len(df)))
        result['duration'] = len(df)
    
    # 制御値
    if 'user/angle' in df.columns:
        result['steering'] = df['user/angle'].fillna(0).tolist()
    elif 'steering' in df.columns:
        result['steering'] = df['steering'].fillna(0).tolist()
    else:
        result['steering'] = [0] * len(df)
    
    if 'user/throttle' in df.columns:
        result['throttle'] = df['user/throttle'].fillna(0).tolist()
    elif 'throttle' in df.columns:
        result['throttle'] = df['throttle'].fillna(0).tolist()
    else:
        result['throttle'] = [0] * len(df)
    
    # 超音波センサーデータ
    ultrasonic_cols = [col for col in df.columns if col in config.ULTRASONIC_SENSOR_LIST]
    result['ultrasonic_columns'] = ultrasonic_cols
    for col in ultrasonic_cols:
        result['sensor_data'][col] = df[col].fillna(config.CUTOFF_RANGE).tolist()
    
    # モード
    if 'mode' in df.columns:
        result['mode'] = df['mode'].iloc[0] if len(df) > 0 else 'Unknown'
    else:
        result['mode'] = config.PLAN
    
    # 画像データ
    if 'image_file' in df.columns or 'cam/image_array' in df.columns:
        result['has_images'] = True
        
        for idx, row in df.iterrows():
            if idx % 10 == 0:  # 10フレームごとにサンプリング（パフォーマンスのため）
                img_info = {
                    'index': idx,
                    'timestamp': result['timestamps'][idx] if idx < len(result['timestamps']) else 0,
                    'steering': result['steering'][idx] if idx < len(result['steering']) else 0,
                    'throttle': result['throttle'][idx] if idx < len(result['throttle']) else 0
                }
                
                if 'image_file' in df.columns:
                    img_info['path'] = row['image_file']
                elif 'cam/image_array' in df.columns:
                    img_info['path'] = row['cam/image_array']
                
                if pd.notna(img_info['path']):
                    result['image_data'].append(img_info)
        
        result['image_count'] = len(result['image_data'])
    
    return result

@app.route('/')
def index():
    """メインページ"""
    files = find_data_files()
    return render_template_string(HTML_TEMPLATE, files=files)

@app.route('/api/load_data')
def load_data():
    """データファイルを読み込む"""
    filepath = request.args.get('file', '')
    
    if not filepath or not os.path.exists(filepath):
        return jsonify({'error': 'ファイルが見つかりません'})
    
    try:
        # ファイル形式に応じて読み込み
        if filepath.endswith('.csv'):
            df = load_csv_data(filepath)
        elif filepath.endswith('.ndjson'):
            df = load_ndjson_data(filepath)
        else:
            return jsonify({'error': 'サポートされていないファイル形式です'})
        
        # データ処理
        result = process_data(df)
        return jsonify(result)
        
    except Exception as e:
        return jsonify({'error': str(e)})

@app.route('/api/get_image')
def get_image():
    """画像を取得してBase64で返す"""
    img_path = request.args.get('path', '')
    
    if not img_path:
        return jsonify({'error': 'パスが指定されていません'})
    
    # 相対パスの場合は絶対パスに変換
    if not os.path.isabs(img_path):
        # imagesディレクトリまたはdataディレクトリをチェック
        if os.path.exists(os.path.join(config.IMAGES_DIRECTORY, img_path)):
            img_path = os.path.join(config.IMAGES_DIRECTORY, img_path)
        elif os.path.exists(os.path.join('data', img_path)):
            img_path = os.path.join('data', img_path)
    
    if not os.path.exists(img_path):
        return jsonify({'error': '画像が見つかりません'})
    
    try:
        # 画像を読み込んでBase64エンコード
        img = Image.open(img_path)
        buffered = BytesIO()
        img.save(buffered, format="JPEG")
        img_base64 = base64.b64encode(buffered.getvalue()).decode('utf-8')
        
        return jsonify({'image': img_base64})
        
    except Exception as e:
        return jsonify({'error': str(e)})

def main():
    """メイン関数"""
    print("走行データビューアを起動します...")
    print(f"http://localhost:5000 でアクセスしてください")
    app.run(host='0.0.0.0', port=5000, debug=True)

if __name__ == '__main__':
    main()