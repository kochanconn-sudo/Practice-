#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
LiDAR点群データビューア Webアプリケーション
記録済みのLiDAR npyファイル（donkeycar形式）をブラウザから可視化・再生・分析
"""

import os
import sys
import json
import glob
import numpy as np
import base64
from flask import Flask, render_template_string, jsonify, request

app = Flask(__name__)

DATA_DIR = os.path.join(os.path.dirname(os.path.dirname(os.path.abspath(__file__))), 'data')

HTML_TEMPLATE = '''
<!DOCTYPE html>
<html>
<head>
    <title>LiDAR点群ビューア</title>
    <meta charset="utf-8">
    <style>
        * { box-sizing: border-box; margin: 0; padding: 0; }
        body {
            font-family: 'Segoe UI', Arial, sans-serif;
            background: #1a1a2e;
            color: #e0e0e0;
        }
        .header {
            background: #16213e;
            padding: 12px 24px;
            border-bottom: 2px solid #0f3460;
            display: flex;
            align-items: center;
            gap: 20px;
        }
        .header h1 {
            font-size: 1.2em;
            color: #00d4ff;
            white-space: nowrap;
        }
        .header select {
            padding: 6px 12px;
            background: #0f3460;
            color: #e0e0e0;
            border: 1px solid #00d4ff;
            border-radius: 4px;
            font-size: 0.9em;
            min-width: 280px;
        }
        .header .session-info {
            font-size: 0.85em;
            color: #8899aa;
        }
        .main {
            display: grid;
            grid-template-columns: 1fr 320px;
            grid-template-rows: 1fr auto;
            height: calc(100vh - 52px);
            gap: 0;
        }
        .canvas-area {
            position: relative;
            background: #0a0a1a;
            display: flex;
            align-items: center;
            justify-content: center;
            overflow: hidden;
        }
        #lidarCanvas {
            background: #0a0a1a;
        }
        .sidebar {
            background: #16213e;
            padding: 16px;
            overflow-y: auto;
            border-left: 1px solid #0f3460;
            display: flex;
            flex-direction: column;
            gap: 12px;
        }
        .panel {
            background: #1a1a2e;
            border: 1px solid #0f3460;
            border-radius: 6px;
            padding: 12px;
        }
        .panel h3 {
            font-size: 0.85em;
            color: #00d4ff;
            margin-bottom: 8px;
            text-transform: uppercase;
            letter-spacing: 0.5px;
        }
        .stat-grid {
            display: grid;
            grid-template-columns: 1fr 1fr;
            gap: 6px;
        }
        .stat-item {
            background: #0f3460;
            padding: 6px 8px;
            border-radius: 4px;
            font-size: 0.8em;
        }
        .stat-item .label {
            color: #8899aa;
            font-size: 0.75em;
        }
        .stat-item .value {
            color: #00ff88;
            font-weight: bold;
            font-size: 1.1em;
        }
        .camera-preview img {
            width: 100%;
            border-radius: 4px;
            border: 1px solid #0f3460;
        }
        .camera-preview .no-image {
            width: 100%;
            height: 120px;
            background: #0a0a1a;
            border-radius: 4px;
            display: flex;
            align-items: center;
            justify-content: center;
            color: #555;
            font-size: 0.85em;
        }
        #histCanvas {
            width: 100%;
            height: 80px;
            border-radius: 4px;
        }
        .controls {
            grid-column: 1 / -1;
            background: #16213e;
            padding: 10px 20px;
            border-top: 1px solid #0f3460;
            display: flex;
            align-items: center;
            gap: 12px;
        }
        .controls button {
            background: #0f3460;
            color: #00d4ff;
            border: 1px solid #00d4ff;
            border-radius: 4px;
            padding: 6px 14px;
            cursor: pointer;
            font-size: 0.9em;
            white-space: nowrap;
        }
        .controls button:hover {
            background: #00d4ff;
            color: #0a0a1a;
        }
        .controls button.active {
            background: #00d4ff;
            color: #0a0a1a;
        }
        .controls input[type="range"] {
            flex: 1;
            accent-color: #00d4ff;
        }
        .controls .frame-info {
            font-size: 0.85em;
            color: #8899aa;
            min-width: 100px;
            text-align: center;
        }
        .controls label {
            font-size: 0.8em;
            color: #8899aa;
        }
        .controls select {
            background: #0f3460;
            color: #e0e0e0;
            border: 1px solid #0f3460;
            border-radius: 4px;
            padding: 4px 8px;
            font-size: 0.85em;
        }
        .session-list {
            position: fixed;
            top: 0; left: 0; right: 0; bottom: 0;
            background: rgba(0,0,0,0.8);
            display: flex;
            align-items: center;
            justify-content: center;
            z-index: 100;
        }
        .session-list.hidden { display: none; }
        .session-list-inner {
            background: #16213e;
            border: 1px solid #0f3460;
            border-radius: 10px;
            padding: 24px;
            max-width: 600px;
            width: 90%;
            max-height: 80vh;
            overflow-y: auto;
        }
        .session-list-inner h2 {
            color: #00d4ff;
            margin-bottom: 16px;
        }
        .session-card {
            background: #1a1a2e;
            border: 1px solid #0f3460;
            border-radius: 6px;
            padding: 12px 16px;
            margin-bottom: 8px;
            cursor: pointer;
            transition: border-color 0.2s;
        }
        .session-card:hover {
            border-color: #00d4ff;
        }
        .session-card .name {
            color: #00d4ff;
            font-weight: bold;
            margin-bottom: 4px;
        }
        .session-card .meta {
            font-size: 0.8em;
            color: #8899aa;
        }
    </style>
</head>
<body>
    <div class="header">
        <h1>LiDAR Viewer</h1>
        <select id="sessionSelect" onchange="loadSession(this.value)">
            <option value="">-- セッション選択 --</option>
        </select>
        <span class="session-info" id="sessionInfo"></span>
    </div>

    <div class="main">
        <div class="canvas-area">
            <canvas id="lidarCanvas"></canvas>
        </div>
        <div class="sidebar">
            <div class="panel">
                <h3>Statistics</h3>
                <div class="stat-grid" id="statsGrid">
                    <div class="stat-item"><div class="label">Points</div><div class="value" id="statPoints">-</div></div>
                    <div class="stat-item"><div class="label">Valid</div><div class="value" id="statValid">-</div></div>
                    <div class="stat-item"><div class="label">Min dist</div><div class="value" id="statMin">-</div></div>
                    <div class="stat-item"><div class="label">Max dist</div><div class="value" id="statMax">-</div></div>
                    <div class="stat-item"><div class="label">Mean dist</div><div class="value" id="statMean">-</div></div>
                    <div class="stat-item"><div class="label">Std dev</div><div class="value" id="statStd">-</div></div>
                </div>
            </div>
            <div class="panel">
                <h3>Distance Histogram</h3>
                <canvas id="histCanvas"></canvas>
            </div>
            <div class="panel">
                <h3>Camera</h3>
                <div class="camera-preview" id="cameraPreview">
                    <div class="no-image">No image</div>
                </div>
            </div>
        </div>
        <div class="controls">
            <button id="btnPrev" onclick="prevFrame()">&lt;</button>
            <button id="btnPlay" onclick="togglePlay()">Play</button>
            <button id="btnNext" onclick="nextFrame()">&gt;</button>
            <input type="range" id="frameSlider" min="0" max="0" value="0" oninput="seekFrame(parseInt(this.value))">
            <span class="frame-info" id="frameInfo">0 / 0</span>
            <label>Speed:</label>
            <select id="speedSelect" onchange="setSpeed(this.value)">
                <option value="200">5 fps</option>
                <option value="100" selected>10 fps</option>
                <option value="50">20 fps</option>
                <option value="33">30 fps</option>
            </select>
        </div>
    </div>

    <div class="session-list" id="sessionListOverlay">
        <div class="session-list-inner">
            <h2>LiDAR Sessions</h2>
            <div id="sessionListCards">Loading...</div>
        </div>
    </div>

    <script>
    const state = {
        session: null,
        meta: null,
        frameIndex: 0,
        frameCount: 0,
        playing: false,
        playInterval: null,
        playSpeed: 100,
        frameCache: {},
        angles: null
    };

    async function init() {
        const resp = await fetch('/api/sessions');
        const sessions = await resp.json();
        const select = document.getElementById('sessionSelect');
        const cards = document.getElementById('sessionListCards');
        cards.innerHTML = '';

        sessions.forEach(s => {
            const opt = document.createElement('option');
            opt.value = s.name;
            opt.textContent = s.name;
            select.appendChild(opt);

            const card = document.createElement('div');
            card.className = 'session-card';
            card.innerHTML = `<div class="name">${s.name}</div>
                <div class="meta">${s.lidar_type || 'Unknown'} | ${s.frame_count} frames | ${s.created || ''}</div>`;
            card.onclick = () => {
                select.value = s.name;
                loadSession(s.name);
            };
            cards.appendChild(card);
        });

        if (sessions.length === 0) {
            cards.innerHTML = '<p style="color:#888">No LiDAR sessions found in data/</p>';
        }
    }

    async function loadSession(name) {
        if (!name) return;
        document.getElementById('sessionListOverlay').classList.add('hidden');

        const resp = await fetch(`/api/session/${name}`);
        const info = await resp.json();
        if (info.error) { alert(info.error); return; }

        state.session = name;
        state.meta = info;
        state.frameCount = info.frame_count;
        state.frameIndex = 0;
        state.frameCache = {};

        // Build angle array from metadata
        const aStart = info.angle_start * Math.PI / 180;
        const aEnd = info.angle_end * Math.PI / 180;
        const nPts = info.data_points;
        state.angles = new Float64Array(nPts);
        for (let i = 0; i < nPts; i++) {
            state.angles[i] = aStart + (aEnd - aStart) * i / (nPts - 1);
        }
        if (info.clockwise) {
            state.angles.reverse();
        }

        document.getElementById('sessionInfo').textContent =
            `${info.lidar_type} | ${info.data_points} pts | ${info.angle_start} to ${info.angle_end} deg`;

        const slider = document.getElementById('frameSlider');
        slider.max = state.frameCount - 1;
        slider.value = 0;

        resizeCanvas();
        loadFrame(0);
    }

    async function loadFrame(index) {
        if (!state.session || index < 0 || index >= state.frameCount) return;
        state.frameIndex = index;

        document.getElementById('frameSlider').value = index;
        document.getElementById('frameInfo').textContent = `${index} / ${state.frameCount - 1}`;

        let data = state.frameCache[index];
        if (!data) {
            const resp = await fetch(`/api/frame/${state.session}/${index}`);
            data = await resp.json();
            if (data.error) return;
            state.frameCache[index] = data;
        }

        drawLidar(data.distances);
        updateStats(data.stats);
        drawHistogram(data.stats.histogram);
        loadCameraImage(index);
    }

    function drawLidar(distances) {
        const canvas = document.getElementById('lidarCanvas');
        const ctx = canvas.getContext('2d');
        const w = canvas.width;
        const h = canvas.height;
        const cx = w / 2;
        const cy = h / 2;

        ctx.clearRect(0, 0, w, h);

        // Determine scale: fit max_range_mm into canvas
        const maxRange = 3000; // 3m in mm
        const radius = Math.min(cx, cy) - 30;
        const scale = radius / maxRange;

        // Draw grid rings
        ctx.strokeStyle = '#1a3a5c';
        ctx.lineWidth = 1;
        ctx.setLineDash([4, 4]);
        const ringSteps = [500, 1000, 1500, 2000, 2500, 3000];
        ctx.font = '11px sans-serif';
        ctx.fillStyle = '#4a6a8a';
        ringSteps.forEach(r => {
            const pr = r * scale;
            ctx.beginPath();
            ctx.arc(cx, cy, pr, 0, Math.PI * 2);
            ctx.stroke();
            ctx.fillText((r / 1000).toFixed(1) + 'm', cx + 4, cy - pr + 14);
        });
        ctx.setLineDash([]);

        // Draw axis lines
        ctx.strokeStyle = '#1a3a5c';
        ctx.lineWidth = 0.5;
        ctx.beginPath();
        ctx.moveTo(cx, 0); ctx.lineTo(cx, h);
        ctx.moveTo(0, cy); ctx.lineTo(w, cy);
        ctx.stroke();

        // Draw angle labels
        ctx.fillStyle = '#4a6a8a';
        ctx.font = '10px sans-serif';
        ctx.fillText('0 (front)', cx + 4, 14);
        ctx.fillText('180', cx + 4, h - 6);
        ctx.fillText('90', w - 24, cy - 4);
        ctx.fillText('-90', 4, cy - 4);

        // Draw points
        if (!state.angles || !distances) return;
        const n = Math.min(distances.length, state.angles.length);

        ctx.fillStyle = '#00ff88';
        for (let i = 0; i < n; i++) {
            const d = distances[i];
            if (d <= 0) continue;

            const a = state.angles[i];
            // LiDAR convention: 0 deg = forward, positive = CCW
            // Canvas: x=right, y=down. Forward = up (-y)
            const px = cx + d * scale * Math.sin(a);
            const py = cy - d * scale * Math.cos(a);

            // Color by distance
            const t = Math.min(d / maxRange, 1);
            const r2 = Math.floor(255 * (1 - t));
            const g2 = Math.floor(255 * t);
            ctx.fillStyle = `rgb(${r2}, ${g2}, 80)`;

            ctx.fillRect(px - 1.5, py - 1.5, 3, 3);
        }

        // Draw vehicle marker
        ctx.fillStyle = '#00d4ff';
        ctx.beginPath();
        ctx.moveTo(cx, cy - 10);
        ctx.lineTo(cx - 6, cy + 6);
        ctx.lineTo(cx + 6, cy + 6);
        ctx.closePath();
        ctx.fill();
    }

    function updateStats(stats) {
        if (!stats) return;
        document.getElementById('statPoints').textContent = stats.total_points;
        document.getElementById('statValid').textContent = stats.valid_points;
        document.getElementById('statMin').textContent = (stats.min_dist / 1000).toFixed(2) + 'm';
        document.getElementById('statMax').textContent = (stats.max_dist / 1000).toFixed(2) + 'm';
        document.getElementById('statMean').textContent = (stats.mean_dist / 1000).toFixed(2) + 'm';
        document.getElementById('statStd').textContent = (stats.std_dist / 1000).toFixed(2) + 'm';
    }

    function drawHistogram(hist) {
        const canvas = document.getElementById('histCanvas');
        const ctx = canvas.getContext('2d');
        const w = canvas.width;
        const h = canvas.height;
        ctx.clearRect(0, 0, w, h);

        if (!hist || !hist.counts || hist.counts.length === 0) return;

        const counts = hist.counts;
        const edges = hist.edges;
        const maxCount = Math.max(...counts);
        if (maxCount === 0) return;

        const barW = w / counts.length;
        for (let i = 0; i < counts.length; i++) {
            const barH = (counts[i] / maxCount) * (h - 16);
            const t = i / counts.length;
            const r = Math.floor(255 * (1 - t));
            const g = Math.floor(255 * t);
            ctx.fillStyle = `rgb(${r}, ${g}, 80)`;
            ctx.fillRect(i * barW, h - 16 - barH, barW - 1, barH);
        }

        // Labels
        ctx.fillStyle = '#8899aa';
        ctx.font = '9px sans-serif';
        ctx.fillText('0m', 0, h - 2);
        ctx.fillText((edges[edges.length - 1] / 1000).toFixed(1) + 'm', w - 30, h - 2);
    }

    async function loadCameraImage(index) {
        const preview = document.getElementById('cameraPreview');
        try {
            const resp = await fetch(`/api/image/${state.session}/${index}`);
            const data = await resp.json();
            if (data.image) {
                preview.innerHTML = `<img src="data:image/jpeg;base64,${data.image}" alt="Frame ${index}">`;
            } else {
                preview.innerHTML = '<div class="no-image">No image</div>';
            }
        } catch {
            preview.innerHTML = '<div class="no-image">No image</div>';
        }
    }

    function seekFrame(index) {
        loadFrame(index);
    }

    function prevFrame() {
        if (state.frameIndex > 0) loadFrame(state.frameIndex - 1);
    }

    function nextFrame() {
        if (state.frameIndex < state.frameCount - 1) loadFrame(state.frameIndex + 1);
    }

    function togglePlay() {
        state.playing = !state.playing;
        const btn = document.getElementById('btnPlay');
        if (state.playing) {
            btn.textContent = 'Pause';
            btn.classList.add('active');
            playLoop();
        } else {
            btn.textContent = 'Play';
            btn.classList.remove('active');
            if (state.playInterval) clearTimeout(state.playInterval);
        }
    }

    function playLoop() {
        if (!state.playing) return;
        if (state.frameIndex >= state.frameCount - 1) {
            state.frameIndex = -1;
        }
        loadFrame(state.frameIndex + 1).then(() => {
            state.playInterval = setTimeout(playLoop, state.playSpeed);
        });
    }

    function setSpeed(ms) {
        state.playSpeed = parseInt(ms);
    }

    function resizeCanvas() {
        const canvas = document.getElementById('lidarCanvas');
        const area = canvas.parentElement;
        const size = Math.min(area.clientWidth, area.clientHeight) - 8;
        canvas.width = size;
        canvas.height = size;

        // Resize histogram canvas
        const hc = document.getElementById('histCanvas');
        hc.width = hc.parentElement.clientWidth - 24;
        hc.height = 80;
    }

    window.addEventListener('resize', () => {
        resizeCanvas();
        if (state.session && state.frameCache[state.frameIndex]) {
            drawLidar(state.frameCache[state.frameIndex].distances);
            drawHistogram(state.frameCache[state.frameIndex].stats.histogram);
        }
    });

    // Keyboard shortcuts
    document.addEventListener('keydown', e => {
        if (e.key === 'ArrowLeft') prevFrame();
        else if (e.key === 'ArrowRight') nextFrame();
        else if (e.key === ' ') { e.preventDefault(); togglePlay(); }
    });

    init();
    </script>
</body>
</html>
'''


def get_sessions():
    """data/ 内の LiDAR データを持つセッション一覧を返す"""
    sessions = []
    if not os.path.isdir(DATA_DIR):
        return sessions

    for name in sorted(os.listdir(DATA_DIR)):
        session_dir = os.path.join(DATA_DIR, name)
        lidar_dir = os.path.join(session_dir, 'lidar')
        if not name.startswith('data_') or not os.path.isdir(lidar_dir):
            continue

        info = {'name': name, 'lidar_type': '', 'frame_count': 0, 'created': ''}

        # Count frames
        npy_files = glob.glob(os.path.join(lidar_dir, '*_lidar_distance_array_.npy'))
        info['frame_count'] = len(npy_files)

        # Read manifest
        manifest_path = os.path.join(session_dir, 'manifest.json')
        if os.path.exists(manifest_path):
            try:
                with open(manifest_path, 'r') as f:
                    lines = f.read().strip().split('\n')
                for line in lines:
                    obj = json.loads(line)
                    if 'lidar_type' in obj:
                        info['lidar_type'] = obj.get('lidar_type', '')
                    if 'created_at' in obj:
                        import datetime
                        ts = obj['created_at']
                        info['created'] = datetime.datetime.fromtimestamp(ts).strftime('%Y-%m-%d %H:%M:%S')
            except Exception:
                pass

        sessions.append(info)

    return sessions


def get_session_info(name):
    """セッションの詳細情報を返す"""
    session_dir = os.path.join(DATA_DIR, name)
    lidar_dir = os.path.join(session_dir, 'lidar')

    if not os.path.isdir(lidar_dir):
        return {'error': 'Session not found'}

    info = {
        'name': name,
        'lidar_type': 'Unknown',
        'angle_start': -135,
        'angle_end': 135,
        'clockwise': False,
        'data_points': 1081,
        'frame_count': 0
    }

    # Count frames
    npy_files = glob.glob(os.path.join(lidar_dir, '*_lidar_distance_array_.npy'))
    info['frame_count'] = len(npy_files)

    # Read manifest
    manifest_path = os.path.join(session_dir, 'manifest.json')
    if os.path.exists(manifest_path):
        try:
            with open(manifest_path, 'r') as f:
                lines = f.read().strip().split('\n')
            for line in lines:
                obj = json.loads(line)
                if 'lidar_type' in obj:
                    info['lidar_type'] = obj.get('lidar_type', 'Unknown')
                    info['angle_start'] = obj.get('lidar_angle_start', -135)
                    info['angle_end'] = obj.get('lidar_angle_end', 135)
                    info['clockwise'] = obj.get('lidar_clockwise', False)
                    info['data_points'] = obj.get('lidar_data_points', 1081)
        except Exception:
            pass

    return info


def load_frame(session_name, index):
    """指定フレームのLiDARデータを読み込む"""
    lidar_dir = os.path.join(DATA_DIR, session_name, 'lidar')
    npy_path = os.path.join(lidar_dir, f'{index}_lidar_distance_array_.npy')

    if not os.path.exists(npy_path):
        return {'error': f'Frame {index} not found'}

    distances = np.load(npy_path)
    dist_list = distances.tolist()

    # Statistics (valid points only, distance > 0)
    valid = distances[distances > 0]
    stats = {
        'total_points': len(distances),
        'valid_points': int(len(valid)),
        'min_dist': float(valid.min()) if len(valid) > 0 else 0,
        'max_dist': float(valid.max()) if len(valid) > 0 else 0,
        'mean_dist': float(valid.mean()) if len(valid) > 0 else 0,
        'std_dist': float(valid.std()) if len(valid) > 0 else 0,
    }

    # Histogram
    if len(valid) > 0:
        counts, edges = np.histogram(valid, bins=30)
        stats['histogram'] = {
            'counts': counts.tolist(),
            'edges': edges.tolist()
        }
    else:
        stats['histogram'] = {'counts': [], 'edges': []}

    return {'distances': dist_list, 'stats': stats}


def load_camera_image(session_name, index):
    """カメラ画像をbase64で返す"""
    images_dir = os.path.join(DATA_DIR, session_name, 'images')

    # Try common naming patterns
    patterns = [
        f'{index}_cam0_image_array_.jpg',
        f'{index}_cam_image_array_.jpg',
        f'{index}_cam0_image_array_.png',
    ]

    for fname in patterns:
        img_path = os.path.join(images_dir, fname)
        if os.path.exists(img_path):
            with open(img_path, 'rb') as f:
                img_data = base64.b64encode(f.read()).decode('utf-8')
            return {'image': img_data}

    return {'image': None}


@app.route('/')
def index():
    return render_template_string(HTML_TEMPLATE)


@app.route('/api/sessions')
def api_sessions():
    return jsonify(get_sessions())


@app.route('/api/session/<name>')
def api_session(name):
    return jsonify(get_session_info(name))


@app.route('/api/frame/<session>/<int:idx>')
def api_frame(session, idx):
    return jsonify(load_frame(session, idx))


@app.route('/api/image/<session>/<int:idx>')
def api_image(session, idx):
    return jsonify(load_camera_image(session, idx))


def main():
    print("LiDAR点群ビューアを起動します...")
    print("http://localhost:5001 でアクセスしてください")
    app.run(host='0.0.0.0', port=5001, debug=True)


if __name__ == '__main__':
    main()
