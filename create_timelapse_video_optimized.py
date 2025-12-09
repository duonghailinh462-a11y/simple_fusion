import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
from matplotlib.patches import Polygon, FancyArrowPatch
from matplotlib.backends.backend_agg import FigureCanvasAgg
import numpy as np
import cv2
import json
import argparse
import sys
from collections import defaultdict
from datetime import datetime

# --- 1. 参数与设置 ---
parser = argparse.ArgumentParser(description='Generate traffic timelapse with Fusion + Radar data')
parser.add_argument('--center-time', type=int, default=None, help='Center time point')
parser.add_argument('--frame-range', type=int, default=20, help='Frames before/after center')
parser.add_argument('--start-time', type=int, default=None, help='Start time point')
parser.add_argument('--end-time', type=int, default=None, help='End time point')
parser.add_argument('--output', type=str, default='traffic_timelapse_fusion_radar.mp4', help='Output file')
parser.add_argument('--fusion-file', type=str, default='output_fusion_refactored.json', help='Fusion JSON file')
parser.add_argument('--radar-file', type=str, default='radar_data_cleaned.jsonl', help='Radar JSONL file')
parser.add_argument('--show-radar', action='store_true', default=True, help='Show radar data')
parser.add_argument('--show-fusion', action='store_true', default=True, help='Show fusion data')
parser.add_argument('--fps', type=int, default=25, help='Video FPS')
args = parser.parse_args()

HDCROSSWALK_FILE = 'hdcrosswalk.json'
OUTPUT_FUSION_FILE = args.fusion_file
RADAR_DATA_FILE = args.radar_file
PUBLIC_AREA = np.array([[113.5843538542, 23.5307406946], [113.5843393637, 23.5307811804],
                        [113.5840623185, 23.5305610454], [113.5840976830, 23.5305455610]])
COLORS_FUSION = [(1.0, 0.42, 0.42), (0.31, 0.80, 0.77), (0.27, 0.72, 0.82), (1.0, 0.63, 0.48), 
                 (0.60, 0.85, 0.78), (0.97, 0.86, 0.44), (0.73, 0.56, 0.81), (0.52, 0.76, 0.88)]
COLORS_RADAR = [(0.91, 0.30, 0.24), (0.20, 0.60, 0.86), (0.18, 0.80, 0.44), (0.95, 0.61, 0.07),
                (0.61, 0.35, 0.71), (0.10, 0.74, 0.61), (0.90, 0.49, 0.13), (0.21, 0.27, 0.37)]
FPS = args.fps

# --- 2. 数据加载 ---
print("Loading data...")

# 加载融合数据
try:
    with open(OUTPUT_FUSION_FILE, 'r') as f:
        fusion_data = json.load(f)
except FileNotFoundError:
    print(f"Warning: {OUTPUT_FUSION_FILE} not found, using empty fusion data")
    fusion_data = []

# 加载雷达数据
radar_by_timestamp = defaultdict(list)
radar_timestamps_sorted = []
try:
    with open(RADAR_DATA_FILE, 'r') as f:
        for line in f:
            if line.strip():
                data_line = json.loads(line)
                ts = data_line.get('time', '')
                radar_by_timestamp[ts].extend(data_line.get('locusList', []))
    radar_timestamps_sorted = sorted(radar_by_timestamp.keys())
    print(f"Loaded {len(radar_by_timestamp)} radar timestamps")
except FileNotFoundError:
    print(f"Warning: {RADAR_DATA_FILE} not found, using empty radar data")

# 加载地图数据（可选）
crosswalks = []
try:
    with open(HDCROSSWALK_FILE, 'r') as f:
        hd_data = json.load(f)
        for f_item in hd_data.get('features', []):
            if f_item.get('geometry', {}).get('type') == 'MultiPolygon':
                for poly in f_item['geometry']['coordinates']:
                    for ring in poly:
                        ring_2d = [p[:2] for p in ring]
                        crosswalks.append(ring_2d)
except FileNotFoundError:
    print(f"Warning: {HDCROSSWALK_FILE} not found, skipping crosswalks")

# 按时间顺序合并融合数据和雷达数据
# 只保留两者都有数据的时间范围
timeline = {}  # {timestamp: {'fusion': [...], 'radar': [...]}}

# 添加融合数据
fusion_timestamps = set()
for r in fusion_data:
    if r.get('participant'):
        # 使用第一个 participant 的 timestamp
        ts = r['participant'][0].get('timestamp', '')
        if ts:
            if ts not in timeline:
                timeline[ts] = {'fusion': [], 'radar': []}
            timeline[ts]['fusion'] = r['participant']
            fusion_timestamps.add(ts)

# 添加雷达数据
radar_timestamps = set(radar_by_timestamp.keys())
for ts, objs in radar_by_timestamp.items():
    if ts not in timeline:
        timeline[ts] = {'fusion': [], 'radar': []}
    timeline[ts]['radar'] = objs

# 只保留两者都有数据的时间戳
if fusion_timestamps and radar_timestamps:
    # 找到时间范围的交集
    fusion_times = sorted([datetime.strptime(ts, '%Y-%m-%d %H:%M:%S.%f').timestamp() for ts in fusion_timestamps])
    radar_times = sorted([datetime.strptime(ts, '%Y-%m-%d %H:%M:%S.%f').timestamp() for ts in radar_timestamps])
    
    time_start = max(fusion_times[0], radar_times[0])
    time_end = min(fusion_times[-1], radar_times[-1])
    
    # 过滤出时间范围内的时间戳
    valid_timestamps = []
    for ts in timeline.keys():
        try:
            ts_seconds = datetime.strptime(ts, '%Y-%m-%d %H:%M:%S.%f').timestamp()
            if time_start <= ts_seconds <= time_end:
                valid_timestamps.append(ts)
        except:
            pass
    
    sorted_timestamps = sorted(valid_timestamps)
    print(f"Time range: {datetime.fromtimestamp(time_start)} ~ {datetime.fromtimestamp(time_end)}")
else:
    sorted_timestamps = sorted(timeline.keys())

print(f"Total timestamps: {len(sorted_timestamps)} (fusion: {len([t for t in sorted_timestamps if timeline[t]['fusion']])}, radar: {len([t for t in sorted_timestamps if timeline[t]['radar']])})")

# --- 3. 计算范围 ---
all_pts = [p for coords in crosswalks for p in coords] + \
          [[p['lon'], p['lat']] for ts in sorted_timestamps for p in timeline[ts]['fusion']] + \
          [[obj['longitude'], obj['latitude']] for ts in sorted_timestamps for obj in timeline[ts]['radar']]

if all_pts:
    lons, lats = zip(*all_pts)
    xlim = (min(lons) - 0.0002, max(lons) + 0.0002)
    ylim = (min(lats) - 0.0002, max(lats) + 0.0002)
else:
    xlim = (113.583, 113.586)
    ylim = (23.530, 23.532)

# --- 4. 初始化图形 ---
fig, ax = plt.subplots(figsize=(14, 11))
canvas = FigureCanvasAgg(fig)
ax.set_xlim(xlim)
ax.set_ylim(ylim)
ax.set_aspect('equal')
ax.grid(True, alpha=0.2, linestyle='--')
ax.set_xlabel('Longitude', fontsize=10)
ax.set_ylabel('Latitude', fontsize=10)

# 绘制地图元素
for i, c in enumerate(crosswalks):
    ax.add_patch(Polygon(c, closed=True, facecolor='lightblue', edgecolor='blue', alpha=0.3, 
                         label='Crosswalk' if i == 0 else ""))
ax.add_patch(Polygon(PUBLIC_AREA, closed=True, facecolor='lightgreen', edgecolor='darkgreen', 
                     alpha=0.3, label='Public Area'))

title = ax.set_title("", fontsize=14, fontweight='bold')
info_text = ax.text(0.02, 0.98, "", transform=ax.transAxes, fontsize=9, verticalalignment='top',
                    bbox=dict(boxstyle='round', facecolor='wheat', alpha=0.8))
artists = []
fusion_colors = {}
radar_colors = {}

# --- 5. 辅助函数 ---
def draw_fusion_object(ax, p, color):
    """绘制融合对象"""
    marker_map = {'car': 'o', 'truck': 's', 'bus': 'D', 'motorcycle': '^', 'bicycle': 'v'}
    size_map = {'car': 80, 'truck': 100, 'bus': 120, 'motorcycle': 60, 'bicycle': 50}
    
    obj_type = p.get('type', 'car')
    marker = marker_map.get(obj_type, 'o')
    size = size_map.get(obj_type, 80)
    
    scatter = ax.scatter(p['lon'], p['lat'], color=color, marker=marker, s=size, alpha=0.8,
                        edgecolors='white', linewidth=1, zorder=8)
    
    label_text = f"T{p.get('track_id', -1)}"
    if p.get('confidence', 0) > 0:
        label_text += f"|c:{p['confidence']:.2f}"
    
    annot = ax.annotate(label_text, (p['lon'], p['lat']), xytext=(6, -6),
                       textcoords='offset points', fontsize=6, color='white', fontweight='bold',
                       bbox=dict(boxstyle='round,pad=0.2', facecolor=color, alpha=0.8, edgecolor='white', linewidth=0.5))
    
    return scatter, annot

def draw_radar_object(ax, obj, color):
    """绘制雷达对象"""
    type_map = {1: 'car', 2: 'truck', 3: 'bus', 4: 'motorcycle', 5: 'bicycle'}
    obj_type = type_map.get(obj.get('objType', 1), 'unknown')
    
    scatter = ax.scatter(obj['longitude'], obj['latitude'], color=color, marker='x', s=100, 
                        alpha=0.9, linewidth=2, zorder=9)
    
    label_text = f"R{obj.get('id', '')[:6]}"
    if obj.get('speed', 0) > 0:
        label_text += f"|v:{obj['speed']:.1f}"
    
    annot = ax.annotate(label_text, (obj['longitude'], obj['latitude']), xytext=(6, -6),
                       textcoords='offset points', fontsize=6, color='white', fontweight='bold',
                       bbox=dict(boxstyle='round,pad=0.2', facecolor=color, alpha=0.8, edgecolor='white', linewidth=0.5))
    
    return scatter, annot

# --- 6. 视频生成循环 ---
canvas.draw()
h, w, _ = np.array(canvas.renderer.buffer_rgba()).shape
writer = cv2.VideoWriter(args.output, cv2.VideoWriter_fourcc(*'mp4v'), FPS, (w, h))

print(f"Generating video: {w}x{h} @ {FPS}fps")

for i, ts in enumerate(sorted_timestamps):
    # 清空之前的艺术元素
    for a in artists:
        a.remove()
    artists.clear()
    
    # 更新标题
    title.set_text(f'Time: {ts}')
    
    # 获取当前时间点的数据
    detection_objects = timeline[ts]['fusion']
    radar_objects = timeline[ts]['radar']
    
    # 绘制检测数据
    if args.show_fusion:
        for p in detection_objects:
            track_id = p.get('track_id', -1)
            if track_id not in fusion_colors:
                fusion_colors[track_id] = COLORS_FUSION[len(fusion_colors) % len(COLORS_FUSION)]
            
            scatter, annot = draw_fusion_object(ax, p, fusion_colors[track_id])
            artists.extend([scatter, annot])
    
    # 绘制雷达数据
    if args.show_radar:
        for obj in radar_objects:
            radar_id = obj.get('id', '')
            if radar_id not in radar_colors:
                radar_colors[radar_id] = COLORS_RADAR[len(radar_colors) % len(COLORS_RADAR)]
            
            scatter, annot = draw_radar_object(ax, obj, radar_colors[radar_id])
            artists.extend([scatter, annot])
    
    # 更新统计信息（只显示当前时间点的数据）
    detection_count = len(timeline[ts]['fusion'])
    radar_count = len(timeline[ts]['radar'])
    info_text.set_text(f'Detection: {detection_count} | Radar: {radar_count} | Frame: {i+1}/{len(sorted_timestamps)}')
    
    # 绘制并写入视频
    canvas.draw()
    img = np.asarray(canvas.buffer_rgba())[:, :, :3]
    writer.write(cv2.cvtColor(img, cv2.COLOR_RGB2BGR))
    
    if (i + 1) % max(1, len(sorted_timestamps) // 10) == 0:
        print(f"Processed {i+1}/{len(sorted_timestamps)} frames")

writer.release()
print(f"Done! Saved to {args.output}")