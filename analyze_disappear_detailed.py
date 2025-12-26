import json
from datetime import datetime

# 读取JSON文件
with open('output_fusion_refactored.json', 'r') as f:
    data = json.load(f)

print("=" * 100)
print("详细分析：目标消失和出现的时刻")
print("=" * 100)

# 跟踪每个pid的出现/消失
pid_timeline = {}

for frame_idx, item in enumerate(data):
    report_time = item['reportTime']
    timestamp = item['participant'][0]['timestamp'] if item['participant'] else None
    
    # 提取所有pid
    current_pids = set()
    for p in item['participant']:
        current_pids.add(p['pid'])
    
    # 初始化timeline
    if frame_idx == 0:
        for pid in current_pids:
            pid_timeline[pid] = {'first_seen': frame_idx, 'last_seen': frame_idx, 'disappear_frames': []}
    else:
        # 检查新出现的pid
        prev_pids = set()
        for p in data[frame_idx-1]['participant']:
            prev_pids.add(p['pid'])
        
        new_pids = current_pids - prev_pids
        disappeared_pids = prev_pids - current_pids
        
        # 更新timeline
        for pid in current_pids:
            if pid not in pid_timeline:
                pid_timeline[pid] = {'first_seen': frame_idx, 'last_seen': frame_idx, 'disappear_frames': []}
            pid_timeline[pid]['last_seen'] = frame_idx
        
        for pid in disappeared_pids:
            if pid in pid_timeline:
                pid_timeline[pid]['disappear_frames'].append(frame_idx)

# 找出所有同时消失的帧
print("\n目标消失事件（按帧号）:")
print("-" * 100)

disappear_events = {}
for pid, info in pid_timeline.items():
    for disappear_frame in info['disappear_frames']:
        if disappear_frame not in disappear_events:
            disappear_events[disappear_frame] = []
        disappear_events[disappear_frame].append(pid)

# 按帧号排序并显示
for frame_idx in sorted(disappear_events.keys()):
    pids = disappear_events[frame_idx]
    if len(pids) >= 3:  # 只显示3个或以上目标同时消失的事件
        report_time = data[frame_idx]['reportTime']
        timestamp = data[frame_idx]['participant'][0]['timestamp'] if data[frame_idx]['participant'] else 'N/A'
        prev_count = len(data[frame_idx-1]['participant']) if frame_idx > 0 else 0
        curr_count = len(data[frame_idx]['participant'])
        
        print(f"\n帧{frame_idx} (时间: {timestamp}, reportTime: {report_time})")
        print(f"  目标数: {prev_count} → {curr_count}")
        print(f"  消失的目标: {pids}")

# 找出所有同时出现的帧
print("\n\n目标出现事件（按帧号）:")
print("-" * 100)

appear_events = {}
for pid, info in pid_timeline.items():
    first_frame = info['first_seen']
    if first_frame > 0:  # 不计第0帧
        if first_frame not in appear_events:
            appear_events[first_frame] = []
        appear_events[first_frame].append(pid)

for frame_idx in sorted(appear_events.keys()):
    pids = appear_events[frame_idx]
    if len(pids) >= 3:  # 只显示3个或以上目标同时出现的事件
        report_time = data[frame_idx]['reportTime']
        timestamp = data[frame_idx]['participant'][0]['timestamp'] if data[frame_idx]['participant'] else 'N/A'
        prev_count = len(data[frame_idx-1]['participant']) if frame_idx > 0 else 0
        curr_count = len(data[frame_idx]['participant'])
        
        print(f"\n帧{frame_idx} (时间: {timestamp}, reportTime: {report_time})")
        print(f"  目标数: {prev_count} → {curr_count}")
        print(f"  出现的目标: {pids}")

# 分析时间戳间隔
print("\n\n时间戳分析:")
print("-" * 100)

times = [item['reportTime'] for item in data]
gaps = [times[i+1] - times[i] for i in range(len(times)-1)]

print(f"总帧数: {len(times)}")
print(f"平均间隔: {sum(gaps)/len(gaps):.1f} ms")
print(f"最大间隔: {max(gaps)} ms (在帧{gaps.index(max(gaps))}-{gaps.index(max(gaps))+1})")

# 找出大间隔（>500ms）
large_gaps = [(i, gaps[i]) for i in range(len(gaps)) if gaps[i] > 500]
if large_gaps:
    print(f"\n大时间间隔(>500ms): {len(large_gaps)}个")
    for i, gap in large_gaps:
        print(f"  帧{i}-{i+1}: {gap} ms")
