import json

# 读取JSON文件
with open('output_fusion_refactored.json', 'r') as f:
    data = json.load(f)

# 统计每帧的目标数量
print("帧号\t目标数\t时间戳\t\t\t间隔(ms)")
print("-" * 60)

prev_time = None
for i, item in enumerate(data):
    count = len(item['participant'])
    time = item['reportTime']
    
    if prev_time:
        gap = time - prev_time
    else:
        gap = 0
    
    # 只打印目标数变化的帧
    if i == 0 or count != data[i-1]['participant'].__len__():
        print(f"{i}\t{count}\t{time}\t{gap}")
    
    prev_time = time

# 找出目标数最少的帧
print("\n\n目标数最少的帧:")
min_count = min(len(item['participant']) for item in data)
for i, item in enumerate(data):
    if len(item['participant']) == min_count:
        print(f"帧{i}: {min_count}个目标 (时间: {item['reportTime']})")
        # 打印这一帧的目标
        for p in item['participant']:
            print(f"  - {p['source']} pid={p['pid']} plate={p['plate']}")
        
        # 打印前后帧的对比
        if i > 0:
            print(f"\n前一帧(帧{i-1}): {len(data[i-1]['participant'])}个目标")
            for p in data[i-1]['participant']:
                print(f"  - {p['source']} pid={p['pid']} plate={p['plate']}")
        
        if i < len(data) - 1:
            print(f"\n后一帧(帧{i+1}): {len(data[i+1]['participant'])}个目标")
            for p in data[i+1]['participant']:
                print(f"  - {p['source']} pid={p['pid']} plate={p['plate']}")
        
        break
