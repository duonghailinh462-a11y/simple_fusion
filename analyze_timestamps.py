import json

# 读取JSON文件
with open('output_fusion_refactored.json', 'r') as f:
    data = json.load(f)

# 提取reportTime
times = [item['reportTime'] for item in data]

print(f'总帧数: {len(times)}')
print(f'首帧时间: {times[0]}')
print(f'末帧时间: {times[-1]}')
print(f'总耗时: {times[-1] - times[0]} ms')

# 计算间隔
gaps = [times[i+1] - times[i] for i in range(len(times)-1)]

print(f'\n时间间隔统计:')
print(f'  平均间隔: {sum(gaps)/len(gaps):.1f} ms')
print(f'  最小间隔: {min(gaps)} ms')
print(f'  最大间隔: {max(gaps)} ms')

# 找出异常间隔
jumps = [(i, gaps[i]) for i in range(len(gaps)) if gaps[i] > 100]
print(f'\n时间跳跃(>100ms): {len(jumps)} 个')
if jumps:
    print('前15个跳跃:')
    for i, gap in jumps[:15]:
        print(f'  帧{i}-{i+1}: {gap} ms')

# 分析目标消失的时间段
print('\n\n分析目标消失的时间段:')
print('查找所有目标都消失的帧...')

# 统计每帧的目标数量
target_counts = []
for item in data:
    count = len(item['participant'])
    target_counts.append(count)

# 找出目标数量为0的帧
zero_frames = [i for i in range(len(target_counts)) if target_counts[i] == 0]
print(f'目标数量为0的帧: {len(zero_frames)} 个')
if zero_frames:
    print(f'  首次: 帧{zero_frames[0]} (时间: {times[zero_frames[0]]})')
    if len(zero_frames) > 1:
        print(f'  最后: 帧{zero_frames[-1]} (时间: {times[zero_frames[-1]]})')

# 找出目标数量最少的帧
min_count = min(target_counts)
min_frames = [i for i in range(len(target_counts)) if target_counts[i] == min_count]
print(f'\n目标数量最少: {min_count} 个')
print(f'  出现在 {len(min_frames)} 帧')
if min_frames:
    print(f'  首次: 帧{min_frames[0]} (时间: {times[min_frames[0]]})')
