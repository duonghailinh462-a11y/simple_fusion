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
    print('所有跳跃:')
    for i, gap in jumps:
        print(f'  帧{i}-{i+1}: {gap} ms')

# 分析目标数量
print('\n\n分析目标数量:')
counts = [len(item['participant']) for item in data]
print(f'最少目标数: {min(counts)}')
print(f'最多目标数: {max(counts)}')

# 找出目标数量变化
print('\n目标数量变化:')
for i in range(len(counts)):
    if i == 0 or counts[i] != counts[i-1]:
        print(f'  帧{i}: {counts[i]} 个目标 (时间: {times[i]})')
