#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
验证三路融合输出是否包含三个摄像头的数据
"""

import json
import sys
from collections import Counter

def verify_triple_fusion(json_file):
    """验证JSON文件中是否包含三路融合数据"""
    
    print(f"正在验证文件: {json_file}")
    
    try:
        with open(json_file, 'r', encoding='utf-8') as f:
            data = json.load(f)
    except Exception as e:
        print(f"❌ 读取JSON文件失败: {e}")
        return False
    
    if not isinstance(data, list):
        print(f"❌ JSON数据不是列表格式")
        return False
    
    print(f"📊 总共有 {len(data)} 个 reportTime 条目")
    
    # 统计每个reportTime中的摄像头数量
    triple_count = 0  # 包含3个摄像头的reportTime数
    dual_count = 0    # 包含2个摄像头的reportTime数
    single_count = 0  # 只包含1个摄像头的reportTime数
    
    # 统计每个摄像头出现的次数
    camera_counts = Counter()
    participants_per_report = []
    
    for i, report in enumerate(data):
        if 'participant' not in report:
            print(f"❌ 第 {i} 条记录缺少 'participant' 字段")
            continue
        
        participants = report['participant']
        num_participants = len(participants)
        
        if num_participants == 0:
            continue
        
        participants_per_report.append(num_participants)
        
        # 统计该reportTime中的摄像头
        cameras_in_report = set()
        for participant in participants:
            if 'cameraid' in participant:
                cameras_in_report.add(participant['cameraid'])
                camera_counts[participant['cameraid']] += 1
        
        num_cameras = len(cameras_in_report)
        
        if num_cameras == 3:
            triple_count += 1
            if triple_count <= 5:  # 只显示前5个三路结果
                print(f"✅ 第 {i} 条：包含3个摄像头 {sorted(cameras_in_report)}")
                for p in participants:
                    print(f"   - C{p['cameraid']}: track_id={p['track_id']}, "
                          f"type={p['type']}, radar_id={p['radar_id']}")
        elif num_cameras == 2:
            dual_count += 1
        else:
            single_count += 1
    
    print("\n" + "="*60)
    print("🔍 验证结果总结")
    print("="*60)
    print(f"✅ 包含3个摄像头的 reportTime: {triple_count}")
    print(f"⚠️  包含2个摄像头的 reportTime: {dual_count}")
    print(f"❌ 只包含1个摄像头的 reportTime: {single_count}")
    
    print("\n📊 每个摄像头的参与者数量:")
    for camera_id in sorted(camera_counts.keys()):
        print(f"   Camera {camera_id}: {camera_counts[camera_id]} 次出现")
    
    # 计算参与者数量的统计信息
    if participants_per_report:
        avg_participants = sum(participants_per_report) / len(participants_per_report)
        max_participants = max(participants_per_report)
        min_participants = min(participants_per_report)
        print(f"\n📈 参与者数量统计:")
        print(f"   平均: {avg_participants:.2f}")
        print(f"   最大: {max_participants}")
        print(f"   最小: {min_participants}")
    
    # 判断是否达到三路融合目标
    if triple_count > 0:
        triple_percentage = (triple_count / (triple_count + dual_count + single_count)) * 100 if (triple_count + dual_count + single_count) > 0 else 0
        print(f"\n🎯 三路融合成功率: {triple_percentage:.1f}% ({triple_count}/{triple_count + dual_count + single_count})")
        
        if triple_percentage >= 50:
            print("✅ 三路融合效果良好！")
            return True
        elif triple_percentage > 0:
            print("⚠️  部分三路融合成功，但比例较低")
            return True
        else:
            print("❌ 没有三路融合")
            return False
    else:
        print("❌ 没有检测到三路融合的数据！")
        return False

if __name__ == "__main__":
    json_file = "output_fusion_refactored.json"
    if len(sys.argv) > 1:
        json_file = sys.argv[1]
    
    success = verify_triple_fusion(json_file)
    sys.exit(0 if success else 1)


