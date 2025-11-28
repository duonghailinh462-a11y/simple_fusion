"""
测试雷达融合模块
"""

import json
from RadarVisionFusion import RadarVisionFusionProcessor, RadarDataLoader, OutputObject


def test_radar_fusion():
    """测试雷达融合功能"""
    
    print("=" * 60)
    print("测试雷达融合模块")
    print("=" * 60)
    
    # 1. 创建融合处理器
    print("\n1. 创建融合处理器...")
    fusion_area_geo = [
        [113.583894894, 23.530394880],
        [113.584462681, 23.530850485],
        [113.584032327, 23.530886446],
        [113.583922645, 23.530898319]
    ]
    
    processor = RadarVisionFusionProcessor(
        fusion_area_geo=fusion_area_geo,
        lat_offset=-0.00000165,
        lon_offset=0.0000450
    )
    print("✅ 融合处理器创建成功")
    
    # 2. 加载雷达数据
    print("\n2. 加载雷达数据...")
    radar_data_path = 'c:/Users/zhenghuiwen1/Desktop/project_simple/radar_vision/radar_data_85_aligned.jsonl'
    
    loader = RadarDataLoader(radar_data_path)
    if loader.load():
        print(f"✅ 雷达数据加载成功: {len(loader.get_all_timestamps())} 帧")
        
        # 将雷达数据添加到处理器
        for ts in loader.get_all_timestamps():
            radar_objs = loader.get_radar_data(ts)
            processor.add_radar_data(ts, radar_objs)
        
        print(f"✅ 雷达数据已添加到处理器")
    else:
        print("❌ 雷达数据加载失败")
        return
    
    # 3. 创建测试视觉目标
    print("\n3. 创建测试视觉目标...")
    vision_timestamp = loader.get_all_timestamps()[0]  # 使用第一个雷达时间戳
    
    vision_objects = [
        OutputObject(
            timestamp="2025-11-21 11:17:58.064",
            cameraid=1,
            type_name="car",
            confidence=0.85,
            track_id=1,
            lon=113.584143,
            lat=23.530363
        ),
        OutputObject(
            timestamp="2025-11-21 11:17:58.064",
            cameraid=2,
            type_name="truck",
            confidence=0.90,
            track_id=2,
            lon=113.584200,
            lat=23.530400
        ),
    ]
    
    print(f"✅ 创建了 {len(vision_objects)} 个测试视觉目标")
    
    # 4. 执行融合
    print("\n4. 执行雷达融合...")
    updated_objects = processor.process_frame(vision_timestamp, vision_objects)
    
    print(f"✅ 融合完成")
    
    # 5. 输出结果
    print("\n5. 融合结果:")
    print("-" * 60)
    for obj in updated_objects:
        result = obj.to_dict()
        print(json.dumps(result, indent=2, ensure_ascii=False))
        print("-" * 60)
    
    # 6. 统计信息
    print("\n6. 统计信息:")
    stats = processor.get_stats()
    print(f"  雷达目标处理数: {stats['radar_objects_processed']}")
    print(f"  视觉目标处理数: {stats['vision_objects_processed']}")
    print(f"  成功匹配数: {stats['successful_matches']}")
    print(f"  失败匹配数: {stats['failed_matches']}")
    
    matched_count = sum(1 for obj in updated_objects if obj.radar_id is not None)
    print(f"  匹配率: {matched_count}/{len(updated_objects)} ({matched_count/len(updated_objects)*100:.1f}%)")
    
    print("\n" + "=" * 60)
    print("测试完成!")
    print("=" * 60)


if __name__ == "__main__":
    test_radar_fusion()
