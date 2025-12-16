"""
雷达数据源抽象层集成示例

演示如何在实际系统中使用不同的数据源实现
"""

import sys
import os

# 添加项目路径
sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..'))

from core.radar_source_abstraction import (
    RadarSourceFactory,
    JSONLRadarSource,
    StreamingRadarSource,
    MultiCameraRadarSource,
    CachedRadarSource,
)


def example_1_basic_usage():
    """示例 1：基本用法 - 使用 JSONL 数据源"""
    print("\n" + "="*70)
    print("示例 1：基本用法 - JSONLRadarSource")
    print("="*70)
    
    source = RadarSourceFactory.create_jsonl_source('radar_data.jsonl')
    
    if not source.initialize():
        print("❌ 数据源初始化失败")
        return
    
    # 获取所有时间戳
    timestamps = source.get_all_timestamps()
    print(f"✅ 加载了 {len(timestamps)} 个时间戳")
    
    # 显示前5个时间戳的数据
    print("\n前5个时间戳的数据：")
    for ts in timestamps[:5]:
        frame = source.get_frame(ts)
        if frame:
            print(f"  {ts}: {frame.get_object_count()} 个目标")
    
    # 显示统计信息
    stats = source.get_stats()
    print(f"\n统计信息：")
    for key, value in stats.items():
        print(f"  {key}: {value}")
    
    source.close()


def example_2_streaming_source():
    """示例 2：流式数据源 - 处理大文件"""
    print("\n" + "="*70)
    print("示例 2：流式数据源 - StreamingRadarSource")
    print("="*70)
    
    source = StreamingRadarSource('radar_data.jsonl', buffer_size=50)
    
    if not source.initialize():
        print("❌ 数据源初始化失败")
        return
    
    # 流式处理数据
    frame_count = 0
    print("\n流式处理前10帧：")
    
    for frame in source.stream_frames():
        frame_count += 1
        if frame_count <= 10:
            print(f"  {frame.timestamp}: {frame.get_object_count()} 个目标")
        
        if frame_count >= 100:
            break
    
    print(f"\n✅ 处理了 {frame_count} 帧")
    
    stats = source.get_stats()
    print(f"统计信息：")
    for key, value in stats.items():
        print(f"  {key}: {value}")
    
    source.close()


def example_3_multi_camera():
    """示例 3：多摄像头数据源"""
    print("\n" + "="*70)
    print("示例 3：多摄像头数据源 - MultiCameraRadarSource")
    print("="*70)
    
    # 使用工厂创建多摄像头数据源
    source = RadarSourceFactory.create_multi_camera_source('radar_data.jsonl', use_streaming=False)
    
    if not source.initialize():
        print("❌ 数据源初始化失败")
        return
    
    stats = source.get_stats()
    print(f"\n✅ 检测到 {stats['cameras_detected']} 个摄像头")
    
    # 显示每个摄像头的统计信息
    for camera_id in range(1, 4):
        if camera_id in stats['frames_per_camera']:
            frames = stats['frames_per_camera'][camera_id]
            objects = stats['objects_per_camera'][camera_id]
            print(f"\n📹 摄像头 C{camera_id}:")
            print(f"    帧数: {frames}")
            print(f"    总目标数: {objects}")
            
            # 显示该摄像头的前3个时间戳
            timestamps = source.get_timestamps_by_camera(camera_id)
            print(f"    时间戳样本: {timestamps[:3]}")
    
    source.close()


def example_4_cached_source():
    """示例 4：缓存装饰器 - 优化频繁访问"""
    print("\n" + "="*70)
    print("示例 4：缓存装饰器 - CachedRadarSource")
    print("="*70)
    
    # 创建带缓存的数据源
    base_source = JSONLRadarSource('radar_data.jsonl')
    cached_source = CachedRadarSource(base_source, cache_size=100)
    
    if not cached_source.initialize():
        print("❌ 数据源初始化失败")
        return
    
    timestamps = cached_source.get_all_timestamps()
    print(f"✅ 加载了 {len(timestamps)} 个时间戳")
    
    # 重复访问同一个时间戳以测试缓存
    print("\n缓存测试 - 重复访问同一时间戳：")
    test_ts = timestamps[0]
    
    for i in range(5):
        frame = cached_source.get_frame(test_ts)
        print(f"  访问 #{i+1}: {test_ts}")
    
    # 显示缓存统计
    stats = cached_source.get_stats()
    print(f"\n缓存统计：")
    print(f"  缓存命中: {stats['cache_hits']}")
    print(f"  缓存未命中: {stats['cache_misses']}")
    print(f"  缓存大小: {stats['cache_size']}")
    if 'cache_hit_rate' in stats:
        print(f"  命中率: {stats['cache_hit_rate']:.1%}")
    
    cached_source.close()


def example_5_auto_factory():
    """示例 5：自动工厂 - 根据文件大小自动选择数据源"""
    print("\n" + "="*70)
    print("示例 5：自动工厂 - 根据文件大小自动选择")
    print("="*70)
    
    import os
    file_path = 'radar_data.jsonl'
    
    if not os.path.exists(file_path):
        print(f"❌ 文件不存在: {file_path}")
        return
    
    file_size = os.path.getsize(file_path)
    print(f"\n📊 文件大小: {file_size / 1024 / 1024:.1f}MB")
    
    # 自动选择合适的数据源
    source = RadarSourceFactory.create_auto(file_path)
    print(f"✅ 选择的数据源类型: {source.__class__.__name__}")
    
    if not source.initialize():
        print("❌ 数据源初始化失败")
        return
    
    # 如果是 JSONLRadarSource，显示时间戳
    if hasattr(source, 'timestamps'):
        print(f"✅ 加载了 {len(source.timestamps)} 个时间戳")
    
    source.close()


def example_6_comparison():
    """示例 6：性能对比 - JSONL vs Streaming"""
    print("\n" + "="*70)
    print("示例 6：性能对比 - JSONLRadarSource vs StreamingRadarSource")
    print("="*70)
    
    import time
    
    # 测试 JSONL 数据源
    print("\n[JSONL 数据源]")
    start_time = time.time()
    source1 = JSONLRadarSource('radar_data.jsonl')
    source1.initialize()
    time1 = time.time() - start_time
    stats1 = source1.get_stats()
    source1.close()
    
    print(f"  初始化时间: {time1:.3f}s")
    print(f"  帧数: {stats1['frames_loaded']}")
    print(f"  目标数: {stats1['objects_loaded']}")
    
    # 测试流式数据源
    print("\n[流式数据源]")
    start_time = time.time()
    source2 = StreamingRadarSource('radar_data.jsonl')
    source2.initialize()
    
    # 处理前100帧
    frame_count = 0
    for frame in source2.stream_frames():
        frame_count += 1
        if frame_count >= 100:
            break
    
    time2 = time.time() - start_time
    stats2 = source2.get_stats()
    source2.close()
    
    print(f"  处理100帧的时间: {time2:.3f}s")
    print(f"  处理的帧数: {stats2['frames_streamed']}")
    print(f"  处理的目标数: {stats2['objects_streamed']}")
    
    print(f"\n📊 结论: ")
    if time1 < time2:
        print(f"  JSONL 数据源更快 ({time1:.3f}s vs {time2:.3f}s)")
    else:
        print(f"  流式数据源更快 ({time2:.3f}s vs {time1:.3f}s)")


def run_all_examples():
    """运行所有示例"""
    print("\n" + "="*70)
    print("雷达数据源抽象层集成示例")
    print("="*70)
    
    examples = [
        ("基本用法", example_1_basic_usage),
        ("流式数据源", example_2_streaming_source),
        ("多摄像头", example_3_multi_camera),
        ("缓存装饰器", example_4_cached_source),
        ("自动工厂", example_5_auto_factory),
        ("性能对比", example_6_comparison),
    ]
    
    for name, func in examples:
        try:
            func()
        except FileNotFoundError as e:
            print(f"\n⚠️  {name} 示例跳过: {e}")
        except Exception as e:
            print(f"\n❌ {name} 示例出错: {e}")
    
    print("\n" + "="*70)
    print("所有示例执行完成")
    print("="*70 + "\n")


if __name__ == "__main__":
    run_all_examples()

