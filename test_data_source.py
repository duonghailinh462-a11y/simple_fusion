"""
数据源抽象层测试
验证FileRadarSource和StreamRadarSource的基本功能
"""

import sys
import json
from pathlib import Path

# 添加项目路径
sys.path.insert(0, str(Path(__file__).parent))

from radar.data_source import FileRadarSource, StreamRadarSource, IRadarSource


def test_file_source():
    """测试文件数据源"""
    print("\n" + "="*60)
    print("✅ 测试1: FileRadarSource基本接口")
    print("="*60)
    
    source = FileRadarSource()
    
    # 检查接口实现
    print("📝 检查接口实现...")
    assert isinstance(source, IRadarSource), "FileRadarSource应实现IRadarSource接口"
    print("✓ FileRadarSource实现了IRadarSource接口")
    
    # 检查必要方法
    required_methods = [
        'initialize', 'start', 'stop', 'get_next_frame', 
        'get_all_frames', 'is_ready', 'get_stats'
    ]
    
    for method in required_methods:
        assert hasattr(source, method), f"缺少方法: {method}"
        print(f"✓ {method}")
    
    print("\n📊 初始化参数检查...")
    # 测试初始化
    result = source.initialize(file_path="/nonexistent/file.jsonl")
    assert result == True, "初始化应该成功"
    print("✓ 初始化成功")
    
    print("\n📊 统计信息格式检查...")
    stats = source.get_stats()
    assert isinstance(stats, dict), "统计信息应该是字典"
    assert 'frames_loaded' in stats, "缺少frames_loaded字段"
    assert 'errors' in stats, "缺少errors字段"
    print(f"✓ 统计信息格式正确: {list(stats.keys())}")


def test_stream_source():
    """测试流数据源"""
    print("\n" + "="*60)
    print("✅ 测试2: StreamRadarSource基本接口")
    print("="*60)
    
    source = StreamRadarSource()
    
    # 检查接口实现
    print("📝 检查接口实现...")
    assert isinstance(source, IRadarSource), "StreamRadarSource应实现IRadarSource接口"
    print("✓ StreamRadarSource实现了IRadarSource接口")
    
    # 检查必要方法
    required_methods = [
        'initialize', 'start', 'stop', 'get_next_frame', 
        'get_all_frames', 'is_ready', 'get_stats'
    ]
    
    for method in required_methods:
        assert hasattr(source, method), f"缺少方法: {method}"
        print(f"✓ {method}")
    
    print("\n📊 初始化参数检查...")
    # 测试初始化
    result = source.initialize(host="127.0.0.1", port=5000, camera_id=1)
    assert result == True, "初始化应该成功"
    print("✓ 初始化成功")
    
    print("\n📊 统计信息格式检查...")
    stats = source.get_stats()
    assert isinstance(stats, dict), "统计信息应该是字典"
    assert 'frames_received' in stats, "缺少frames_received字段"
    assert 'bytes_received' in stats, "缺少bytes_received字段"
    print(f"✓ 统计信息格式正确: {list(stats.keys())}")


def test_data_frame():
    """测试RadarDataFrame数据结构"""
    print("\n" + "="*60)
    print("✅ 测试3: RadarDataFrame数据结构")
    print("="*60)
    
    from radar.data_source import RadarDataFrame
    
    print("📝 创建数据帧...")
    frame = RadarDataFrame(
        timestamp="2025-11-21 11:59:10.171",
        radar_objects=[],
        source="file",
        camera_id=1
    )
    
    assert frame.timestamp == "2025-11-21 11:59:10.171", "时间戳不匹配"
    assert frame.source == "file", "数据源不匹配"
    assert frame.camera_id == 1, "摄像头ID不匹配"
    
    print("✓ 数据帧创建成功")
    print(f"  - timestamp: {frame.timestamp}")
    print(f"  - source: {frame.source}")
    print(f"  - camera_id: {frame.camera_id}")
    print(f"  - radar_objects: {len(frame.radar_objects)} 个对象")


if __name__ == "__main__":
    print("\n" + "="*60)
    print("🚀 开始数据源抽象层测试")
    print("="*60)
    
    try:
        test_file_source()
        test_stream_source()
        test_data_frame()
        
        print("\n" + "="*60)
        print("✅ 所有测试通过！")
        print("="*60)
        print("""
📋 测试结果总结：
  ✓ FileRadarSource 接口完整
  ✓ StreamRadarSource 接口完整
  ✓ RadarDataFrame 数据结构完整
  ✓ 统计信息格式正确

🎯 第1步完成：数据源抽象层已创建
  - radar/data_source/base.py (IRadarSource接口)
  - radar/data_source/file_source.py (测试模式)
  - radar/data_source/stream_source.py (实际模式)
  - radar/data_source/__init__.py (导出模块)

📝 下一步: 创建数据源管理器 (radar_source_manager.py)
        """)
    
    except AssertionError as e:
        print(f"\n❌ 测试失败: {e}")
        sys.exit(1)
    except Exception as e:
        print(f"\n❌ 发生异常: {e}")
        import traceback
        traceback.print_exc()
        sys.exit(1)

