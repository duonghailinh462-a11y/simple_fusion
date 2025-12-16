"""
雷达数据源抽象层单元测试

测试覆盖：
1. RadarObject 和 RadarFrame
2. JSONLRadarSource
3. StreamingRadarSource
4. MultiCameraRadarSource
5. CachedRadarSource
6. RadarSourceFactory
"""

import unittest
import json
import os
import tempfile
from typing import List

# 导入被测试的模块
from core.radar_source_abstraction import (
    RadarObject,
    RadarFrame,
    JSONLRadarSource,
    StreamingRadarSource,
    MultiCameraRadarSource,
    CachedRadarSource,
    RadarSourceFactory,
    parse_time,
)


# ==========================================
# 测试数据生成器
# ==========================================
class TestDataGenerator:
    """生成测试数据"""
    
    @staticmethod
    def create_test_jsonl_file(file_path: str, num_frames: int = 10) -> None:
        """创建测试 JSONL 文件"""
        with open(file_path, 'w', encoding='utf-8') as f:
            for i in range(num_frames):
                timestamp = f"2025-11-21 11:59:{10+i:02d}.{i:03d}"
                
                # 分配到不同的摄像头
                camera_id = (i % 3) + 1
                source_ip = {
                    1: '44.30.142.88',
                    2: '44.30.142.85',
                    3: '44.30.142.87',
                }[camera_id]
                
                frame = {
                    'time': timestamp,
                    'source_ip': source_ip,
                    'locusList': [
                        {
                            'id': f'radar_{i}_0',
                            'objType': 1,
                            'latitude': 23.530 + i * 0.001,
                            'longitude': 113.584 + i * 0.001,
                            'speed': 10.0 + i,
                            'azimuth': 45.0,
                            'lane': (i % 3) + 1,
                        },
                        {
                            'id': f'radar_{i}_1',
                            'objType': 1,
                            'latitude': 23.531 + i * 0.001,
                            'longitude': 113.585 + i * 0.001,
                            'speed': 15.0 + i,
                            'azimuth': 90.0,
                            'lane': ((i + 1) % 3) + 1,
                        }
                    ]
                }
                f.write(json.dumps(frame) + '\n')


# ==========================================
# 测试用例
# ==========================================
class TestRadarObject(unittest.TestCase):
    """测试 RadarObject"""
    
    def test_creation(self):
        """测试对象创建"""
        obj = RadarObject(
            radar_id='radar_1',
            latitude=23.530,
            longitude=113.584,
            speed=10.0,
            azimuth=45.0,
            lane='lane_1',
            timestamp_str='2025-11-21 11:59:10.171',
            source_ip='44.30.142.88'
        )
        
        self.assertEqual(obj.id, 'radar_1')
        self.assertEqual(obj.latitude, 23.530)
        self.assertEqual(obj.longitude, 113.584)
        self.assertEqual(obj.speed, 10.0)
        self.assertEqual(obj.azimuth, 45.0)
        self.assertEqual(obj.lane, 'lane_1')
        self.assertEqual(obj.source_ip, '44.30.142.88')
    
    def test_to_dict(self):
        """测试转换为字典"""
        obj = RadarObject(
            radar_id='radar_1',
            latitude=23.530,
            longitude=113.584,
        )
        
        d = obj.to_dict()
        self.assertIsInstance(d, dict)
        self.assertEqual(d['id'], 'radar_1')
        self.assertEqual(d['latitude'], 23.530)


class TestRadarFrame(unittest.TestCase):
    """测试 RadarFrame"""
    
    def setUp(self):
        """设置测试数据"""
        self.objects = [
            RadarObject('r1', 23.530, 113.584, lane='lane_1', source_ip='44.30.142.88'),
            RadarObject('r2', 23.531, 113.585, lane='lane_2', source_ip='44.30.142.85'),
        ]
    
    def test_creation(self):
        """测试帧创建"""
        frame = RadarFrame('2025-11-21 11:59:10.171', self.objects)
        
        self.assertEqual(frame.timestamp, '2025-11-21 11:59:10.171')
        self.assertEqual(len(frame.objects), 2)
    
    def test_camera_id_inference(self):
        """测试摄像头 ID 推断"""
        frame = RadarFrame('2025-11-21 11:59:10.171', self.objects)
        
        # 第一个对象的 source_ip 对应摄像头 1
        self.assertEqual(frame.camera_id, 1)
    
    def test_filter_by_lane(self):
        """测试按车道过滤"""
        frame = RadarFrame('2025-11-21 11:59:10.171', self.objects)
        
        lane_1_objects = frame.filter_by_lane('lane_1')
        self.assertEqual(len(lane_1_objects), 1)
        self.assertEqual(lane_1_objects[0].lane, 'lane_1')
    
    def test_get_object_count(self):
        """测试获取对象数量"""
        frame = RadarFrame('2025-11-21 11:59:10.171', self.objects)
        
        self.assertEqual(frame.get_object_count(), 2)


class TestJSONLRadarSource(unittest.TestCase):
    """测试 JSONLRadarSource"""
    
    def setUp(self):
        """创建临时测试文件"""
        self.test_dir = tempfile.mkdtemp()
        self.test_file = os.path.join(self.test_dir, 'radar_test.jsonl')
        TestDataGenerator.create_test_jsonl_file(self.test_file, num_frames=10)
    
    def tearDown(self):
        """清理临时文件"""
        if os.path.exists(self.test_file):
            os.remove(self.test_file)
        os.rmdir(self.test_dir)
    
    def test_initialization(self):
        """测试初始化"""
        source = JSONLRadarSource(self.test_file)
        self.assertTrue(source.initialize())
        source.close()
    
    def test_get_all_timestamps(self):
        """测试获取所有时间戳"""
        source = JSONLRadarSource(self.test_file)
        source.initialize()
        
        timestamps = source.get_all_timestamps()
        self.assertEqual(len(timestamps), 10)
        
        source.close()
    
    def test_get_frame(self):
        """测试获取特定帧"""
        source = JSONLRadarSource(self.test_file)
        source.initialize()
        
        timestamps = source.get_all_timestamps()
        frame = source.get_frame(timestamps[0])
        
        self.assertIsNotNone(frame)
        self.assertEqual(frame.timestamp, timestamps[0])
        self.assertGreater(frame.get_object_count(), 0)
        
        source.close()
    
    def test_stream_frames(self):
        """测试流式生成"""
        source = JSONLRadarSource(self.test_file)
        source.initialize()
        
        count = 0
        for frame in source.stream_frames():
            self.assertIsNotNone(frame)
            self.assertGreater(frame.get_object_count(), 0)
            count += 1
        
        self.assertEqual(count, 10)
        source.close()
    
    def test_get_stats(self):
        """测试获取统计信息"""
        source = JSONLRadarSource(self.test_file)
        source.initialize()
        
        stats = source.get_stats()
        self.assertIn('frames_loaded', stats)
        self.assertIn('objects_loaded', stats)
        self.assertEqual(stats['frames_loaded'], 10)
        self.assertGreater(stats['objects_loaded'], 0)
        
        source.close()


class TestStreamingRadarSource(unittest.TestCase):
    """测试 StreamingRadarSource"""
    
    def setUp(self):
        """创建临时测试文件"""
        self.test_dir = tempfile.mkdtemp()
        self.test_file = os.path.join(self.test_dir, 'radar_test.jsonl')
        TestDataGenerator.create_test_jsonl_file(self.test_file, num_frames=10)
    
    def tearDown(self):
        """清理临时文件"""
        if os.path.exists(self.test_file):
            os.remove(self.test_file)
        os.rmdir(self.test_dir)
    
    def test_initialization(self):
        """测试初始化"""
        source = StreamingRadarSource(self.test_file)
        self.assertTrue(source.initialize())
        source.close()
    
    def test_stream_frames(self):
        """测试流式生成"""
        source = StreamingRadarSource(self.test_file)
        source.initialize()
        
        count = 0
        for frame in source.stream_frames():
            self.assertIsNotNone(frame)
            count += 1
            if count >= 10:
                break
        
        self.assertGreater(count, 0)
        source.close()
    
    def test_buffer_size(self):
        """测试缓冲区大小"""
        source = StreamingRadarSource(self.test_file, buffer_size=5)
        source.initialize()
        
        # 消费所有帧
        for frame in source.stream_frames():
            pass
        
        # 缓冲区大小不应超过 buffer_size
        self.assertLessEqual(len(source.buffer), 5)
        source.close()


class TestMultiCameraRadarSource(unittest.TestCase):
    """测试 MultiCameraRadarSource"""
    
    def setUp(self):
        """创建临时测试文件"""
        self.test_dir = tempfile.mkdtemp()
        self.test_file = os.path.join(self.test_dir, 'radar_test.jsonl')
        TestDataGenerator.create_test_jsonl_file(self.test_file, num_frames=12)
    
    def tearDown(self):
        """清理临时文件"""
        if os.path.exists(self.test_file):
            os.remove(self.test_file)
        os.rmdir(self.test_dir)
    
    def test_initialization(self):
        """测试初始化"""
        base_source = JSONLRadarSource(self.test_file)
        source = MultiCameraRadarSource(base_source)
        self.assertTrue(source.initialize())
        source.close()
    
    def test_camera_detection(self):
        """测试摄像头检测"""
        base_source = JSONLRadarSource(self.test_file)
        source = MultiCameraRadarSource(base_source)
        source.initialize()
        
        stats = source.get_stats()
        self.assertEqual(stats['cameras_detected'], 3)
        
        source.close()
    
    def test_get_timestamps_by_camera(self):
        """测试按摄像头获取时间戳"""
        base_source = JSONLRadarSource(self.test_file)
        source = MultiCameraRadarSource(base_source)
        source.initialize()
        
        for camera_id in range(1, 4):
            timestamps = source.get_timestamps_by_camera(camera_id)
            self.assertGreater(len(timestamps), 0)
        
        source.close()
    
    def test_stream_frames_by_camera(self):
        """测试按摄像头流式生成"""
        base_source = JSONLRadarSource(self.test_file)
        source = MultiCameraRadarSource(base_source)
        source.initialize()
        
        for camera_id in range(1, 4):
            count = 0
            for frame in source.stream_frames_by_camera(camera_id):
                self.assertEqual(frame.camera_id, camera_id)
                count += 1
            
            self.assertGreater(count, 0)
        
        source.close()


class TestCachedRadarSource(unittest.TestCase):
    """测试 CachedRadarSource"""
    
    def setUp(self):
        """创建临时测试文件"""
        self.test_dir = tempfile.mkdtemp()
        self.test_file = os.path.join(self.test_dir, 'radar_test.jsonl')
        TestDataGenerator.create_test_jsonl_file(self.test_file, num_frames=10)
    
    def tearDown(self):
        """清理临时文件"""
        if os.path.exists(self.test_file):
            os.remove(self.test_file)
        os.rmdir(self.test_dir)
    
    def test_cache_hit(self):
        """测试缓存命中"""
        base_source = JSONLRadarSource(self.test_file)
        cached_source = CachedRadarSource(base_source, cache_size=100)
        cached_source.initialize()
        
        timestamps = cached_source.get_all_timestamps()
        
        # 第一次访问 - 缓存未命中
        frame1 = cached_source.get_frame(timestamps[0])
        self.assertEqual(cached_source.cache_misses, 1)
        
        # 第二次访问相同帧 - 缓存命中
        frame2 = cached_source.get_frame(timestamps[0])
        self.assertEqual(cached_source.cache_hits, 1)
        
        self.assertIs(frame1, frame2)
        cached_source.close()
    
    def test_cache_size_limit(self):
        """测试缓存大小限制"""
        base_source = JSONLRadarSource(self.test_file)
        cached_source = CachedRadarSource(base_source, cache_size=5)
        cached_source.initialize()
        
        timestamps = cached_source.get_all_timestamps()
        
        # 访问超过缓存大小的帧
        for ts in timestamps[:10]:
            cached_source.get_frame(ts)
        
        # 缓存大小不应超过限制
        self.assertLessEqual(len(cached_source.cache), 5)
        cached_source.close()


class TestRadarSourceFactory(unittest.TestCase):
    """测试 RadarSourceFactory"""
    
    def setUp(self):
        """创建临时测试文件"""
        self.test_dir = tempfile.mkdtemp()
        self.test_file = os.path.join(self.test_dir, 'radar_test.jsonl')
        TestDataGenerator.create_test_jsonl_file(self.test_file, num_frames=10)
    
    def tearDown(self):
        """清理临时文件"""
        if os.path.exists(self.test_file):
            os.remove(self.test_file)
        os.rmdir(self.test_dir)
    
    def test_create_jsonl_source(self):
        """测试创建 JSONL 源"""
        source = RadarSourceFactory.create_jsonl_source(self.test_file)
        self.assertIsNotNone(source)
        source.close()
    
    def test_create_streaming_source(self):
        """测试创建流式源"""
        source = RadarSourceFactory.create_streaming_source(self.test_file)
        self.assertIsNotNone(source)
        source.close()
    
    def test_create_multi_camera_source(self):
        """测试创建多摄像头源"""
        source = RadarSourceFactory.create_multi_camera_source(self.test_file)
        self.assertIsNotNone(source)
        source.close()
    
    def test_create_auto(self):
        """测试自动创建"""
        source = RadarSourceFactory.create_auto(self.test_file)
        self.assertIsNotNone(source)
        source.close()


class TestParseTime(unittest.TestCase):
    """测试时间戳解析"""
    
    def test_parse_with_milliseconds(self):
        """测试解析毫秒格式"""
        ts = parse_time('2025-11-21 11:59:10.171')
        self.assertGreater(ts, 0)
    
    def test_parse_without_milliseconds(self):
        """测试解析无毫秒格式"""
        ts = parse_time('2025-11-21 11:59:10')
        self.assertGreater(ts, 0)
    
    def test_parse_numeric(self):
        """测试解析数字格式"""
        ts = parse_time(1700555950.171)
        self.assertEqual(ts, 1700555950.171)
    
    def test_parse_none(self):
        """测试解析空值"""
        ts = parse_time(None)
        self.assertEqual(ts, 0.0)


# ==========================================
# 测试套件
# ==========================================
def suite():
    """创建测试套件"""
    test_suite = unittest.TestSuite()
    
    # 添加所有测试
    test_suite.addTests(unittest.TestLoader().loadTestsFromTestCase(TestRadarObject))
    test_suite.addTests(unittest.TestLoader().loadTestsFromTestCase(TestRadarFrame))
    test_suite.addTests(unittest.TestLoader().loadTestsFromTestCase(TestJSONLRadarSource))
    test_suite.addTests(unittest.TestLoader().loadTestsFromTestCase(TestStreamingRadarSource))
    test_suite.addTests(unittest.TestLoader().loadTestsFromTestCase(TestMultiCameraRadarSource))
    test_suite.addTests(unittest.TestLoader().loadTestsFromTestCase(TestCachedRadarSource))
    test_suite.addTests(unittest.TestLoader().loadTestsFromTestCase(TestRadarSourceFactory))
    test_suite.addTests(unittest.TestLoader().loadTestsFromTestCase(TestParseTime))
    
    return test_suite


if __name__ == '__main__':
    runner = unittest.TextTestRunner(verbosity=2)
    result = runner.run(suite())
    
    # 输出摘要
    print("\n" + "="*70)
    print("测试摘要")
    print("="*70)
    print(f"总测试数: {result.testsRun}")
    print(f"成功: {result.testsRun - len(result.failures) - len(result.errors)}")
    print(f"失败: {len(result.failures)}")
    print(f"错误: {len(result.errors)}")
    print("="*70)

