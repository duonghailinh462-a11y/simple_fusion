#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
像素坐标到经纬度的转换工具
使用项目中的摄像头矩阵和BEV转换矩阵
"""

import numpy as np
import math
import json
from typing import List, Tuple, Dict, Optional

# ==========================================
# 常量定义（来自 core/Basic.py）
# ==========================================

# 摄像头转换矩阵（像素 -> BEV）
CAMERA_MATRICES = {
    1: np.array([
        [3.57185777, -95.12052479, 4179.24844873],
        [3.46221359, -30.18092945, -4782.56623337],
        [0.00086667, -0.07779328, 1.00000000]
    ], dtype=np.float64),
    2: np.array([
        [-3.14205205, -15.41287574, -466.38259912],
        [-3.41382642, -24.02931190, 3191.87948399],
        [-0.00259235, -0.02359469, 1.00000000]
    ], dtype=np.float64),
    3: np.array([
        [2.30699835, -25.77644591, -1583.82133879],
        [-0.42448874, -13.71274357, -988.28445704],
        [0.00112695, -0.03632265, 1.00000000]
    ], dtype=np.float64),
}

# BEV到地理坐标的变换矩阵
BEV_TO_GEO_MATRIX = np.array([
    [1.32977514e-01, -1.04276598e-04, -1.50540001e+02],
    [1.45689395e-03, -1.33712569e-01, 7.61259809e+01],
    [-1.97872483e-06, -2.12579392e-05, 1.00000000e+00]
], dtype=np.float64)

# 地理坐标原点（参考点）
GEO_ORIGIN_LON = 113.584439426
GEO_ORIGIN_LAT = 23.530769118

# 地球相关常数
EARTH_RADIUS = 6378137.0  # 地球半径 (米)
METERS_PER_DEG_LAT = (math.pi / 180.0) * EARTH_RADIUS
METERS_PER_DEG_LON = (math.pi / 180.0) * EARTH_RADIUS * math.cos(math.radians(GEO_ORIGIN_LAT))

# ==========================================
# 转换函数
# ==========================================

def pixel_to_bev(H: np.ndarray, u: float, v: float) -> Optional[Tuple[float, float]]:
    """
    将像素坐标转换为BEV坐标
    
    Args:
        H: 转换矩阵（3x3）
        u: 像素x坐标
        v: 像素y坐标
    
    Returns:
        BEV坐标 (x, y)，失败返回None
    """
    p = np.array([u, v, 1.0])
    q = H @ p
    if abs(q[2]) < 1e-8:
        return None
    x, y = q[0] / q[2], q[1] / q[2]
    return (x, y)


def bev_to_geo(x_bev: float, y_bev: float) -> Optional[Tuple[float, float]]:
    """
    将BEV坐标转换为地理坐标（经纬度）
    
    步骤：
    1. BEV像素 -> 世界米制坐标
    2. 世界米制坐标 -> 地理坐标 (经纬度)
    
    Args:
        x_bev: BEV x坐标
        y_bev: BEV y坐标
    
    Returns:
        地理坐标 (lon, lat)，失败返回None
    """
    try:
        # 步骤1: BEV像素 -> 世界米制坐标
        p = np.array([x_bev, y_bev, 1.0])
        q = BEV_TO_GEO_MATRIX @ p
        q /= q[2]
        x_meters = q[0]  # 相对于原点的X偏移 (米)
        y_meters = q[1]  # 相对于原点的Y偏移 (米)
        
        # 步骤2: 世界米制坐标 -> 地理坐标 (经纬度)
        lon = (x_meters / METERS_PER_DEG_LON) + GEO_ORIGIN_LON
        lat = (y_meters / METERS_PER_DEG_LAT) + GEO_ORIGIN_LAT
        
        return lon, lat
    except Exception as e:
        print(f"  ❌ BEV转地理坐标失败: {e}")
        return None


def pixel_to_geo(camera_id: int, u: float, v: float) -> Optional[Tuple[float, float]]:
    """
    将像素坐标直接转换为地理坐标（经纬度）
    
    Args:
        camera_id: 摄像头ID (1, 2, 或 3)
        u: 像素x坐标
        v: 像素y坐标
    
    Returns:
        地理坐标 (lon, lat)，失败返回None
    """
    if camera_id not in CAMERA_MATRICES:
        print(f"❌ 不支持的摄像头ID: {camera_id}")
        return None
    
    # 步骤1: 像素 -> BEV
    H = CAMERA_MATRICES[camera_id]
    bev_coord = pixel_to_bev(H, u, v)
    if bev_coord is None:
        print(f"  ❌ 像素({u}, {v})转BEV失败")
        return None
    
    x_bev, y_bev = bev_coord
    
    # 步骤2: BEV -> 地理坐标
    geo_coord = bev_to_geo(x_bev, y_bev)
    return geo_coord


def convert_region(camera_id: int, region_pixels: List[Tuple[float, float]], 
                   region_name: str = "") -> Dict:
    """
    转换整个区域的所有顶点
    
    Args:
        camera_id: 摄像头ID
        region_pixels: 像素坐标列表 [(x1, y1), (x2, y2), ...]
        region_name: 区域名称（可选）
    
    Returns:
        包含转换结果的字典
    """
    result = {
        'region_name': region_name,
        'camera_id': camera_id,
        'pixel_coords': region_pixels,
        'geo_coords': [],
        'bev_coords': [],
        'details': []
    }
    
    H = CAMERA_MATRICES[camera_id]
    
    for idx, (u, v) in enumerate(region_pixels):
        # 转换到BEV
        bev_coord = pixel_to_bev(H, u, v)
        if bev_coord is None:
            result['bev_coords'].append(None)
            result['geo_coords'].append(None)
            result['details'].append({
                'index': idx,
                'pixel': (u, v),
                'status': 'failed at pixel->BEV conversion'
            })
            continue
        
        x_bev, y_bev = bev_coord
        
        # 转换到地理坐标
        geo_coord = bev_to_geo(x_bev, y_bev)
        
        result['bev_coords'].append(bev_coord)
        result['geo_coords'].append(geo_coord)
        
        if geo_coord:
            lon, lat = geo_coord
            result['details'].append({
                'index': idx,
                'pixel': (u, v),
                'bev': (round(x_bev, 2), round(y_bev, 2)),
                'geo': (round(lon, 10), round(lat, 10)),
                'status': 'success'
            })
        else:
            result['details'].append({
                'index': idx,
                'pixel': (u, v),
                'bev': (round(x_bev, 2), round(y_bev, 2)),
                'status': 'failed at BEV->geo conversion'
            })
    
    return result


# ==========================================
# 主程序
# ==========================================

if __name__ == "__main__":
    # 定义三个区域
    regions = {
        'camera_1_fusion_area': {
            'camera_id': 1,
            'pixels': [[110, 429], [0, 536], [0, 720], [1280, 720], [1280, 458]]
        },
        'camera_2_fusion_area': {
            'camera_id': 2,
            'pixels': [[0, 720], [1280, 720], [1280, 418], [109, 432]]
        },
        'camera_3_fusion_area': {
            'camera_id': 3,
            'pixels': [[328, 472], [186, 720], [1033, 720], [985, 468]]
        }
    }
    
    all_results = {}
    
    print("\n" + "="*80)
    print("像素坐标到经纬度转换结果")
    print("="*80)
    
    for region_name, region_info in regions.items():
        camera_id = region_info['camera_id']
        pixels = region_info['pixels']
        
        print(f"\n📍 {region_name} (摄像头 {camera_id})")
        print("-" * 80)
        
        result = convert_region(camera_id, pixels, region_name)
        all_results[region_name] = result
        
        # 打印详细结果
        print(f"像素坐标数量: {len(pixels)}")
        print(f"\n转换结果:")
        
        for detail in result['details']:
            idx = detail['index']
            pixel = detail['pixel']
            
            if detail['status'] == 'success':
                geo = detail['geo']
                bev = detail['bev']
                print(f"  [{idx}] 像素 {pixel} -> BEV {bev} -> 地理坐标 (lon: {geo[0]}, lat: {geo[1]})")
            else:
                print(f"  [{idx}] 像素 {pixel} -> ❌ {detail['status']}")
        
        # 输出GeoJSON格式的坐标
        geo_coords = [coord for coord in result['geo_coords'] if coord is not None]
        if geo_coords:
            print(f"\n📌 GeoJSON 格式的坐标 (可用于地图):")
            print(f"  坐标: {json.dumps(geo_coords, indent=4)}")
    
    # 生成汇总报告
    print("\n" + "="*80)
    print("汇总报告")
    print("="*80)
    
    for region_name, result in all_results.items():
        success_count = sum(1 for d in result['details'] if d['status'] == 'success')
        total_count = len(result['details'])
        print(f"\n{region_name}:")
        print(f"  摄像头: {result['camera_id']}")
        print(f"  转换成功: {success_count}/{total_count}")
        
        if success_count > 0:
            geo_coords = [d['geo'] for d in result['details'] if d['status'] == 'success']
            print(f"  经纬度坐标:")
            for idx, (lon, lat) in enumerate(geo_coords):
                print(f"    点{idx}: (lon={lon}, lat={lat})")
    
    # 保存结果到JSON文件
    output_file = '/zhw/no-frame-sync/pixel_to_geo_results.json'
    with open(output_file, 'w', encoding='utf-8') as f:
        # 转换结果便于JSON序列化
        json_results = {}
        for region_name, result in all_results.items():
            json_results[region_name] = {
                'camera_id': result['camera_id'],
                'pixel_coords': result['pixel_coords'],
                'geo_coords': result['geo_coords'],
                'details': result['details']
            }
        json.dump(json_results, f, indent=2, ensure_ascii=False)
    
    print(f"\n✅ 结果已保存到: {output_file}")

