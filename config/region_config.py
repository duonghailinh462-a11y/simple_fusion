"""
车道配置文件 - 像素坐标系中的车道多边形定义
每个车道由多个点组成的多边形表示
"""

import numpy as np

# Camera 1 车道配置
CAMERA_1_LANES = {
    'lane_1': np.array([[1280, 457], [1280, 720], [1191, 720], [995, 455]], dtype=np.int32),
    'lane_2': np.array([[1191, 720], [995, 455], [771, 448], [854, 720]], dtype=np.int32),
    'lane_3': np.array([[771, 448], [854, 720], [484, 720], [535, 441]], dtype=np.int32),
    'lane_4': np.array([[484, 720], [535, 441], [309, 435], [115, 720]], dtype=np.int32),
    'lane_5': np.array([[309, 435], [115, 720], [0, 720], [0, 536], [109, 431]], dtype=np.int32),
}

# Camera 2 车道配置
CAMERA_2_LANES = {
    'lane_1': np.array([[962, 422], [1158, 720], [1280, 720], [1280, 419]], dtype=np.int32),
    'lane_2': np.array([[739, 432], [798, 720], [1158, 720], [739, 432]], dtype=np.int32),
    'lane_3': np.array([[516, 430], [421, 720], [798, 720], [739, 432]], dtype=np.int32),
    'lane_4': np.array([[308, 430], [92, 720], [421, 720], [516, 430]], dtype=np.int32),
    'lane_5': np.array([[104, 434], [0, 536], [0, 720], [92, 720], [308, 430]], dtype=np.int32),
}

# Camera 3 车道配置
CAMERA_3_LANES = {
    'lane_1': np.array([[541,473], [504,720], [1029,720], [989, 469]], dtype=np.int32),
    'lane_2': np.array([[194, 720], [331, 472], [539, 469], [503, 720]], dtype=np.int32),
}

# 统一的车道配置字典
LANE_CONFIG = {
    1: CAMERA_1_LANES,
    2: CAMERA_2_LANES,
    3: CAMERA_3_LANES,
}

def get_lane_for_point(camera_id: int, x: float, y: float) -> str:
    """
    根据像素坐标 (x, y) 判断该点属于哪个车道
    
    Args:
        camera_id: 摄像头ID (1, 2, 3)
        x: 像素x坐标
        y: 像素y坐标
    
    Returns:
        车道名称，如 'lane_1', 'lane_2' ... 如果不在任何车道内则返回 'unknown'
    """
    if camera_id not in LANE_CONFIG:
        return 'unknown'
    
    point = (int(x), int(y))
    lanes = LANE_CONFIG[camera_id]
    
    for lane_name, lane_polygon in lanes.items():
        # 使用 OpenCV 的点多边形测试
        import cv2
        result = cv2.pointPolygonTest(lane_polygon, point, False)
        if result >= 0:  # 点在多边形内或边上
            return lane_name
    
    return 'unknown'


# 如果需要直接打印车道配置，用于调试
if __name__ == '__main__':
    print("车道配置已加载:")
    for camera_id, lanes in LANE_CONFIG.items():
        print(f"\nCamera {camera_id}:")
        for lane_name, polygon in lanes.items():
            print(f"  {lane_name}: {polygon.tolist()}")

