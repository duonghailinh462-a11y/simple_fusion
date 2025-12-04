import numpy as np
from collections import deque
import os
import os.path as osp
import copy
import time
import torch
import torch.nn.functional as F

from .kalman_filter import KalmanFilter
from . import matching
from .basetrack import BaseTrack, TrackState
from .byte_tracker import STrack as BaseSTrack, joint_stracks, sub_stracks, remove_duplicate_stracks


class OptimizedSTrack(BaseSTrack):
    """扩展的STrack类，支持预测帧标记"""
    
    def __init__(self, tlwh, score):
        super().__init__(tlwh, score)
        self.prediction_frame = False  # 标记是否为预测帧
        self.last_update_frame = 0    # 最后一次更新的帧号
        self.prediction_count = 0     # 连续预测帧计数


class OptimizedBYTETracker(object):
    """
    优化版ByteTracker - 实现"一帧跟踪一帧只预测"的优化策略
    
    优化策略：
    - 偶数帧：执行完整跟踪（预测+匹配+更新）
    - 奇数帧：只执行预测，跳过匹配和更新
    """
    
    def __init__(self, args, frame_rate=30):
        self.tracked_stracks = []  # type: list[STrack]
        self.lost_stracks = []  # type: list[STrack]
        self.removed_stracks = []  # type: list[STrack]

        self.frame_id = 0
        self.args = args
        self.det_thresh = args.track_thresh + 0.1
        self.buffer_size = int(frame_rate / 30.0 * args.track_buffer)
        self.max_time_lost = self.buffer_size
        self.kalman_filter = KalmanFilter()
        
        # 优化相关参数
        self.alternating_mode = True  # 是否启用交替模式
        self.prediction_only_frames = set()  # 记录只预测的帧
        self.tracking_frames = set()  # 记录完整跟踪的帧
        
        # 性能统计
        self.stats = {
            'total_frames': 0,
            'tracking_frames': 0,
            'prediction_only_frames': 0,
            'computation_saved': 0.0,
            'total_tracking_time': 0.0,  # 累积跟踪时间 (秒)
            'total_prediction_time': 0.0,  # 累积预测时间 (秒)
            'avg_tracking_time': 0.0,     # 平均跟踪时间
            'avg_prediction_time': 0.0    # 平均预测时间
        }

    def update(self, output_results, img_info, img_size):
        self.frame_id += 1
        self.stats['total_frames'] += 1
        
        # 判断当前帧是否只执行预测
        is_prediction_only = (self.alternating_mode and 
                            self.frame_id % 2 == 1 and 
                            len(self.tracked_stracks) > 0)
        
        start_time = time.time()
        
        if is_prediction_only:
            result = self._prediction_only_update()
            elapsed = time.time() - start_time
            self.stats['total_prediction_time'] += elapsed
        else:
            result = self._full_tracking_update(output_results, img_info, img_size)
            elapsed = time.time() - start_time
            self.stats['total_tracking_time'] += elapsed
        
        # 更新平均时间
        if self.stats['tracking_frames'] > 0:
            self.stats['avg_tracking_time'] = self.stats['total_tracking_time'] / self.stats['tracking_frames']
        if self.stats['prediction_only_frames'] > 0:
            self.stats['avg_prediction_time'] = self.stats['total_prediction_time'] / self.stats['prediction_only_frames']
        
        return result

    def _prediction_only_update(self):
        """只执行预测的更新 - 奇数帧"""
        self.stats['prediction_only_frames'] += 1
        self.prediction_only_frames.add(self.frame_id)
        
        # 只对所有track进行预测
        all_tracks = joint_stracks(self.tracked_stracks, self.lost_stracks)
        if len(all_tracks) > 0:
            OptimizedSTrack.multi_predict(all_tracks)
        
        # 更新track状态（标记为预测状态）
        for track in self.tracked_stracks:
            track.frame_id = self.frame_id
            # 保持Tracked状态，但标记为预测帧
            track.prediction_frame = True
        
        # 处理超时的lost tracks
        removed_stracks = []
        for track in self.lost_stracks:
            if self.frame_id - track.end_frame > self.max_time_lost:
                track.mark_removed()
                removed_stracks.append(track)
        
        # 更新track列表
        self.lost_stracks = [t for t in self.lost_stracks if t.state != TrackState.Removed]
        self.removed_stracks.extend(removed_stracks)
        
        # 返回当前活跃的tracks（基于预测位置）
        output_stracks = [track for track in self.tracked_stracks if track.is_activated]
        
        # 计算节省的计算量
        self.stats['computation_saved'] += self._estimate_computation_saved()
        
        return output_stracks

    def _full_tracking_update(self, output_results, img_info, img_size):
        """完整跟踪更新 - 偶数帧"""
        self.stats['tracking_frames'] += 1
        self.tracking_frames.add(self.frame_id)
        
        activated_starcks = []
        refind_stracks = []
        lost_stracks = []
        removed_stracks = []

        # 处理检测结果
        if output_results.shape[1] == 5:
            scores = output_results[:, 4]
            bboxes = output_results[:, :4]
        else:
            if hasattr(output_results, 'cpu'):
                output_results = output_results.cpu().numpy()
            scores = output_results[:, 4] * output_results[:, 5]
            bboxes = output_results[:, :4]

        img_h, img_w = img_info[0], img_info[1]
        scale = min(img_size[0] / float(img_h), img_size[1] / float(img_w))
        bboxes /= scale

        # 分离高置信度和低置信度检测
        remain_inds = scores > self.args.track_thresh
        inds_low = scores > 0.1
        inds_high = scores < self.args.track_thresh
        inds_second = np.logical_and(inds_low, inds_high)
        
        dets_second = bboxes[inds_second]
        dets = bboxes[remain_inds]
        scores_keep = scores[remain_inds]
        scores_second = scores[inds_second]

        if len(dets) > 0:
            detections = [OptimizedSTrack(OptimizedSTrack.tlbr_to_tlwh(tlbr), s) for
                          (tlbr, s) in zip(dets, scores_keep)]
        else:
            detections = []

        # 分离已确认和未确认的tracks
        unconfirmed = []
        tracked_stracks = []
        for track in self.tracked_stracks:
            if not track.is_activated:
                unconfirmed.append(track)
            else:
                tracked_stracks.append(track)
                # 重置预测帧标记
                track.prediction_frame = False

        # 第一次关联 - 高置信度检测
        strack_pool = joint_stracks(tracked_stracks, self.lost_stracks)
        OptimizedSTrack.multi_predict(strack_pool)
        
        dists = matching.iou_distance(strack_pool, detections)
        if not self.args.mot20:
            dists = matching.fuse_score(dists, detections)
        matches, u_track, u_detection = matching.linear_assignment(dists, thresh=self.args.match_thresh)

        for itracked, idet in matches:
            track = strack_pool[itracked]
            det = detections[idet]
            if track.state == TrackState.Tracked:
                track.update(detections[idet], self.frame_id)
                activated_starcks.append(track)
            else:
                track.re_activate(det, self.frame_id, new_id=False)
                refind_stracks.append(track)

        # 第二次关联 - 低置信度检测
        if len(dets_second) > 0:
            detections_second = [OptimizedSTrack(OptimizedSTrack.tlbr_to_tlwh(tlbr), s) for
                          (tlbr, s) in zip(dets_second, scores_second)]
        else:
            detections_second = []
            
        r_tracked_stracks = [strack_pool[i] for i in u_track if strack_pool[i].state == TrackState.Tracked]
        dists = matching.iou_distance(r_tracked_stracks, detections_second)
        matches, u_track, u_detection_second = matching.linear_assignment(dists, thresh=0.5)
        
        for itracked, idet in matches:
            track = r_tracked_stracks[itracked]
            det = detections_second[idet]
            if track.state == TrackState.Tracked:
                track.update(det, self.frame_id)
                activated_starcks.append(track)
            else:
                track.re_activate(det, self.frame_id, new_id=False)
                refind_stracks.append(track)

        # 处理未匹配的tracks
        for it in u_track:
            track = r_tracked_stracks[it]
            if not track.state == TrackState.Lost:
                track.mark_lost()
                lost_stracks.append(track)

        # 处理未确认的tracks
        detections = [detections[i] for i in u_detection]
        dists = matching.iou_distance(unconfirmed, detections)
        if not self.args.mot20:
            dists = matching.fuse_score(dists, detections)
        matches, u_unconfirmed, u_detection = matching.linear_assignment(dists, thresh=0.7)
        
        for itracked, idet in matches:
            unconfirmed[itracked].update(detections[idet], self.frame_id)
            activated_starcks.append(unconfirmed[itracked])
            
        for it in u_unconfirmed:
            track = unconfirmed[it]
            track.mark_removed()
            removed_stracks.append(track)

        # 初始化新tracks
        for inew in u_detection:
            track = detections[inew]
            if track.score < self.det_thresh:
                continue
            track.activate(self.kalman_filter, self.frame_id)
            activated_starcks.append(track)

        # 更新状态
        for track in self.lost_stracks:
            if self.frame_id - track.end_frame > self.max_time_lost:
                track.mark_removed()
                removed_stracks.append(track)

        # 更新track列表
        self.tracked_stracks = [t for t in self.tracked_stracks if t.state == TrackState.Tracked]
        self.tracked_stracks = joint_stracks(self.tracked_stracks, activated_starcks)
        self.tracked_stracks = joint_stracks(self.tracked_stracks, refind_stracks)
        self.lost_stracks = sub_stracks(self.lost_stracks, self.tracked_stracks)
        self.lost_stracks.extend(lost_stracks)
        self.lost_stracks = sub_stracks(self.lost_stracks, self.removed_stracks)
        self.removed_stracks.extend(removed_stracks)
        self.tracked_stracks, self.lost_stracks = remove_duplicate_stracks(self.tracked_stracks, self.lost_stracks)

        output_stracks = [track for track in self.tracked_stracks if track.is_activated]
        return output_stracks

    def _estimate_computation_saved(self):
        """估算节省的计算量"""
        n_tracks = len(self.tracked_stracks) + len(self.lost_stracks)
        n_detections = 10  # 假设平均每帧10个检测
        
        # 估算节省的IoU计算量
        iou_computations_saved = n_tracks * n_detections
        # 估算节省的匈牙利算法计算量
        hungarian_computations_saved = n_tracks * n_detections * 0.1  # 匈牙利算法复杂度
        
        return iou_computations_saved + hungarian_computations_saved

    def get_performance_stats(self):
        """获取性能统计信息"""
        if self.stats['total_frames'] == 0:
            return self.stats
            
        self.stats['tracking_ratio'] = self.stats['tracking_frames'] / self.stats['total_frames']
        self.stats['prediction_only_ratio'] = self.stats['prediction_only_frames'] / self.stats['total_frames']
        self.stats['computation_saved_ratio'] = (self.stats['computation_saved'] / 
                                               (self.stats['total_frames'] * 100))  # 归一化
        
        return self.stats

    def set_alternating_mode(self, enabled: bool):
        """设置是否启用交替模式"""
        self.alternating_mode = enabled

    def reset_stats(self):
        """重置统计信息"""
        self.stats = {
            'total_frames': 0,
            'tracking_frames': 0,
            'prediction_only_frames': 0,
            'computation_saved': 0.0,
            'total_tracking_time': 0.0,
            'total_prediction_time': 0.0,
            'avg_tracking_time': 0.0,
            'avg_prediction_time': 0.0
        }
        self.prediction_only_frames.clear()
        self.tracking_frames.clear()
