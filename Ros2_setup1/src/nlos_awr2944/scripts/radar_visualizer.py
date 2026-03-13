#!/usr/bin/env python3
"""
Radar Data Visualizer & Converter
==================================
rosbag 데이터를 CSV로 변환하고 시각화하는 통합 스크립트

사용법:
  python3 radar_visualizer.py                    # 최신 rosbag 자동 변환 + 시각화
  python3 radar_visualizer.py -d /path/to/rosbag # 특정 rosbag 변환 + 시각화
  python3 radar_visualizer.py --convert-only     # 변환만 (시각화 없음)
  python3 radar_visualizer.py --visualize-only   # 기존 CSV 시각화만

출력 구조:
  rosbag_folder/
  └── transformed_rosbag_YYYYMMDD_HHMMSS/
      ├── pcd/
      │   ├── pointcloud.csv
      │   ├── frame_0001_pointcloud.png
      │   ├── frame_0001_heatmap.png
      │   └── ...
      └── others/
          ├── range_profile.csv
          ├── noise_profile.csv
          └── azimuth_heatmap.csv
"""

import os
import sys
import argparse
import math
from datetime import datetime

# ROS2 관련
try:
    from rosbags.rosbag2 import Reader
    from rosbags.serde import deserialize_cdr
    HAS_ROSBAGS = True
except ImportError:
    HAS_ROSBAGS = False
    print("⚠️ rosbags 라이브러리 없음. 설치: pip install rosbags")

# 시각화 관련
import pandas as pd
import numpy as np
import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation
from matplotlib.widgets import Slider, Button
import struct
import csv

# ==========================================
# 설정
# ==========================================
MAX_RANGE_METERS = 10.0
VIRTUAL_ANTENNAS = 12

# 기본 경로
DEFAULT_ROSBAG_DIR = os.path.expanduser("~/Nlos/Ros2_setup/src/nlos_awr2944/rosbag")


def find_latest_rosbag(base_path):
    """가장 최근 rosbag 폴더 찾기"""
    base_path = os.path.expanduser(base_path)
    if not os.path.exists(base_path):
        print(f"❌ 경로 없음: {base_path}")
        return None
    
    # rosbag 폴더들 찾기 (metadata.yaml 있는 폴더)
    rosbag_dirs = []
    for item in os.listdir(base_path):
        item_path = os.path.join(base_path, item)
        if os.path.isdir(item_path):
            # metadata.yaml이 있으면 rosbag 폴더
            if os.path.exists(os.path.join(item_path, 'metadata.yaml')):
                rosbag_dirs.append(item_path)
    
    if not rosbag_dirs:
        print(f"❌ rosbag 폴더를 찾을 수 없음: {base_path}")
        return None
    
    # 가장 최근 폴더 반환
    return max(rosbag_dirs, key=os.path.getmtime)


def find_latest_transformed(base_path):
    """가장 최근 transformed_rosbag 폴더 찾기"""
    base_path = os.path.expanduser(base_path)
    if not os.path.exists(base_path):
        return None
    
    transformed_dirs = []
    for item in os.listdir(base_path):
        item_path = os.path.join(base_path, item)
        if os.path.isdir(item_path) and 'transformed_rosbag' in item:
            transformed_dirs.append(item_path)
    
    if not transformed_dirs:
        return None
    
    return max(transformed_dirs, key=os.path.getmtime)


# ==========================================
# Rosbag -> CSV 변환기
# ==========================================
class RosbagConverter:
    def __init__(self, rosbag_path):
        self.rosbag_path = rosbag_path
        self.output_dir = None
        self.pcd_dir = None
        self.others_dir = None
        
        # 데이터 저장용
        self.pointcloud_data = []
        self.range_profile_data = []
        self.noise_profile_data = []
        self.heatmap_data = []
        self.first_timestamp = None
        
    def setup_output_dirs(self):
        """출력 디렉토리 생성"""
        parent_dir = os.path.dirname(self.rosbag_path)
        timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
        self.output_dir = os.path.join(parent_dir, f"transformed_rosbag_{timestamp}")
        self.pcd_dir = os.path.join(self.output_dir, "pcd")
        self.others_dir = os.path.join(self.output_dir, "others")
        
        os.makedirs(self.pcd_dir, exist_ok=True)
        os.makedirs(self.others_dir, exist_ok=True)
        
        print(f"📁 출력 폴더: {self.output_dir}")
        
    def convert(self):
        """rosbag을 CSV로 변환"""
        if not HAS_ROSBAGS:
            print("❌ rosbags 라이브러리가 필요합니다: pip install rosbags")
            return None
        
        self.setup_output_dirs()
        
        print(f"📂 Rosbag 읽기: {self.rosbag_path}")
        
        try:
            with Reader(self.rosbag_path) as reader:
                # 토픽 정보 출력
                print(f"   토픽 목록:")
                for connection in reader.connections:
                    print(f"      - {connection.topic}: {connection.msgtype}")
                
                # 프레임 카운터
                frame_counters = {
                    'points': 0,
                    'range': 0,
                    'noise': 0,
                    'heatmap': 0
                }
                
                # 메시지 처리
                for connection, timestamp, rawdata in reader.messages():
                    topic = connection.topic
                    
                    # 첫 타임스탬프 저장
                    if self.first_timestamp is None:
                        self.first_timestamp = timestamp
                    
                    try:
                        msg = deserialize_cdr(rawdata, connection.msgtype)
                        
                        if 'points' in topic:
                            self.process_pointcloud(msg, timestamp, frame_counters['points'])
                            frame_counters['points'] += 1
                        elif 'range_profile' in topic:
                            self.process_range_profile(msg, timestamp, frame_counters['range'])
                            frame_counters['range'] += 1
                        elif 'noise_profile' in topic:
                            self.process_noise_profile(msg, timestamp, frame_counters['noise'])
                            frame_counters['noise'] += 1
                        elif 'heatmap' in topic:
                            self.process_heatmap(msg, timestamp, frame_counters['heatmap'])
                            frame_counters['heatmap'] += 1
                    except Exception as e:
                        print(f"   ⚠️ 메시지 처리 오류 ({topic}): {e}")
                        continue
                        
        except Exception as e:
            print(f"❌ Rosbag 읽기 오류: {e}")
            import traceback
            traceback.print_exc()
            return None
        
        # CSV 저장
        self.save_csvs()
        
        print(f"✅ 변환 완료!")
        print(f"   - PointCloud: {len(self.pointcloud_data)} points")
        print(f"   - Range Profile: {len(self.range_profile_data)} frames")
        print(f"   - Noise Profile: {len(self.noise_profile_data)} frames")
        print(f"   - Heatmap: {len(self.heatmap_data)} frames")
        
        return self.output_dir
    
    def process_pointcloud(self, msg, timestamp, frame_num):
        """PointCloud2 메시지 처리"""
        # 타임스탬프 (나노초 -> 초)
        ts_sec = timestamp / 1e9
        rel_time = (timestamp - self.first_timestamp) / 1e9
        
        # 포인트 개수
        point_step = msg.point_step
        num_points = msg.width * msg.height
        data = bytes(msg.data)
        
        if num_points == 0:
            return
        
        # 필드 오프셋 찾기
        field_offsets = {}
        for field in msg.fields:
            field_offsets[field.name] = field.offset
        
        for i in range(num_points):
            offset = i * point_step
            
            try:
                # 기본 필드 읽기
                x = struct.unpack_from('<f', data, offset + field_offsets.get('x', 0))[0]
                y = struct.unpack_from('<f', data, offset + field_offsets.get('y', 4))[0]
                z = struct.unpack_from('<f', data, offset + field_offsets.get('z', 8))[0]
                v = struct.unpack_from('<f', data, offset + field_offsets.get('velocity', 12))[0] if 'velocity' in field_offsets else 0
                intensity = struct.unpack_from('<f', data, offset + field_offsets.get('intensity', 16))[0] if 'intensity' in field_offsets else 0
                
                # RCS 값 (intensity가 이미 RCS로 계산되어 있음)
                rcs = intensity
                
                self.pointcloud_data.append({
                    'Frame': frame_num,
                    'Rel_Time(s)': f"{rel_time:.3f}",
                    'Unix_Time(s)': f"{ts_sec:.3f}",
                    'X(m)': f"{x:.4f}",
                    'Y(m)': f"{y:.4f}",
                    'Z(m)': f"{z:.4f}",
                    'V(m/s)': f"{v:.4f}",
                    'RCS(dB)': f"{rcs:.2f}"
                })
            except Exception:
                continue
    
    def process_range_profile(self, msg, timestamp, frame_num):
        """Range Profile 메시지 처리"""
        rel_time = (timestamp - self.first_timestamp) / 1e9
        
        data = list(msg.data)
        self.range_profile_data.append({
            'Frame': frame_num,
            'Rel_Time(s)': f"{rel_time:.3f}",
            'data': data
        })
    
    def process_noise_profile(self, msg, timestamp, frame_num):
        """Noise Profile 메시지 처리"""
        rel_time = (timestamp - self.first_timestamp) / 1e9
        
        data = list(msg.data)
        self.noise_profile_data.append({
            'Frame': frame_num,
            'Rel_Time(s)': f"{rel_time:.3f}",
            'data': data
        })
    
    def process_heatmap(self, msg, timestamp, frame_num):
        """Heatmap (Image) 메시지 처리"""
        rel_time = (timestamp - self.first_timestamp) / 1e9
        
        width = msg.width
        height = msg.height
        data = list(msg.data)
        
        self.heatmap_data.append({
            'Frame': frame_num,
            'Rel_Time(s)': f"{rel_time:.3f}",
            'Width': width,
            'Height': height,
            'data': data
        })
    
    def save_csvs(self):
        """CSV 파일 저장"""
        # PointCloud CSV
        if self.pointcloud_data:
            pc_path = os.path.join(self.pcd_dir, "pointcloud.csv")
            with open(pc_path, 'w', newline='') as f:
                writer = csv.DictWriter(f, fieldnames=['Frame', 'Rel_Time(s)', 'Unix_Time(s)', 
                                                        'X(m)', 'Y(m)', 'Z(m)', 'V(m/s)', 'RCS(dB)'])
                writer.writeheader()
                writer.writerows(self.pointcloud_data)
            print(f"   💾 저장: {pc_path}")
        
        # Range Profile CSV
        if self.range_profile_data:
            rp_path = os.path.join(self.others_dir, "range_profile.csv")
            with open(rp_path, 'w', newline='') as f:
                max_bins = max(len(d['data']) for d in self.range_profile_data) if self.range_profile_data else 0
                header = ['Frame', 'Rel_Time(s)'] + [f'bin_{i}' for i in range(max_bins)]
                writer = csv.writer(f)
                writer.writerow(header)
                for row in self.range_profile_data:
                    writer.writerow([row['Frame'], row['Rel_Time(s)']] + row['data'])
            print(f"   💾 저장: {rp_path}")
        
        # Noise Profile CSV
        if self.noise_profile_data:
            np_path = os.path.join(self.others_dir, "noise_profile.csv")
            with open(np_path, 'w', newline='') as f:
                max_bins = max(len(d['data']) for d in self.noise_profile_data) if self.noise_profile_data else 0
                header = ['Frame', 'Rel_Time(s)'] + [f'bin_{i}' for i in range(max_bins)]
                writer = csv.writer(f)
                writer.writerow(header)
                for row in self.noise_profile_data:
                    writer.writerow([row['Frame'], row['Rel_Time(s)']] + row['data'])
            print(f"   💾 저장: {np_path}")
        
        # Heatmap CSV
        if self.heatmap_data:
            hm_path = os.path.join(self.others_dir, "azimuth_heatmap.csv")
            with open(hm_path, 'w', newline='') as f:
                writer = csv.writer(f)
                writer.writerow(['Frame', 'Rel_Time(s)', 'Width', 'Height'] + 
                               [f'val_{i}' for i in range(max(len(d['data']) for d in self.heatmap_data))])
                for row in self.heatmap_data:
                    writer.writerow([row['Frame'], row['Rel_Time(s)'], row['Width'], row['Height']] + row['data'])
            print(f"   💾 저장: {hm_path}")


# ==========================================
# 시각화 클래스
# ==========================================
class RadarVisualizer:
    def __init__(self, data_dir, save_plots=True):
        self.data_dir = data_dir
        self.save_plots = save_plots
        
        # pcd 폴더 또는 직접 경로 확인
        if os.path.exists(os.path.join(data_dir, 'pcd')):
            self.pcd_dir = os.path.join(data_dir, 'pcd')
            self.others_dir = os.path.join(data_dir, 'others')
        else:
            self.pcd_dir = data_dir
            self.others_dir = os.path.join(data_dir, 'others')
        
        self.pc_file = os.path.join(self.pcd_dir, "pointcloud.csv")
        self.hm_file = os.path.join(self.others_dir, "azimuth_heatmap.csv")
        
        print(f"📂 Loading Data from: {data_dir}")
        self.pc_data = {}
        self.hm_data = {}
        self.frames = []

        self.load_pointcloud()
        self.load_heatmap()
        
        pc_frames = set(self.pc_data.keys())
        hm_frames = set(self.hm_data.keys())
        self.frames = sorted(list(pc_frames | hm_frames))
        
        if not self.frames:
            print("❌ No frames found.")
            return
        
        print(f"✅ Loaded {len(self.frames)} frames")
        print(f"   - PointCloud frames: {len(pc_frames)}")
        print(f"   - Heatmap frames: {len(hm_frames)}")
        
        # 플롯 저장
        if self.save_plots and self.frames:
            self.save_all_plots()

    def load_pointcloud(self):
        if not os.path.exists(self.pc_file):
            print(f"⚠️ PointCloud file not found: {self.pc_file}")
            return
        try:
            df = pd.read_csv(self.pc_file)
            df.columns = [c.strip() for c in df.columns]
            if 'Frame' in df.columns:
                for frame, group in df.groupby('Frame'):
                    self.pc_data[int(frame)] = group
            print(f"   📍 PointCloud: {len(self.pc_data)} frames loaded")
        except Exception as e:
            print(f"❌ Error loading pointcloud: {e}")

    def load_heatmap(self):
        if not os.path.exists(self.hm_file):
            print(f"⚠️ Heatmap file not found: {self.hm_file}")
            return
        print("   ⏳ Loading Heatmap...")
        try:
            with open(self.hm_file, 'r') as f:
                reader = csv.reader(f)
                header = next(reader)
                for row in reader:
                    if not row:
                        continue
                    try:
                        frame = int(float(row[0]))
                        w, h = int(row[2]), int(row[3])
                        raw = np.array(row[4:], dtype=float)
                        if len(raw) >= w * h:
                            self.hm_data[frame] = raw[:w*h].reshape(h, w)
                    except Exception:
                        pass
            print(f"   🔥 Heatmap: {len(self.hm_data)} frames loaded")
        except Exception as e:
            print(f"❌ Error loading heatmap: {e}")

    def save_all_plots(self):
        """모든 프레임의 플롯 저장"""
        print(f"🖼️ 플롯 저장 중... ({len(self.frames)} frames)")
        
        for i, frame in enumerate(self.frames):
            if i % 10 == 0:
                print(f"   진행: {i}/{len(self.frames)}")
            
            # PointCloud 플롯
            if frame in self.pc_data:
                self.save_pointcloud_plot(frame)
            
            # Heatmap 플롯
            if frame in self.hm_data:
                self.save_heatmap_plot(frame)
        
        print(f"✅ 플롯 저장 완료!")

    def save_pointcloud_plot(self, frame):
        """PointCloud 플롯 저장"""
        fig, ax = plt.subplots(figsize=(8, 8))
        
        df = self.pc_data[frame]
        col_x = 'X(m)' if 'X(m)' in df.columns else 'x'
        col_y = 'Y(m)' if 'Y(m)' in df.columns else 'y'
        col_v = 'V(m/s)' if 'V(m/s)' in df.columns else 'v'

        plot_x = -df[col_y].values
        plot_y = df[col_x].values
        vs = df[col_v].values

        threshold = 0.1
        fast_mask = np.abs(vs) >= threshold
        slow_mask = ~fast_mask
        
        if np.any(slow_mask):
            ax.scatter(plot_x[slow_mask], plot_y[slow_mask], 
                      c='blue', s=20, alpha=0.5, label='Static')
        
        if np.any(fast_mask):
            ax.scatter(plot_x[fast_mask], plot_y[fast_mask], 
                      c='red', s=40, label='Moving')
            norms = np.sqrt(plot_x[fast_mask]**2 + plot_y[fast_mask]**2)
            norms[norms == 0] = 1
            u = (plot_x[fast_mask] / norms) * vs[fast_mask]
            v = (plot_y[fast_mask] / norms) * vs[fast_mask]
            ax.quiver(plot_x[fast_mask], plot_y[fast_mask], u, v, 
                     color='red', scale=1, scale_units='xy', angles='xy', width=0.005)

        ax.set_title(f'Point Cloud - Top Down View (Frame {frame})')
        ax.set_xlim(-5, 5)
        ax.set_ylim(0, MAX_RANGE_METERS)
        ax.set_xlabel('X (m)')
        ax.set_ylabel('Y (m)')
        ax.set_aspect('equal')
        ax.grid(True, alpha=0.3)
        ax.plot(0, 0, 'k+', markersize=15)
        ax.legend(loc='upper right')
        
        save_path = os.path.join(self.pcd_dir, f"frame_{frame:04d}_pointcloud.png")
        plt.savefig(save_path, dpi=100, bbox_inches='tight')
        plt.close(fig)

    def save_heatmap_plot(self, frame):
        """Heatmap 플롯 저장"""
        fig, ax = plt.subplots(figsize=(8, 6))
        
        img_data = self.hm_data[frame]
        ax.imshow(img_data, aspect='auto', origin='lower', cmap='jet', 
                 interpolation='bicubic', extent=[0, VIRTUAL_ANTENNAS, 0, MAX_RANGE_METERS])
        ax.set_title(f'Range-Azimuth Heatmap (Frame {frame})')
        ax.set_xlabel('Azimuth (Virtual Antennas)')
        ax.set_ylabel('Distance (m)')
        
        save_path = os.path.join(self.pcd_dir, f"frame_{frame:04d}_heatmap.png")
        plt.savefig(save_path, dpi=100, bbox_inches='tight')
        plt.close(fig)

    def setup_interactive_plot(self):
        """인터랙티브 플롯 설정"""
        self.fig = plt.figure(figsize=(16, 8))
        self.fig.suptitle('AWR2944 Radar Visualizer', fontsize=14, fontweight='bold')
        
        self.ax_hm = self.fig.add_subplot(1, 2, 1)
        self.ax_pc = self.fig.add_subplot(1, 2, 2)
        plt.subplots_adjust(bottom=0.15)

        ax_slider = plt.axes([0.2, 0.05, 0.6, 0.03])
        self.slider = Slider(ax_slider, 'Frame', 0, len(self.frames)-1, valinit=0, valstep=1)
        self.slider.on_changed(self.update)

        ax_play = plt.axes([0.85, 0.05, 0.1, 0.04])
        self.btn_play = Button(ax_play, 'Play')
        self.btn_play.on_clicked(self.toggle_animation)
        self.anim_running = False
        self.anim = None

        self.update(0)

    def toggle_animation(self, event):
        if self.anim_running:
            if self.anim:
                self.anim.event_source.stop()
            self.btn_play.label.set_text('Play')
            self.anim_running = False
        else:
            self.anim = FuncAnimation(
                self.fig, 
                self.animate, 
                frames=range(int(self.slider.val), len(self.frames)), 
                interval=100, 
                repeat=False
            )
            self.btn_play.label.set_text('Pause')
            self.anim_running = True
        plt.draw()

    def animate(self, frame_idx):
        self.slider.set_val(frame_idx)

    def update(self, val):
        frame_idx = int(self.slider.val)
        if frame_idx >= len(self.frames):
            return
        target_frame = self.frames[frame_idx]
        
        # --- Heatmap Update ---
        self.ax_hm.clear()
        if target_frame in self.hm_data:
            img_data = self.hm_data[target_frame]
            self.ax_hm.imshow(img_data, aspect='auto', origin='lower', cmap='jet', 
                            interpolation='bicubic', extent=[0, VIRTUAL_ANTENNAS, 0, MAX_RANGE_METERS])
            self.ax_hm.set_title(f'Range-Azimuth Heatmap (Frame {target_frame})')
            self.ax_hm.set_xlabel('Azimuth (Virtual Antennas)')
            self.ax_hm.set_ylabel('Distance (m)')
        else:
            self.ax_hm.text(0.5, 0.5, "No Heatmap Data", ha='center', va='center', 
                          transform=self.ax_hm.transAxes, fontsize=14)
            self.ax_hm.set_title(f'Range-Azimuth Heatmap (Frame {target_frame})')

        # --- Point Cloud Update ---
        self.ax_pc.clear()
        if target_frame in self.pc_data:
            df = self.pc_data[target_frame]
            col_x = 'X(m)' if 'X(m)' in df.columns else 'x'
            col_y = 'Y(m)' if 'Y(m)' in df.columns else 'y'
            col_v = 'V(m/s)' if 'V(m/s)' in df.columns else 'v'

            plot_x = -df[col_y].values
            plot_y = df[col_x].values
            vs = df[col_v].values

            threshold = 0.1
            fast_mask = np.abs(vs) >= threshold
            slow_mask = ~fast_mask
            
            if np.any(slow_mask):
                self.ax_pc.scatter(plot_x[slow_mask], plot_y[slow_mask], 
                                  c='blue', s=20, alpha=0.5, label='Static')
            
            if np.any(fast_mask):
                self.ax_pc.scatter(plot_x[fast_mask], plot_y[fast_mask], 
                                  c='red', s=40, label='Moving')
                norms = np.sqrt(plot_x[fast_mask]**2 + plot_y[fast_mask]**2)
                norms[norms == 0] = 1
                u = (plot_x[fast_mask] / norms) * vs[fast_mask]
                v = (plot_y[fast_mask] / norms) * vs[fast_mask]
                self.ax_pc.quiver(plot_x[fast_mask], plot_y[fast_mask], u, v, 
                                 color='red', scale=1, scale_units='xy', angles='xy', width=0.005)
            self.ax_pc.legend(loc='upper right')
        else:
            self.ax_pc.text(0.5, 0.5, "No PointCloud Data", ha='center', va='center',
                          transform=self.ax_pc.transAxes, fontsize=14)
        
        self.ax_pc.set_title(f'Point Cloud - Top Down View (Frame {target_frame})')
        self.ax_pc.set_xlim(-5, 5)
        self.ax_pc.set_ylim(0, MAX_RANGE_METERS)
        self.ax_pc.set_xlabel('X (m)')
        self.ax_pc.set_ylabel('Y (m)')
        self.ax_pc.set_aspect('equal')
        self.ax_pc.grid(True, alpha=0.3)
        self.ax_pc.plot(0, 0, 'k+', markersize=15)

        self.fig.canvas.draw_idle()

    def show(self):
        """인터랙티브 시각화 실행"""
        if not self.frames:
            print("❌ No data to visualize")
            return
        self.setup_interactive_plot()
        plt.show()


# ==========================================
# 메인 함수
# ==========================================
def main():
    parser = argparse.ArgumentParser(
        description='Radar Data Visualizer & Converter',
        formatter_class=argparse.RawDescriptionHelpFormatter,
        epilog="""
예제:
  python3 radar_visualizer.py                      # 최신 rosbag 변환 + 시각화
  python3 radar_visualizer.py -d /path/to/rosbag   # 특정 rosbag 처리
  python3 radar_visualizer.py --convert-only       # 변환만
  python3 radar_visualizer.py --visualize-only     # CSV 시각화만
        """
    )
    parser.add_argument('-d', '--data-dir', type=str, default=None,
                        help='rosbag 또는 CSV 폴더 경로')
    parser.add_argument('-b', '--base-dir', type=str, default=DEFAULT_ROSBAG_DIR,
                        help='rosbag 검색 기본 경로')
    parser.add_argument('--convert-only', action='store_true',
                        help='변환만 수행 (시각화 없음)')
    parser.add_argument('--visualize-only', action='store_true',
                        help='기존 CSV 시각화만')
    parser.add_argument('--no-save-plots', action='store_true',
                        help='개별 프레임 플롯 저장 안함')
    
    args = parser.parse_args()
    
    # 경로 결정
    if args.data_dir:
        data_path = os.path.expanduser(args.data_dir)
    else:
        data_path = None
    
    # 시각화만 모드
    if args.visualize_only:
        if data_path is None:
            data_path = find_latest_transformed(args.base_dir)
        
        if not data_path:
            print("❌ CSV 데이터를 찾을 수 없습니다.")
            print("   먼저 rosbag을 변환하세요: python3 radar_visualizer.py")
            sys.exit(1)
        
        visualizer = RadarVisualizer(data_path, save_plots=False)
        visualizer.show()
        return
    
    # 변환 수행
    if data_path is None:
        data_path = find_latest_rosbag(args.base_dir)
    
    if not data_path:
        print(f"❌ rosbag을 찾을 수 없습니다.")
        print(f"   검색 경로: {args.base_dir}")
        print(f"   먼저 레이더 데이터를 녹화하세요:")
        print(f"   ros2 launch nlos_awr2944 single_radar.launch.py record_bag:=true")
        sys.exit(1)
    
    print("=" * 50)
    print("   AWR2944 Radar Data Converter & Visualizer")
    print("=" * 50)
    
    # rosbag 변환
    converter = RosbagConverter(data_path)
    output_dir = converter.convert()
    
    if output_dir is None:
        print("❌ 변환 실패")
        sys.exit(1)
    
    # 변환만 모드
    if args.convert_only:
        print(f"\n✅ 변환 완료: {output_dir}")
        return
    
    # 시각화
    print("\n" + "=" * 50)
    print("   시각화 시작")
    print("=" * 50)
    
    visualizer = RadarVisualizer(output_dir, save_plots=not args.no_save_plots)
    visualizer.show()


if __name__ == "__main__":
    main()