#!/usr/bin/env python3
"""
==============================================
NLOS Audio-Only Rosbag → Video Processor
==============================================
rosbag에서 음향 데이터만 추출하여:
  1. 프레임별 polar plot 이미지 생성
  2. 10fps 영상(mp4)으로 합성

사용법:
  python3 process_audio_video.py                    # 최신 rosbag 자동
  python3 process_audio_video.py /path/to/rosbag    # 특정 rosbag
  python3 process_audio_video.py --fps 10           # FPS 지정
  python3 process_audio_video.py --skip-plots       # 이미 plot이 있으면 영상만 생성
"""

import os
import sys
import glob
import cv2
import numpy as np
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
import argparse
import warnings
warnings.filterwarnings('ignore')

try:
    from rclpy.serialization import deserialize_message
    from rosbag2_py import SequentialReader, StorageOptions, ConverterOptions
    from std_msgs.msg import Float32MultiArray, Float32, String
    HAS_ROS = True
except ImportError:
    HAS_ROS = False
    print("⚠️ ROS2 라이브러리가 필요합니다.")

# ==========================================
# 설정
# ==========================================
WORKSPACE_DIR = os.path.expanduser('~/Nlos/Ros2_setup')
ROSBAG_DIR = os.path.join(WORKSPACE_DIR, 'rosbag')
OUTPUT_BASE_DIR = os.path.join(WORKSPACE_DIR, 'processed_data')

AUDIO_ANGLE_MIN, AUDIO_ANGLE_MAX = 0, 180
PLOT_DPI = 100
PLOT_FIGSIZE = (14, 4)


def get_latest_bag_path(base_dir):
    all_bags = glob.glob(os.path.join(base_dir, 'sync_*'))
    if not all_bags:
        print("❌ Rosbag 폴더가 비어있습니다.")
        return None
    return max(all_bags, key=os.path.getctime)


class AudioVideoProcessor:
    def __init__(self, bag_path, fps=10):
        self.bag_path = bag_path
        self.bag_name = os.path.basename(bag_path)
        self.fps = fps
        
        self.output_dir = os.path.join(OUTPUT_BASE_DIR, self.bag_name)
        self.plots_dir = os.path.join(self.output_dir, 'acoustic', 'plots')
        self.csv_dir = os.path.join(self.output_dir, 'acoustic', 'csv')
        os.makedirs(self.plots_dir, exist_ok=True)
        os.makedirs(self.csv_dir, exist_ok=True)
        
        self.frame_count = 0

    def extract_audio_plots(self):
        """rosbag에서 audio/spectrum 토픽만 읽어 polar plot 이미지 생성"""
        if not HAS_ROS:
            print("❌ ROS2 라이브러리가 필요합니다.")
            return False

        print(f"🎵 Audio extraction from: {self.bag_path}")
        
        storage_options = StorageOptions(uri=self.bag_path, storage_id='sqlite3')
        converter_options = ConverterOptions(
            input_serialization_format='cdr',
            output_serialization_format='cdr'
        )
        reader = SequentialReader()
        reader.open(storage_options, converter_options)

        topics = reader.get_all_topics_and_types()
        type_map = {t.name: t.type for t in topics}
        
        # 오디오 토픽 확인
        audio_topics = [t.name for t in topics if 'audio' in t.name.lower()]
        print(f"   📋 오디오 토픽: {audio_topics}")
        
        if not any('spectrum' in t for t in audio_topics):
            print("❌ audio/spectrum 토픽을 찾을 수 없습니다.")
            return False

        self.frame_count = 0
        while reader.has_next():
            (topic, data, t) = reader.read_next()
            
            if 'audio/spectrum' not in topic and 'audio_spectrum' not in topic:
                continue
            
            try:
                msg = deserialize_message(data, Float32MultiArray)
                timestamp = t / 1e9
                self.frame_count += 1
                frame_idx = self.frame_count
                
                spectrum = np.array(msg.data)
                num_angles = len(spectrum)
                azimuth_grid = np.linspace(-180, 180, num_angles, endpoint=False)
                
                max_idx = np.argmax(spectrum)
                max_angle = azimuth_grid[max_idx]
                
                self._save_plot(spectrum, azimuth_grid, frame_idx, max_angle, timestamp)
                
                if frame_idx % 50 == 0:
                    print(f"   🔊 Frame {frame_idx} processed (angle: {max_angle:.1f}°)")
                    
            except Exception as e:
                print(f"   ⚠️ Frame 처리 오류: {e}")
                continue

        print(f"   ✅ 총 {self.frame_count} 프레임 추출 완료")
        return self.frame_count > 0

    def _save_plot(self, spectrum, azimuth_grid, frame_idx, max_angle, timestamp):
        """프레임별 polar plot 저장 (위쪽 반원, 좌우 넓게)"""
        fig = plt.figure(figsize=(14, 5.5))
        # 반원을 약간 위로 올려서 아래 텍스트와 안 겹치게
        ax = fig.add_axes([0.05, -0.15, 0.9, 1.2], projection='polar')
        
        # 0°=오른쪽, 반시계방향 → 위쪽 반원
        ax.set_theta_zero_location('E')
        ax.set_theta_direction(1)
        
        # 0~180도 필터링
        mask = (azimuth_grid >= AUDIO_ANGLE_MIN) & (azimuth_grid <= AUDIO_ANGLE_MAX)
        angles_filtered = azimuth_grid[mask]
        spectrum_filtered = spectrum[mask]
        
        ax.plot(np.deg2rad(angles_filtered), spectrum_filtered, color='red', linewidth=1.5)
        ax.fill(np.deg2rad(angles_filtered), spectrum_filtered, color='red', alpha=0.3)
        
        # 최대 방향 표시
        if AUDIO_ANGLE_MIN <= max_angle <= AUDIO_ANGLE_MAX:
            max_val = spectrum[np.argmax(spectrum)]
            ax.plot(np.deg2rad(max_angle), max_val, 'bo', markersize=10)
            ax.annotate(f'{max_angle:.1f}°', 
                       xy=(np.deg2rad(max_angle), max_val),
                       xytext=(0, 12), textcoords='offset points', 
                       fontsize=11, fontweight='bold',
                       color='blue', ha='center')
        
        ax.set_thetamin(AUDIO_ANGLE_MIN)
        ax.set_thetamax(AUDIO_ANGLE_MAX)
        
        # 타이틀 아래쪽 배치
        fig.text(0.5, 0.03, f'Audio Direction | Max: {max_angle:.1f}° | Time: {timestamp:.3f}s',
                 fontsize=13, ha='center')
        
        plt.savefig(os.path.join(self.plots_dir, f'frame_{frame_idx:04d}.png'),
                   dpi=PLOT_DPI)
        plt.close(fig)

    def make_video(self):
        """plots 폴더의 이미지들을 영상으로 합성"""
        # 이미지 파일 정렬
        image_files = sorted(glob.glob(os.path.join(self.plots_dir, 'frame_*.png')))
        
        if not image_files:
            print("❌ plot 이미지가 없습니다. 먼저 extract_audio_plots()를 실행하세요.")
            return None
        
        print(f"\n🎬 영상 생성 중... ({len(image_files)} frames @ {self.fps}fps)")
        
        # 첫 프레임으로 크기 결정
        first_img = cv2.imread(image_files[0])
        h, w = first_img.shape[:2]
        
        # 출력 경로
        video_path = os.path.join(self.output_dir, 'acoustic', 
                                  f'audio_direction_{self.fps}fps.mp4')
        
        # VideoWriter (mp4v 코덱)
        fourcc = cv2.VideoWriter_fourcc(*'mp4v')
        writer = cv2.VideoWriter(video_path, fourcc, self.fps, (w, h))
        
        if not writer.isOpened():
            # mp4v 실패 시 XVID로 폴백
            video_path = video_path.replace('.mp4', '.avi')
            fourcc = cv2.VideoWriter_fourcc(*'XVID')
            writer = cv2.VideoWriter(video_path, fourcc, self.fps, (w, h))
        
        for i, img_path in enumerate(image_files):
            img = cv2.imread(img_path)
            if img is not None:
                # 크기 통일 (혹시 다른 게 있으면)
                if img.shape[:2] != (h, w):
                    img = cv2.resize(img, (w, h))
                writer.write(img)
            
            if (i + 1) % 100 == 0:
                duration = (i + 1) / self.fps
                print(f"   📝 {i+1}/{len(image_files)} frames ({duration:.1f}s)")
        
        writer.release()
        
        duration = len(image_files) / self.fps
        file_size_mb = os.path.getsize(video_path) / (1024 * 1024)
        
        print(f"\n{'='*50}")
        print(f"  ✅ 영상 생성 완료!")
        print(f"  📁 경로: {video_path}")
        print(f"  🎞️  프레임: {len(image_files)}")
        print(f"  ⏱️  길이: {duration:.1f}초")
        print(f"  📊 FPS: {self.fps}")
        print(f"  💾 크기: {file_size_mb:.1f}MB")
        print(f"{'='*50}")
        
        return video_path


def main():
    parser = argparse.ArgumentParser(description='NLOS Audio Rosbag → Video')
    parser.add_argument('bag_path', nargs='?', default=None,
                       help='rosbag 폴더 경로 (없으면 최신 자동)')
    parser.add_argument('--fps', type=int, default=10,
                       help='출력 영상 FPS (default: 10)')
    parser.add_argument('--skip-plots', action='store_true',
                       help='plot 생성 건너뛰고 영상만 생성 (이미 plot이 있을 때)')
    
    args = parser.parse_args()
    
    if args.bag_path:
        bag_path = os.path.expanduser(args.bag_path)
    else:
        bag_path = get_latest_bag_path(ROSBAG_DIR)
    
    if not bag_path or not os.path.exists(bag_path):
        print(f"❌ rosbag을 찾을 수 없습니다: {bag_path}")
        print(f"   검색 경로: {ROSBAG_DIR}")
        return
    
    processor = AudioVideoProcessor(bag_path, fps=args.fps)
    
    if not args.skip_plots:
        processor.extract_audio_plots()
    else:
        print("⏩ plot 생성 건너뜀 (--skip-plots)")
    
    processor.make_video()


if __name__ == '__main__':
    main()