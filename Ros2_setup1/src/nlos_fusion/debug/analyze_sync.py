#!/usr/bin/env python3
"""
==============================================
Sync Quality Analyzer
==============================================
rosbag의 /sync/frame_info 토픽을 분석하여
각 센서별 동기화 품질을 확인하는 스크립트

사용법:
  python3 analyze_sync.py /path/to/rosbag_folder
  python3 analyze_sync.py  # 최신 rosbag 자동 탐색

출력:
  - 콘솔에 통계 출력
  - sync_analysis.csv 파일 생성
  - sync_analysis.png 그래프 생성
"""

import os
import sys
import argparse
from datetime import datetime

try:
    from rosbags.rosbag2 import Reader
    from rosbags.serde import deserialize_cdr
    HAS_ROSBAGS = True
except ImportError:
    HAS_ROSBAGS = False
    print("⚠️ rosbags 라이브러리 필요: pip install rosbags")

import numpy as np
import matplotlib.pyplot as plt
import csv

# 기본 rosbag 경로
DEFAULT_ROSBAG_DIR = os.path.expanduser("~/Nlos/Ros2_setup/rosbag")


def find_latest_rosbag(base_path):
    """가장 최근 rosbag 폴더 찾기"""
    base_path = os.path.expanduser(base_path)
    if not os.path.exists(base_path):
        return None
    
    rosbag_dirs = []
    for item in os.listdir(base_path):
        item_path = os.path.join(base_path, item)
        if os.path.isdir(item_path):
            if os.path.exists(os.path.join(item_path, 'metadata.yaml')):
                rosbag_dirs.append(item_path)
    
    if not rosbag_dirs:
        return None
    
    return max(rosbag_dirs, key=os.path.getmtime)


def parse_frame_info(data_str):
    """
    frame_info 문자열 파싱
    형식: frame,sync_time,sensors,max_diff_ms,radar_orig,radar_diff,lidar_orig,lidar_diff,camera_orig,camera_diff,audio_orig,audio_diff
    """
    parts = data_str.split(',')
    if len(parts) < 12:
        return None
    
    try:
        return {
            'frame': int(parts[0]),
            'sync_time': float(parts[1]),
            'sensors': int(parts[2]),
            'max_diff_ms': float(parts[3]),
            'radar_orig': float(parts[4]),
            'radar_diff_ms': float(parts[5]),
            'lidar_orig': float(parts[6]),
            'lidar_diff_ms': float(parts[7]),
            'camera_orig': float(parts[8]),
            'camera_diff_ms': float(parts[9]),
            'audio_orig': float(parts[10]),
            'audio_diff_ms': float(parts[11]),
        }
    except (ValueError, IndexError):
        return None


def analyze_rosbag(rosbag_path):
    """rosbag에서 frame_info 추출 및 분석"""
    if not HAS_ROSBAGS:
        print("❌ rosbags 라이브러리가 필요합니다")
        return None
    
    print(f"📂 분석 중: {rosbag_path}")
    
    frame_data = []
    
    try:
        with Reader(rosbag_path) as reader:
            for connection, timestamp, rawdata in reader.messages():
                if connection.topic == '/sync/frame_info':
                    msg = deserialize_cdr(rawdata, connection.msgtype)
                    parsed = parse_frame_info(msg.data)
                    if parsed:
                        frame_data.append(parsed)
    except Exception as e:
        print(f"❌ rosbag 읽기 오류: {e}")
        return None
    
    if not frame_data:
        print("❌ frame_info 데이터가 없습니다")
        return None
    
    print(f"✅ {len(frame_data)} 프레임 로드됨")
    return frame_data


def calculate_statistics(frame_data):
    """통계 계산"""
    radar_diffs = [f['radar_diff_ms'] for f in frame_data if f['radar_diff_ms'] != 0]
    lidar_diffs = [f['lidar_diff_ms'] for f in frame_data if f['lidar_diff_ms'] != 0]
    camera_diffs = [f['camera_diff_ms'] for f in frame_data if f['camera_diff_ms'] != 0]
    audio_diffs = [f['audio_diff_ms'] for f in frame_data if f['audio_diff_ms'] != 0]
    max_diffs = [f['max_diff_ms'] for f in frame_data]
    sensors_counts = [f['sensors'] for f in frame_data]
    
    stats = {
        'total_frames': len(frame_data),
        'radar': {
            'count': len(radar_diffs),
            'mean': np.mean(radar_diffs) if radar_diffs else 0,
            'std': np.std(radar_diffs) if radar_diffs else 0,
            'max': np.max(radar_diffs) if radar_diffs else 0,
            'min': np.min(radar_diffs) if radar_diffs else 0,
        },
        'lidar': {
            'count': len(lidar_diffs),
            'mean': np.mean(lidar_diffs) if lidar_diffs else 0,
            'std': np.std(lidar_diffs) if lidar_diffs else 0,
            'max': np.max(lidar_diffs) if lidar_diffs else 0,
            'min': np.min(lidar_diffs) if lidar_diffs else 0,
        },
        'camera': {
            'count': len(camera_diffs),
            'mean': np.mean(camera_diffs) if camera_diffs else 0,
            'std': np.std(camera_diffs) if camera_diffs else 0,
            'max': np.max(camera_diffs) if camera_diffs else 0,
            'min': np.min(camera_diffs) if camera_diffs else 0,
        },
        'audio': {
            'count': len(audio_diffs),
            'mean': np.mean(audio_diffs) if audio_diffs else 0,
            'std': np.std(audio_diffs) if audio_diffs else 0,
            'max': np.max(audio_diffs) if audio_diffs else 0,
            'min': np.min(audio_diffs) if audio_diffs else 0,
        },
        'overall': {
            'mean_max_diff': np.mean(max_diffs),
            'max_max_diff': np.max(max_diffs),
            'mean_sensors': np.mean(sensors_counts),
        }
    }
    
    return stats


def print_statistics(stats):
    """통계 출력"""
    print("\n" + "="*60)
    print("           동기화 품질 분석 결과")
    print("="*60)
    print(f"총 프레임 수: {stats['total_frames']}")
    print(f"평균 활성 센서 수: {stats['overall']['mean_sensors']:.1f}")
    print(f"평균 최대 시간차: {stats['overall']['mean_max_diff']:.2f} ms")
    print(f"최대 시간차: {stats['overall']['max_max_diff']:.2f} ms")
    print("-"*60)
    print(f"{'센서':<10} {'프레임수':<10} {'평균(ms)':<12} {'표준편차':<12} {'최대(ms)':<10}")
    print("-"*60)
    
    for sensor in ['radar', 'lidar', 'camera', 'audio']:
        s = stats[sensor]
        print(f"{sensor:<10} {s['count']:<10} {s['mean']:<12.2f} {s['std']:<12.2f} {s['max']:<10.2f}")
    
    print("="*60)
    
    # 동기화 품질 평가
    max_diff = stats['overall']['max_max_diff']
    if max_diff < 20:
        quality = "🟢 우수 (< 20ms)"
    elif max_diff < 50:
        quality = "🟡 양호 (< 50ms)"
    elif max_diff < 100:
        quality = "🟠 보통 (< 100ms)"
    else:
        quality = "🔴 주의 (>= 100ms)"
    
    print(f"동기화 품질: {quality}")
    print("="*60)


def save_csv(frame_data, output_path):
    """CSV 파일 저장"""
    with open(output_path, 'w', newline='') as f:
        fieldnames = ['frame', 'sync_time', 'sensors', 'max_diff_ms',
                     'radar_orig', 'radar_diff_ms', 
                     'lidar_orig', 'lidar_diff_ms',
                     'camera_orig', 'camera_diff_ms',
                     'audio_orig', 'audio_diff_ms']
        writer = csv.DictWriter(f, fieldnames=fieldnames)
        writer.writeheader()
        writer.writerows(frame_data)
    
    print(f"💾 CSV 저장: {output_path}")


def plot_analysis(frame_data, output_path):
    """분석 그래프 생성"""
    frames = [f['frame'] for f in frame_data]
    radar_diffs = [f['radar_diff_ms'] for f in frame_data]
    lidar_diffs = [f['lidar_diff_ms'] for f in frame_data]
    camera_diffs = [f['camera_diff_ms'] for f in frame_data]
    audio_diffs = [f['audio_diff_ms'] for f in frame_data]
    max_diffs = [f['max_diff_ms'] for f in frame_data]
    
    fig, axes = plt.subplots(3, 1, figsize=(14, 10))
    
    # 1. 각 센서별 시간차
    ax1 = axes[0]
    ax1.plot(frames, radar_diffs, 'r-', alpha=0.7, label='Radar', linewidth=0.5)
    ax1.plot(frames, lidar_diffs, 'b-', alpha=0.7, label='Lidar', linewidth=0.5)
    ax1.plot(frames, camera_diffs, 'g-', alpha=0.7, label='Camera', linewidth=0.5)
    ax1.plot(frames, audio_diffs, 'm-', alpha=0.7, label='Audio', linewidth=0.5)
    ax1.set_xlabel('Frame')
    ax1.set_ylabel('Time Diff (ms)')
    ax1.set_title('Sensor Time Differences from Sync Timestamp')
    ax1.legend(loc='upper right')
    ax1.grid(True, alpha=0.3)
    ax1.axhline(y=50, color='orange', linestyle='--', alpha=0.5, label='50ms threshold')
    
    # 2. 최대 시간차
    ax2 = axes[1]
    ax2.plot(frames, max_diffs, 'k-', alpha=0.7, linewidth=0.5)
    ax2.fill_between(frames, 0, max_diffs, alpha=0.3)
    ax2.set_xlabel('Frame')
    ax2.set_ylabel('Max Time Diff (ms)')
    ax2.set_title('Maximum Time Difference per Frame')
    ax2.grid(True, alpha=0.3)
    ax2.axhline(y=50, color='orange', linestyle='--', alpha=0.5)
    
    # 3. 히스토그램
    ax3 = axes[2]
    all_diffs = radar_diffs + lidar_diffs + camera_diffs + audio_diffs
    all_diffs = [d for d in all_diffs if d != 0]
    ax3.hist(all_diffs, bins=50, edgecolor='black', alpha=0.7)
    ax3.set_xlabel('Time Diff (ms)')
    ax3.set_ylabel('Count')
    ax3.set_title('Distribution of Time Differences')
    ax3.axvline(x=np.mean(all_diffs), color='r', linestyle='--', label=f'Mean: {np.mean(all_diffs):.1f}ms')
    ax3.legend()
    ax3.grid(True, alpha=0.3)
    
    plt.tight_layout()
    plt.savefig(output_path, dpi=150)
    plt.close()
    
    print(f"📊 그래프 저장: {output_path}")


def main():
    parser = argparse.ArgumentParser(description='Sync Quality Analyzer')
    parser.add_argument('rosbag_path', nargs='?', default=None,
                       help='rosbag 폴더 경로 (없으면 최신 자동 탐색)')
    parser.add_argument('-o', '--output-dir', default=None,
                       help='출력 폴더 (기본: rosbag 폴더)')
    
    args = parser.parse_args()
    
    # rosbag 경로 결정
    if args.rosbag_path:
        rosbag_path = os.path.expanduser(args.rosbag_path)
    else:
        rosbag_path = find_latest_rosbag(DEFAULT_ROSBAG_DIR)
    
    if not rosbag_path or not os.path.exists(rosbag_path):
        print(f"❌ rosbag을 찾을 수 없습니다: {rosbag_path}")
        print(f"   검색 경로: {DEFAULT_ROSBAG_DIR}")
        sys.exit(1)
    
    # 출력 폴더
    output_dir = args.output_dir or os.path.dirname(rosbag_path)
    
    # 분석 실행
    print("="*60)
    print("       NLOS Multi-Sensor Sync Quality Analyzer")
    print("="*60)
    
    frame_data = analyze_rosbag(rosbag_path)
    if frame_data is None:
        sys.exit(1)
    
    # 통계 계산 및 출력
    stats = calculate_statistics(frame_data)
    print_statistics(stats)
    
    # 파일 저장
    timestamp = datetime.now().strftime('%Y%m%d_%H%M%S')
    csv_path = os.path.join(output_dir, f'sync_analysis_{timestamp}.csv')
    png_path = os.path.join(output_dir, f'sync_analysis_{timestamp}.png')
    
    save_csv(frame_data, csv_path)
    plot_analysis(frame_data, png_path)
    
    print("\n✅ 분석 완료!")


if __name__ == "__main__":
    main()