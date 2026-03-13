import sqlite3
from rclpy.serialization import deserialize_message
from std_msgs.msg import Float32MultiArray
import pandas as pd
import numpy as np
import os
import argparse
import matplotlib.pyplot as plt

def extract_from_bag(bag_path, output_csv):
    # 1. DB 파일 찾기 및 폴더 설정
    db_file = None
    for f in os.listdir(bag_path):
        if f.endswith('.db3'):
            db_file = os.path.join(bag_path, f)
            break
    
    if not db_file:
        print(f"❌ Error: No .db3 file found in {bag_path}")
        return

    bag_name = os.path.basename(os.path.normpath(bag_path))
    image_dir = os.path.join("vis_results", bag_name)
    os.makedirs(image_dir, exist_ok=True)

    conn = sqlite3.connect(db_file)
    cursor = conn.cursor()

    cursor.execute("SELECT id FROM topics WHERE name = '/audio/spectrum'")
    topic_row = cursor.fetchone()
    if not topic_row:
        print("❌ Error: Topic not found.")
        return
    topic_id = topic_row[0]

    data_rows = []
    frame_count = 0

    print(f"🚀 Processing bag: {bag_name}...")
    
    cursor.execute("SELECT timestamp, data FROM messages WHERE topic_id = ?", (topic_id,))
    
    for timestamp_ns, data_blob in cursor.fetchall():
        try:
            msg = deserialize_message(data_blob, Float32MultiArray)
            spectrum = np.array(msg.data)
            
            # [수정] 데이터 개수에 맞춰 azimuth_grid를 유연하게 설정
            num_angles = len(spectrum)
            azimuth_grid = np.linspace(-180, 180, num_angles, endpoint=False)
            azimuth_rad = np.deg2rad(azimuth_grid)

            max_idx = np.argmax(spectrum)
            max_angle = azimuth_grid[max_idx]
            timestamp_sec = timestamp_ns / 1e9

            data_rows.append([frame_count, timestamp_sec, max_angle] + spectrum.tolist())

            # === 시각화 수정: 잘린 부분 없이 전체 그리기 ===
            fig = plt.figure(figsize=(8, 8))
            ax = fig.add_subplot(111, projection='polar')
            
            # CSV 데이터(-180~180)를 그대로 그래프에 대응
            ax.plot(azimuth_rad, spectrum, color='red', linewidth=2)
            
            # [핵심] 정면 0도를 위쪽(N)으로 고정
            ax.set_theta_zero_location('N')
            # [핵심] 왼쪽이 +, 오른쪽이 -가 되도록 방향 설정
            ax.set_theta_direction(1) 
            
            # 전체 360도 표시 (잘림 방지)
            ax.set_ylim(0, np.max(spectrum) * 1.1) 
            
            # 각도 라벨 설정 (-180부터 180까지 45도 간격)
            ticks = np.arange(-180, 181, 45)
            ax.set_xticks(np.deg2rad(ticks))
            ax.set_xticklabels([f"{t}°" for t in ticks])
            
            ax.set_title(f"Frame {frame_count:04d} | Max: {max_angle:.1f}°", pad=30, fontsize=15)
            ax.grid(True, linestyle=':', alpha=0.6)
            
            # 이미지 저장
            plt.savefig(os.path.join(image_dir, f"frame_{frame_count:04d}.png"), bbox_inches='tight')
            plt.close(fig)
            
            frame_count += 1
            if frame_count % 10 == 0:
                print(f"  Processed {frame_count} frames...", end='\r')

        except Exception as e:
            print(f"⚠️ Error: {e}")

    conn.close()
    
    # CSV 저장 (마지막 확인된 num_angles 기준)
    cols = ['frame', 'timestamp', 'max_angle'] + [f"deg_{int(deg)}" for deg in azimuth_grid]
    pd.DataFrame(data_rows, columns=cols).to_csv(output_csv, index=False)
    print(f"\n✅ Complete! All 360 degrees visualized in {image_dir}")

if __name__ == "__main__":
    parser = argparse.ArgumentParser()
    parser.add_argument("bag_folder", type=str)
    args = parser.parse_args()
    extract_from_bag(args.bag_folder, "output_audio.csv")