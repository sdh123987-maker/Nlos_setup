#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32MultiArray, Float32
import numpy as np
import pyroomacoustics as pra
from scipy import signal
import xml.etree.ElementTree as ET
import os
from ament_index_python.packages import get_package_share_directory

class SoundLocalizer(Node):
    def __init__(self):
        super().__init__('sound_localizer')

        self.declare_parameter('channels', 16)
        self.declare_parameter('sample_rate', 48000)
        self.declare_parameter('window_size', 4096)

        self.channels = self.get_parameter('channels').value
        self.fs = self.get_parameter('sample_rate').value
        self.window_size = self.get_parameter('window_size').value

        default_xml = os.path.join(
            get_package_share_directory('nlos_fusion'),
            'config', 'mic_geometry.xml'
        )
        self.declare_parameter('mic_config_file', default_xml)
        xml_path = self.get_parameter('mic_config_file').value
        self.mic_locs = self.load_mic_geometry_from_xml(xml_path)

        self.sub_audio = self.create_subscription(
            Float32MultiArray, 'audio/raw', self.listener_callback, 10)
        
        self.pub_direction = self.create_publisher(Float32, 'audio/direction', 10)
        self.pub_spectrum = self.create_publisher(Float32MultiArray, 'audio/spectrum', 10)

        self.audio_buffer = np.zeros((self.window_size, self.channels), dtype=np.float32)
        self.buffer_idx = 0
        self.nfft = 512

        # 1도 단위 해상도 (360개)
        self.num_angles = 360
        self.azimuth_grid = np.linspace(-180, 180, self.num_angles, endpoint=False) * np.pi / 180

        try:
            self.doa = pra.doa.algorithms['SRP'](
                self.mic_locs, 
                self.fs, 
                self.nfft, 
                num_src=1, 
                azimuth=self.azimuth_grid,
                max_four=4,
                dim=2
            )
            self.get_logger().info("👂 Sound Localizer Ready (Continuous Recording Mode)")
        except Exception as e:
            self.get_logger().error(f"Failed to init DOA: {e}")

    def load_mic_geometry_from_xml(self, xml_path):
        try:
            tree = ET.parse(xml_path)
            root = tree.getroot()
            locs = []
            for pos in root.findall('pos'):
                x = float(pos.get('x'))
                y = float(pos.get('y'))
                locs.append([x, y])
            locs = np.array(locs).T 
            locs[0, :] -= locs[0, :].mean()
            locs[1, :] -= locs[1, :].mean()
            return locs
        except Exception as e:
            self.get_logger().error(f"Failed to load XML: {e}")
            return np.zeros((2, self.channels))

    def listener_callback(self, msg):
        if not rclpy.ok(): return
        raw_data = np.array(msg.data, dtype=np.float32)
        num_samples = len(raw_data) // self.channels
        chunk = raw_data.reshape((num_samples, self.channels))

        remaining = self.window_size - self.buffer_idx
        if num_samples < remaining:
            self.audio_buffer[self.buffer_idx : self.buffer_idx + num_samples] = chunk
            self.buffer_idx += num_samples
        else:
            self.audio_buffer[self.buffer_idx : ] = chunk[:remaining]
            
            # [수정] Threshold 제거 -> 무조건 처리
            self.process_doa(self.audio_buffer)
            
            self.audio_buffer = np.zeros_like(self.audio_buffer)
            self.buffer_idx = 0
            leftover = chunk[remaining:]
            if len(leftover) > 0:
                self.audio_buffer[0:len(leftover)] = leftover
                self.buffer_idx = len(leftover)

    def process_doa(self, signal_window):
        try:
            container = []
            for i in range(self.channels):
                _, _, Zxx = signal.stft(signal_window[:, i], fs=self.fs, nperseg=self.nfft)
                container.append(Zxx)
            X = np.stack(container)

            self.doa.locate_sources(X, freq_range=[500, 4000])
            
            spatial_spectrum = self.doa.grid.values # 전체 스펙트럼
            max_idx = np.argmax(spatial_spectrum)
            azimuth_deg = self.azimuth_grid[max_idx] * 180 / np.pi
            
            dir_msg = Float32()
            dir_msg.data = float(azimuth_deg)
            self.pub_direction.publish(dir_msg)

            spec_msg = Float32MultiArray()
            spec_msg.data = spatial_spectrum.tolist()
            self.pub_spectrum.publish(spec_msg)
            
            # 로그는 너무 빠르니 주석 처리 하거나 필요시 해제
            # self.get_logger().info(f"🔊 Angle: {azimuth_deg:.2f}°")
            
        except Exception as e:
            self.get_logger().warn(f"DOA Error: {e}")

def main(args=None):
    rclpy.init(args=args)
    node = SoundLocalizer()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if rclpy.ok():
            node.destroy_node()
            rclpy.shutdown()

if __name__ == '__main__':
    main()