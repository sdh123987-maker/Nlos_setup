#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32MultiArray, MultiArrayDimension, Header
import sounddevice as sd
import numpy as np
import sys

class AudioPublisher(Node):
    def __init__(self):
        super().__init__('audio_publisher')
        
        self.declare_parameter('device_name', 'UMIK')
        self.declare_parameter('channels', 16)
        self.declare_parameter('sample_rate', 48000)
        self.declare_parameter('chunk_size', 1024)
        self.declare_parameter('frame_id', 'mic_array_link')

        self.device_name_substr = self.get_parameter('device_name').value
        self.channels = self.get_parameter('channels').value
        self.fs = self.get_parameter('sample_rate').value
        self.chunk_size = self.get_parameter('chunk_size').value
        self.frame_id = self.get_parameter('frame_id').value

        self.pub_audio = self.create_publisher(Float32MultiArray, 'audio/raw', 10)
        self.pub_header = self.create_publisher(Header, 'audio/header', 10)

        self.device_id = self.find_device(self.device_name_substr)
        if self.device_id is None:
            self.get_logger().error(f"❌ Microphone matching '{self.device_name_substr}' not found!")
            sys.exit(1)

        self.get_logger().info(f"✅ Found Device: ID {self.device_id}")

        try:
            self.stream = sd.InputStream(
                device=self.device_id,
                channels=self.channels,
                samplerate=self.fs,
                dtype='float32',
                blocksize=self.chunk_size,
                callback=self.audio_callback
            )
            self.stream.start()
        except Exception as e:
            self.get_logger().error(f"Failed to start stream: {e}")
            sys.exit(1)

    def find_device(self, name_substr):
        devices = sd.query_devices()
        for i, dev in enumerate(devices):
            if name_substr in dev['name'] and dev['max_input_channels'] >= self.channels:
                return i
        return None

    def audio_callback(self, indata, frames, time, status):
        # ROS가 종료되었으면 즉시 리턴
        if not rclpy.ok():
            return

        if status:
            self.get_logger().warn(f"Audio Status: {status}")

        try:
            now = self.get_clock().now().to_msg()

            header_msg = Header()
            header_msg.stamp = now
            header_msg.frame_id = self.frame_id
            self.pub_header.publish(header_msg)

            flat_data = indata.flatten().tolist()
            msg = Float32MultiArray()
            
            dim_chunk = MultiArrayDimension(label="samples", size=frames, stride=frames*self.channels)
            dim_chan = MultiArrayDimension(label="channels", size=self.channels, stride=self.channels)
            msg.layout.dim = [dim_chunk, dim_chan]
            msg.data = flat_data
            
            self.pub_audio.publish(msg)
        except Exception:
            # 종료 시 발생하는 퍼블리시 에러 무시
            pass

    def destroy_node(self):
        if hasattr(self, 'stream'):
            self.stream.stop()
            self.stream.close()
        super().destroy_node()

def main(args=None):
    rclpy.init(args=args)
    node = AudioPublisher()
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