import rclpy
from rclpy.node import Node
from pydub import AudioSegment
import sounddevice as sd
import numpy as np
import threading

class Mp3AudioPlayer(Node):
    def __init__(self):
        super().__init__('mp3_audio_player')

        # 声明参数（MP3 文件路径）
        self.declare_parameter('file', '/home/agilex/ros2_project/piper_dancer_ws/src/2025-1104-测试+音乐.mp3')
        file_path = self.get_parameter('file').get_parameter_value().string_value
        self.get_logger().info(f"Loading MP3 file: {file_path}")

        # 读取音频文件（自动解码 MP3）
        audio = AudioSegment.from_file(file_path, format="mp3")

        # 转换为 numpy 数组
        samples = np.array(audio.get_array_of_samples()).astype(np.float32)
        if audio.channels == 2:
            samples = samples.reshape((-1, 2))  # 立体声
        else:
            samples = samples.reshape((-1, 1))

        # 归一化到 [-1.0, 1.0]
        self.audio_data = samples / (2**15)
        self.sample_rate = audio.frame_rate

        self.get_logger().info(f"Loaded MP3: {audio.frame_rate} Hz, {audio.channels} ch, {len(samples)} samples")

        # 播放线程
        self.play_thread = threading.Thread(target=self.play_audio)
        self.play_thread.start()

    def play_audio(self):
        self.get_logger().info("Playing audio...")
        sd.play(self.audio_data, self.sample_rate)
        sd.wait()
        self.get_logger().info("Playback finished.")
        rclpy.shutdown()

def main(args=None):
    rclpy.init(args=args)
    node = Mp3AudioPlayer()
    rclpy.spin(node)

if __name__ == '__main__':
    main()
