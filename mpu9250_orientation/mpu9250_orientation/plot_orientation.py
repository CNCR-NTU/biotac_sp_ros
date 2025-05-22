import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32MultiArray

import matplotlib.pyplot as plt
import numpy as np
from collections import deque
import time


class OrientationPlotter(Node):
    def __init__(self):
        super().__init__('orientation_plotter')
        self.subscription = self.create_subscription(
            Float32MultiArray,
            'mpu9250/orientation',
            self.listener_callback,
            10
        )

        self.buffer_size = 200
        self.pitch_buffer = deque(maxlen=self.buffer_size)
        self.roll_buffer = deque(maxlen=self.buffer_size)
        self.yaw_buffer = deque(maxlen=self.buffer_size)
        self.time_buffer = deque(maxlen=self.buffer_size)
        self.start_time = time.time()

        # Setup matplotlib
        plt.ion()
        self.fig, self.axs = plt.subplots(3, 1, figsize=(10, 8))
        self.lines = []

        for ax, label in zip(self.axs, ['Pitch', 'Roll', 'Yaw']):
            ax.set_title(label)
            ax.set_xlabel('Time (s)')
            ax.set_ylabel('Degrees')
            line, = ax.plot([], [], label=label)
            ax.legend()
            self.lines.append(line)

        # Setup timer to update plot every 100 ms
        self.create_timer(0.1, self.update_plot)

    def listener_callback(self, msg: Float32MultiArray):
        now = time.time() - self.start_time
        pitch, roll, yaw = msg.data

        self.pitch_buffer.append(pitch)
        self.roll_buffer.append(roll)
        self.yaw_buffer.append(yaw)
        self.time_buffer.append(now)

    def update_plot(self):
        if len(self.time_buffer) < 2:
            return

        for i, data in enumerate([self.pitch_buffer, self.roll_buffer, self.yaw_buffer]):
            self.lines[i].set_xdata(self.time_buffer)
            self.lines[i].set_ydata(data)
            self.axs[i].relim()
            self.axs[i].autoscale_view()

        self.fig.canvas.draw()
        self.fig.canvas.flush_events()


def main(args=None):
    rclpy.init(args=args)
    node = OrientationPlotter()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
