# gait_analysis_subscriber.py
import rospy
from sensor_msgs.msg import JointState
import numpy as np
import matplotlib.pyplot as plt
from collections import deque

class GaitAnalyzer:
    def __init__(self):
        rospy.init_node('gait_analyzer', anonymous=True)
        self.sub = rospy.Subscriber('/quadruped/joint_states', JointState, self.callback)
        self.joint_positions = {}
        self.history_length = 100  # Number of samples to keep
        self.position_history = {}
        self.time_history = {}
        self.fig, self.ax = plt.subplots()
        plt.ion()
        plt.show()

    def callback(self, data):
        current_time = data.header.stamp.to_sec()
        for name, position in zip(data.name, data.position):
            if name not in self.position_history:
                self.position_history[name] = deque(maxlen=self.history_length)
                self.time_history[name] = deque(maxlen=self.history_length)
            self.position_history[name].append(position)
            self.time_history[name].append(current_time)

        # Example Analysis: Calculate frequency of oscillations for a specific joint
        target_joint = 'joint_0'  # Replace with your joint name
        if target_joint in self.position_history:
            positions = np.array(self.position_history[target_joint])
            times = np.array(self.time_history[target_joint])
            if len(positions) > 10:
                # Detrend the signal
                positions_detrended = positions - np.mean(positions)
                # Compute FFT
                fft_vals = np.fft.fft(positions_detrended)
                fft_freq = np.fft.fftfreq(len(fft_vals), d=(times[-1] - times[0])/len(times))
                positive_freqs = fft_freq[:len(fft_freq)//2]
                magnitudes = np.abs(fft_vals[:len(fft_vals)//2])
                peak_freq = positive_freqs[np.argmax(magnitudes)]
                rospy.loginfo(f"Joint: {target_joint}, Peak Frequency: {peak_freq:.2f} Hz")

        # Optional: Plot joint positions
        self.plot_joint_positions()

    def plot_joint_positions(self):
        self.ax.cla()
        for name, history in self.position_history.items():
            times = list(self.time_history[name])
            positions = list(history)
            self.ax.plot(times, positions, label=name)
        self.ax.legend()
        self.ax.set_xlabel('Time (s)')
        self.ax.set_ylabel('Joint Position (rad)')
        plt.draw()
        plt.pause(0.001)

    def run(self):
        rospy.spin()

if __name__ == '__main__':
    analyzer = GaitAnalyzer()
    analyzer.run()
