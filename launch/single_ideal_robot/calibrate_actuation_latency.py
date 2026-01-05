#!/usr/bin/env python3

import numpy as np
import matplotlib.pyplot as plt
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from rclpy.node import Node
from scipy import signal
from scipy.signal import find_peaks

import rclpy


class ActuationLatencyCalibrator(Node):
    def __init__(self):
        super().__init__('actuation_latency_calibrator')

        # Publishers and subscribers
        self.cmd_pub = self.create_publisher(Twist, '/robot0/cmd_vel', 10)
        self.odom_sub = self.create_subscription(Odometry, '/robot0/odom', self.odom_callback, 10)

        # Data recording
        self.cmd_times = []
        self.cmd_velocities = []
        self.odom_times = []
        self.odom_velocities = []

        # Sine wave parameters
        self.frequency = 0.05  # Hz
        self.amplitude = 1.5  # m/s
        self.duration = 60.0  # seconds
        self.start_time = None
        self.recording = False
        self.setup_complete = False
        self.setup_wait_seconds = 30
        self.setup_counter = 0

        # Timer for sending commands at nav's rate (uses sim time if use_sim_time=true)
        self.timer = self.create_timer(0.025, self.command_callback)
        # Setup countdown timer (1 second interval)
        self.setup_timer = self.create_timer(1.0, self.setup_countdown_callback)

        self.get_logger().info('Actuation Latency Calibrator started')
        self.get_logger().info(f'Will run for {self.duration} seconds with {self.frequency} Hz sine wave')
        self.get_logger().info(f'Waiting {self.setup_wait_seconds} seconds for setup...')

    def get_ros_time_sec(self):
        """Get current time in seconds (respects use_sim_time)"""
        return self.get_clock().now().nanoseconds / 1e9

    def setup_countdown_callback(self):
        """Called every second during setup countdown"""
        if self.setup_complete:
            return

        self.setup_counter += 1
        remaining = self.setup_wait_seconds - self.setup_counter

        if remaining > 0:
            self.get_logger().info(f'Waiting {self.setup_counter}... ({remaining}s remaining)')
        else:
            self.setup_complete = True
            self.start_time = self.get_ros_time_sec()
            self.recording = True
            self.get_logger().info('Started recording!')
            self.setup_timer.cancel()

    def odom_callback(self, msg):
        if not self.recording:
            return

        current_time = self.get_ros_time_sec() - self.start_time
        if current_time <= self.duration:
            self.odom_times.append(current_time)
            # Record linear velocity in x direction
            self.odom_velocities.append(msg.twist.twist.linear.x)
    
    def command_callback(self):
        if not self.setup_complete:
            return

        current_time = self.get_ros_time_sec() - self.start_time

        if current_time > self.duration:
            if self.recording:
                self.recording = False
                self.get_logger().info('Recording finished. Processing data...')
                self.process_and_plot()
                rclpy.shutdown()
            return

        # Generate sine wave command
        cmd_vel = self.amplitude * np.sin(2 * np.pi * self.frequency * current_time)

        # Record commanded velocity
        self.cmd_times.append(current_time)
        self.cmd_velocities.append(cmd_vel)

        # Publish command
        msg = Twist()
        msg.linear.x = cmd_vel
        msg.linear.y = 0.0
        msg.angular.z = 0.0
        self.cmd_pub.publish(msg)
    
    def process_and_plot(self):
        if len(self.cmd_times) == 0 or len(self.odom_times) == 0:
            self.get_logger().error('No data recorded!')
            return

        self.get_logger().info(f'Recorded {len(self.cmd_times)} commands and {len(self.odom_times)} odometry messages')

        # Convert to numpy arrays
        cmd_times = np.array(self.cmd_times)
        cmd_velocities = np.array(self.cmd_velocities)
        odom_times = np.array(self.odom_times)
        odom_velocities = np.array(self.odom_velocities)

        # Interpolate to common time base (use odometry times as reference)
        cmd_velocities_interp = np.interp(odom_times, cmd_times, cmd_velocities)

        # Calculate cross-correlation to find phase lag
        correlation = signal.correlate(odom_velocities, cmd_velocities_interp, mode='full')
        lags = signal.correlation_lags(len(odom_velocities), len(cmd_velocities_interp), mode='full')

        # Find the lag that maximizes correlation
        max_corr_idx = np.argmax(correlation)
        lag_samples = lags[max_corr_idx]

        # Calculate time lag in seconds
        dt = np.mean(np.diff(odom_times))
        time_lag = lag_samples * dt

        # Calculate phase difference in degrees
        phase_diff_deg = (time_lag * self.frequency * 360.0) % 360

        self.get_logger().info(f'Actuation Latency: {time_lag*1000:.3f} ms')
        self.get_logger().info(f'Phase Difference: {phase_diff_deg:.2f} degrees')

        # Plot results
        plt.figure(figsize=(16, 10))

        plt.subplot(3, 1, 1)
        plt.plot(cmd_times, cmd_velocities, 'b-', label='Commanded Velocity', alpha=0.7, linewidth=1.5)
        plt.plot(odom_times, odom_velocities, 'r-', label='Measured Velocity (Odom)', alpha=0.7, linewidth=1.5)
        plt.xlabel('Time [s]')
        plt.ylabel('Linear Velocity [m/s]')
        plt.title(f'Actuation Latency: {time_lag*1000:.3f} ms | Phase Diff: {phase_diff_deg:.2f}°',
                  fontsize=14, fontweight='bold')
        plt.legend()
        plt.grid(True, alpha=0.3)

        # Zoomed view of first few cycles with peak markers
        plt.subplot(3, 1, 2)
        zoom_duration = min(10.0, self.duration / 3)
        zoom_mask_cmd = cmd_times <= zoom_duration
        zoom_mask_odom = odom_times <= zoom_duration

        zoom_cmd_times = cmd_times[zoom_mask_cmd]
        zoom_cmd_velocities = cmd_velocities[zoom_mask_cmd]
        zoom_odom_times = odom_times[zoom_mask_odom]
        zoom_odom_velocities = odom_velocities[zoom_mask_odom]

        plt.plot(zoom_cmd_times, zoom_cmd_velocities,
                'b-', label='Commanded Velocity', alpha=0.7, linewidth=2)
        plt.plot(zoom_odom_times, zoom_odom_velocities,
                'r-', label='Measured Velocity (Odom)', alpha=0.7, linewidth=2)

        # Find and mark peaks for commanded velocity
        cmd_peaks, _ = find_peaks(zoom_cmd_velocities, height=0.5, distance=len(zoom_cmd_times)//8)
        if len(cmd_peaks) > 0:
            plt.plot(zoom_cmd_times[cmd_peaks], zoom_cmd_velocities[cmd_peaks],
                    'bo', markersize=8, label='Command Peaks')

        # Find and mark peaks for measured velocity
        odom_peaks, _ = find_peaks(zoom_odom_velocities, height=0.3, distance=len(zoom_odom_times)//8)
        if len(odom_peaks) > 0:
            plt.plot(zoom_odom_times[odom_peaks], zoom_odom_velocities[odom_peaks],
                    'ro', markersize=8, label='Measured Peaks')

        plt.xlabel('Time [s]')
        plt.ylabel('Linear Velocity [m/s]')
        plt.title(f'Zoomed View (First {zoom_duration:.1f}s) with Peak Markers', fontsize=12)
        plt.legend()
        plt.grid(True, alpha=0.3)

        # Compensated plot - shift measured curve backward by latency
        plt.subplot(3, 1, 3)

        # Shift the measured curve backward by the calculated time lag
        # (subtract time_lag to move measured curve earlier in time)
        compensated_odom_times = odom_times - time_lag

        # Interpolate the compensated measured data to match commanded time points
        compensated_velocities = np.interp(cmd_times, compensated_odom_times, odom_velocities)

        plt.plot(cmd_times, cmd_velocities, 'b-', label='Commanded Velocity', alpha=0.7, linewidth=1.5)
        plt.plot(cmd_times, compensated_velocities, 'g-', label='Compensated Measured', alpha=0.7, linewidth=1.5)

        # Calculate correlation between commanded and compensated measured
        correlation = np.corrcoef(cmd_velocities, compensated_velocities)[0, 1]

        plt.xlabel('Time [s]')
        plt.ylabel('Linear Velocity [m/s]')
        plt.title(f'Compensated Plot (Latency Removed) | Correlation: {correlation:.4f}',
                  fontsize=12, fontweight='bold')
        plt.legend()
        plt.grid(True, alpha=0.3)
        
        plt.tight_layout()
        plt.savefig('/tmp/actuation_latency_calibration.png', dpi=150, bbox_inches='tight')
        self.get_logger().info('Plot saved to /tmp/actuation_latency_calibration.png')
        plt.show()


def main(args=None):
    rclpy.init(args=args)
    calibrator = ActuationLatencyCalibrator()
    
    try:
        rclpy.spin(calibrator)
    except KeyboardInterrupt:
        pass
    finally:
        # Stop the robot
        msg = Twist()
        calibrator.cmd_pub.publish(msg)
        calibrator.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
