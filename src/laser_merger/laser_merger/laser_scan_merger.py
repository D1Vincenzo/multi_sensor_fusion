#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
import numpy as np
from message_filters import ApproximateTimeSynchronizer, Subscriber

class LaserScanMerger(Node):
    def __init__(self):
        super().__init__('laser_scan_merger')
        self.lidar_sub = Subscriber(self, LaserScan, '/lidarscan')
        self.depth_sub = Subscriber(self, LaserScan, '/dep2scan')
        
        self.ats = ApproximateTimeSynchronizer([self.lidar_sub, self.depth_sub], queue_size=5, slop=0.1, allow_headerless=True)
        self.ats.registerCallback(self.merge_callback)

        self.publisher = self.create_publisher(LaserScan, '/scan', 10)

    def adjust_for_translation(scan, translation_x):
        adjusted_ranges = []
        for angle, range_val in zip(np.arange(scan.angle_min, scan.angle_max, scan.angle_increment), scan.ranges):
            # Calculate the original x and y coordinates
            original_x = range_val * np.cos(angle)
            original_y = range_val * np.sin(angle)
            
            # Adjust the x coordinate by the known translation
            adjusted_x = original_x + translation_x
            
            # Recalculate the range based on the adjusted coordinates
            adjusted_range = np.sqrt(adjusted_x**2 + original_y**2)
            adjusted_ranges.append(adjusted_range)
        
        return adjusted_ranges


    def resample_scan(self, original_scan, target_scan):
        #这里可以改成lidar_scan而不是target
        # Calculate the original angles and target angles
        self.print_scan_data(original_scan, 'Original Scan')
        original_angles = np.arange(original_scan.angle_min, original_scan.angle_max, original_scan.angle_increment)
        target_angles = np.arange(target_scan.angle_min, target_scan.angle_max, target_scan.angle_increment)

        # Interpolate the ranges for the target angles
        # This handles extrapolation by setting the values outside the original range to NaN
        interpolated_ranges = np.interp(target_angles, original_angles, original_scan.ranges, left=np.nan, right=np.nan)

        # Create a new LaserScan message for the resampled scan
        new_scan = LaserScan()
        new_scan.header = original_scan.header
        new_scan.angle_min = target_scan.angle_min
        new_scan.angle_max = target_scan.angle_max
        new_scan.angle_increment = target_scan.angle_increment
        new_scan.time_increment = original_scan.time_increment
        new_scan.scan_time = original_scan.scan_time
        new_scan.range_min = original_scan.range_min
        new_scan.range_max = original_scan.range_max
        new_scan.ranges = interpolated_ranges.tolist()  # Convert numpy array back to list

        return new_scan
    
    # def resample_scan(self, original_scan, target_angle_increment, target_angle_min, target_angle_max):
    #     # Calculate original angles and target angles
    #     original_angles = np.arange(original_scan.angle_min, original_scan.angle_max, original_scan.angle_increment)
    #     target_angles = np.arange(target_angle_min, target_angle_max, target_angle_increment)

    #     # Ensure that the range values are in a numpy array for interpolation
    #     original_ranges = np.array(original_scan.ranges)

    #     # Interpolate the ranges for the target angles
    #     # This handles extrapolation by repeating the closest value instead of setting it to NaN
    #     interpolated_ranges = np.interp(target_angles, original_angles, original_ranges,
    #                                     left=original_ranges[0], right=original_ranges[-1])
        
    #     # Create a new LaserScan message for the resampled scan
    #     new_scan = LaserScan()
    #     new_scan.header = original_scan.header
    #     new_scan.angle_min = target_angle_min
    #     new_scan.angle_max = target_angle_max
    #     new_scan.angle_increment = target_angle_increment
    #     new_scan.time_increment = original_scan.time_increment
    #     new_scan.scan_time = original_scan.scan_time
    #     new_scan.range_min = original_scan.range_min
    #     new_scan.range_max = original_scan.range_max
    #     new_scan.ranges = interpolated_ranges.tolist()  # Convert numpy array back to list

    #     return new_scan

    
    def print_scan_data(self, scan, scan_name):
        # Print basic info about the scan
        # self.get_logger().info(f'{scan_name} - Min Angle: {scan.angle_min}, Max Angle: {scan.angle_max}, Angle Increment: {scan.angle_increment}, Min Range: {scan.range_min}, Max Range: {scan.range_max}')

        # Print the first 10 range values as a sample
        range_samples = scan.ranges[:10]  # Adjust this slice as needed
        # self.get_logger().info(f'{scan_name} - Sample Ranges: {range_samples}')

    def merge_callback(self, lidar_msg, depth_msg):
        # Resample /dep2scan to match the angular resolution of /lidarscan
        resampled_depth_scan = self.resample_scan(depth_msg, lidar_msg.angle_increment, lidar_msg.angle_min, lidar_msg.angle_max)
        # resampled_depth_scan = depth_msg
        # Now both scans have the same angular resolution, we can merge them
        merged_ranges = [min(l_range, d_range) for l_range, d_range in zip(lidar_msg.ranges, resampled_depth_scan.ranges)]
        

        merged_scan = LaserScan()
        merged_scan.header = lidar_msg.header
        merged_scan.angle_min = lidar_msg.angle_min
        merged_scan.angle_max = lidar_msg.angle_max
        merged_scan.angle_increment = lidar_msg.angle_increment
        merged_scan.time_increment = lidar_msg.time_increment
        merged_scan.scan_time = lidar_msg.scan_time
        merged_scan.range_min = min(lidar_msg.range_min, depth_msg.range_min)
        merged_scan.range_max = max(lidar_msg.range_max, depth_msg.range_max)
        merged_scan.ranges = merged_ranges
        
        self.publisher.publish(merged_scan)
        # self.publisher.publish(lidar_msg)
        # self.get_logger().info('Published')



def main(args=None):
    rclpy.init(args=args)
    node = LaserScanMerger()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
