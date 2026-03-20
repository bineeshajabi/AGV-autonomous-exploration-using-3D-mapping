#!/usr/bin/env python3

import rclpy
import time
from rclpy.node import Node
from rclpy.action import ActionClient
from geometry_msgs.msg import PoseStamped
from nav2_msgs.action import NavigateToPose
from builtin_interfaces.msg import Duration


class WaypointNavigator(Node):

    def __init__(self):
        super().__init__('waypoint_navigator')

        self._action_client = ActionClient(
            self,
            NavigateToPose,
            'navigate_to_pose'
        )

        # Define 5 waypoints (x, y, yaw)
        self.waypoints = [
            (1.0, 0.0, 0.0),
            (1.0, 1.0, 1.57),
            (0.0, 1.0, 3.14),
            (-1.0, 1.0, -1.57),
            (-1.0, 0.0, 0.0)
        ]

        # Pause times (seconds)
        self.pause_times = [3, 5, 2, 4, 3]

        self.start_time = None
        self.home_pose = None

    def send_goal(self, x, y, yaw):

        goal_msg = NavigateToPose.Goal()
        goal_msg.pose = self.create_pose(x, y, yaw)

        self._action_client.wait_for_server()

        send_goal_future = self._action_client.send_goal_async(goal_msg)
        rclpy.spin_until_future_complete(self, send_goal_future)

        goal_handle = send_goal_future.result()

        if not goal_handle.accepted:
            self.get_logger().error('Goal rejected')
            return False

        result_future = goal_handle.get_result_async()
        rclpy.spin_until_future_complete(self, result_future)

        return True

    def create_pose(self, x, y, yaw):

        pose = PoseStamped()
        pose.header.frame_id = 'map'
        pose.header.stamp = self.get_clock().now().to_msg()

        pose.pose.position.x = x
        pose.pose.position.y = y

        # Convert yaw to quaternion
        import math
        pose.pose.orientation.z = math.sin(yaw / 2.0)
        pose.pose.orientation.w = math.cos(yaw / 2.0)

        return pose

    def run_mission(self):

        self.get_logger().info("Starting waypoint mission...")

        self.start_time = time.time()

        # Save home position
        self.home_pose = self.create_pose(0.0, 0.0, 0.0)

        for i, waypoint in enumerate(self.waypoints):

            x, y, yaw = waypoint
            pause_time = self.pause_times[i]

            self.get_logger().info(
                f"Navigating to waypoint {i+1}: ({x}, {y})"
            )

            waypoint_start = time.time()

            success = self.send_goal(x, y, yaw)

            if not success:
                self.get_logger().error("Navigation failed.")
                return

            arrival_time = time.time()
            travel_time = arrival_time - waypoint_start

            self.get_logger().info(
                f"Arrived at waypoint {i+1}. Travel time: {travel_time:.2f}s"
            )

            self.get_logger().info(
                f"Pausing for {pause_time} seconds..."
            )

            time.sleep(pause_time)

            self.get_logger().info(
                f"Time spent at waypoint {i+1}: {pause_time}s"
            )

        # Return home
        self.get_logger().info("Returning home...")

        self.send_goal(0.0, 0.0, 0.0)

        total_time = time.time() - self.start_time

        self.get_logger().info(
            f"Mission complete! Total route time: {total_time:.2f}s"
        )


def main(args=None):
    rclpy.init(args=args)

    navigator = WaypointNavigator()

    navigator.run_mission()

    navigator.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
