import rclpy
from rclpy.node import Node
from nav_msgs.msg import OccupancyGrid, Odometry
from nav2_msgs.action import NavigateToPose
import numpy as np
from collections import deque
from geometry_msgs.msg import PoseStamped
from rclpy.action import ActionClient
from rclpy.qos import QoSProfile, DurabilityPolicy, ReliabilityPolicy

class FrontierExplorer(Node):

    def __init__(self):
        super().__init__('frontier_explorer')
        self.map_sub = self.create_subscription(
            OccupancyGrid,
            '/map',
            self.map_callback,
            10)
        #Altering the QoS of map topic published by frontier_explorer
        map_qos = QoSProfile(
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL
        )

        self.map_sub = self.create_subscription(
            OccupancyGrid, '/map', self.map_callback, map_qos)

        self.odom_sub = self.create_subscription(
        Odometry, '/odom', self.odom_callback, 10)
        self.nav_client = ActionClient(self, NavigateToPose, 'navigate_to_pose')
        self.map_data = None
        self.robot_x = 0.0
        self.robot_y = 0.0
        self.exploring = False

    #To get the positions/pose
    def odom_callback(self, msg):
        self.robot_x = msg.pose.pose.position.x
        self.robot_y = msg.pose.pose.position.y

    def map_callback(self, msg):
        self.map_data = msg
        width = msg.info.width
        height = msg.info.height
        data = np.array(msg.data).reshape((height, width))

        frontiers = []
        for y in range(1, height - 1):
            for x in range(1, width - 1):
                if data[y, x] == 0:
                    neighbours = [
                        data[y+1, x],
                        data[y-1, x],
                        data[y, x+1],
                        data[y, x-1]
                    ]
                    if -1 in neighbours:
                        frontiers.append((x, y))
        #Prints all undiscovered areas/frontier cells
        self.get_logger().info(f'Found {len(frontiers)} frontier cells')

        #Cluster them as groups 
        clusters = self.cluster_frontiers(frontiers)
        self.get_logger().info(f'Found {len(clusters)} frontier clusters')
        # goal = self.get_best_frontier(clusters, msg.info)
        # if goal:
        #     self.get_logger().info(f'Best frontier at world coords: {goal[0]:.2f}, {goal[1]:.2f}')
        # goal = self.get_best_frontier(clusters, msg.info)
        # if goal:
        #     self.send_goal(goal[0], goal[1])
        goal = self.get_best_frontier(clusters, msg.info)
        if goal and not self.exploring:
            self.exploring = True
            self.send_goal(goal[0], goal[1])
                    
    def cluster_frontiers(self, frontiers, min_cluster_size=10):
        visited = set()
        clusters = []
        frontier_set = set(frontiers)

        for cell in frontiers:
            if cell in visited:
                continue
            cluster = []
            queue = deque([cell])
            while queue:
                cx, cy = queue.popleft()
                if (cx, cy) in visited:
                    continue
                visited.add((cx, cy))
                cluster.append((cx, cy))
                for nx, ny in [(cx+1, cy), (cx-1, cy), (cx, cy+1), (cx, cy-1)]:
                    if (nx, ny) in frontier_set and (nx, ny) not in visited:
                        queue.append((nx, ny))
            if len(cluster) >= min_cluster_size:
                clusters.append(cluster)

        return clusters

    def get_best_frontier(self, clusters, map_info):
        best = None
        best_dist = float('inf')

        for cluster in clusters:
            # centroid in cell coordinates
            cx = sum(c[0] for c in cluster) / len(cluster)
            cy = sum(c[1] for c in cluster) / len(cluster)

            # convert cell coordinates to world coordinates
            wx = map_info.origin.position.x + cx * map_info.resolution
            wy = map_info.origin.position.y + cy * map_info.resolution

            dist = ((wx - self.robot_x)**2 + (wy - self.robot_y)**2)**0.5

            if dist < best_dist:
                best_dist = dist
                best = (wx, wy)

        return best
    
    def send_goal(self, x, y):
        goal_msg = NavigateToPose.Goal()
        goal_msg.pose.header.frame_id = 'map'
        goal_msg.pose.header.stamp = self.get_clock().now().to_msg()
        goal_msg.pose.pose.position.x = x
        goal_msg.pose.pose.position.y = y
        goal_msg.pose.pose.orientation.w = 1.0

        self.nav_client.wait_for_server()
        future = self.nav_client.send_goal_async(goal_msg)
        future.add_done_callback(self.goal_accepted_callback)
        self.get_logger().info(f'Sent goal: {x:.2f}, {y:.2f}')

    def goal_accepted_callback(self, future):
        goal_handle = future.result()
        result_future = goal_handle.get_result_async()
        result_future.add_done_callback(self.goal_result_callback)

    def goal_result_callback(self, future):
        self.get_logger().info('Goal reached, finding next frontier')
        self.exploring = False
        
def main(args=None):
    rclpy.init(args=args)
    node = FrontierExplorer()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()