import rclpy
from rclpy.node import Node
from nav_msgs.msg import OccupancyGrid
from geometry_msgs.msg import PoseStamped
from nav2_msgs.action import NavigateToPose
from rclpy.action import ActionClient
import numpy as np


class FrontierExplorer(Node):

    def __init__(self, ):
        super().__init__('front_explorer')
        

        self.map_sub = self.create_subscription(
            OccupancyGrid,
            '/map',
            self.map_callback,
            10
        )

        self.nav_client = ActionClient(
            self,
            NavigateToPose,
            '/navigate_to_pose'
        )

        self.map = None
        self.busy = False

        self.timer = self.create_timer(3.0, self.explore)

    def map_callback(self, msg):
        self.map = msg

    def explore(self):
        if self.map is None or self.busy:
            return

        frontiers = self.find_frontiers(self.map)

        if not frontiers:
            self.get_logger().info('Exploration finished')
            return

        target = frontiers[0]
        self.send_goal(target)

    def send_goal(self, cell):
        self.busy = True

        y, x = cell
        res = self.map.info.resolution
        ox = self.map.info.origin.position.x
        oy = self.map.info.origin.position.y

        goal = PoseStamped()
        goal.header.frame_id = 'map'
        goal.pose.position.x = ox + (x + 0.5) * res
        goal.pose.position.y = oy + (y + 0.5) * res
        goal.pose.orientation.w = 1.0

        nav_goal = NavigateToPose.Goal()
        nav_goal.pose = goal

        self.nav_client.wait_for_server()
        future = self.nav_client.send_goal_async(nav_goal)
        future.add_done_callback(self.goal_response)

    def goal_response(self, future):
        goal_handle = future.result()

        if not goal_handle.accepted:
            self.get_logger().warn( 'Goal rejected')
            self.busy = False
            return

        result_future = goal_handle.get_result_async()
        result_future.add_done_callback(self.goal_result)

    def goal_result(self, future):
        status = future.result().status

        if status == 4:  # SUCCEEDED
            self.get_logger().info('Goal reached')
        else:
            self.get_logger().warn('Goal failed, retrying')

        self.busy = False

    def find_frontiers(self, map_msg):
        data = np.array(map_msg.data).reshape(
            map_msg.info.height,
            map_msg.info.width
        )

        frontiers = []
        for y in range(1, map_msg.info.height - 1):
            for x in range(1, map_msg.info.width - 1):
                if data[y, x] == 0:
                    if -1 in data[y-1:y+2, x-1:x+2]:
                        frontiers.append((y, x))

        return frontiers


def main():
    rclpy.init()
    nodes = [
        FrontierExplorer()
       
    ]
    executor = rclpy.executors.MultiThreadedExecutor()
    for n in nodes:
        executor.add_node(n)
    executor.spin()
    rclpy.shutdown()
