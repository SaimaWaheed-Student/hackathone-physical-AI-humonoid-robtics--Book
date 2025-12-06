import rclpy
from rclpy.node import Node
from nav_msgs.msg import Path
from geometry_msgs.msg import PoseStamped

class HumanoidPlanner(Node):
    def __init__(self):
        super().__init__('humanoid_planner_node')
        self.publisher_ = self.create_publisher(Path, 'plan', 10)
        self.timer = self.create_timer(1.0, self.timer_callback)
        self.get_logger().info('Humanoid Path Planner Node (Placeholder) started. Publishing dummy path.')

    def timer_callback(self):
        msg = Path()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'map' # Assuming a 'map' frame

        # Create a simple straight path as a placeholder
        pose = PoseStamped()
        pose.header.stamp = self.get_clock().now().to_msg()
        pose.header.frame_id = 'map'
        pose.pose.position.x = 0.0
        pose.pose.position.y = 0.0
        pose.pose.orientation.w = 1.0
        msg.poses.append(pose)

        pose = PoseStamped()
        pose.header.stamp = self.get_clock().now().to_msg()
        pose.header.frame_id = 'map'
        pose.pose.position.x = 1.0
        pose.pose.position.y = 0.0
        pose.pose.orientation.w = 1.0
        msg.poses.append(pose)
        
        self.publisher_.publish(msg)
        self.get_logger().info('Publishing dummy path.')

def main(args=None):
    rclpy.init(args=args)
    node = HumanoidPlanner()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
