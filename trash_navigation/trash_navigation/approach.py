"""Rotate until YOLO sees something, then drive toward it.

Subscribes /trash/target (geometry_msgs/Point from trash_vision)
Publishes  /stretch/cmd_vel (geometry_msgs/Twist)
At startup, tilts the head down via /stretch_controller/follow_joint_trajectory
"""
import rclpy
from control_msgs.action import FollowJointTrajectory
from geometry_msgs.msg import Point, Twist
from rclpy.action import ActionClient
from rclpy.node import Node
from trajectory_msgs.msg import JointTrajectoryPoint


class Approach(Node):
    def __init__(self):
        super().__init__('approach')
        self.declare_parameter('search_speed', 0.3)   # rad/s while searching
        self.declare_parameter('drive_speed', 0.1)    # m/s toward the object
        self.declare_parameter('turn_gain', 0.5)      # steering toward the object
        self.declare_parameter('stop_size', 0.5)      # stop when box is this tall (fraction of image)
        self.declare_parameter('lost_timeout', 1.0)   # s without detection -> search again

        self.target = None
        self.last_seen = None
        self.done = False

        self.pub = self.create_publisher(Twist, '/stretch/cmd_vel', 10)
        self.create_subscription(Point, '/trash/target', self.on_target, 10)
        self.create_timer(0.1, self.tick)  # 10 Hz, or the driver times out

        # Tilt the head camera down once so it can see the table instead of the horizon
        self.declare_parameter('head_tilt', -0.8)  # rad, negative = look down
        self.head = ActionClient(self, FollowJointTrajectory,
                                 '/stretch_controller/follow_joint_trajectory')
        self.head.wait_for_server()
        goal = FollowJointTrajectory.Goal()
        goal.trajectory.joint_names = ['joint_head_pan', 'joint_head_tilt']
        goal.trajectory.points = [JointTrajectoryPoint(
            positions=[0.0, self.get_parameter('head_tilt').value])]
        self.head.send_goal_async(goal)

    def on_target(self, msg):
        self.target = msg
        self.last_seen = self.get_clock().now()

    def tick(self):
        p = lambda n: self.get_parameter(n).value  # noqa: E731
        cmd = Twist()
        now = self.get_clock().now()
        seen = (self.last_seen is not None and
                (now - self.last_seen).nanoseconds / 1e9 < p('lost_timeout'))

        if self.done:
            pass  # stay stopped
        elif not seen:
            cmd.angular.z = p('search_speed')
            self.get_logger().info('Searching...', throttle_duration_sec=2.0)
        elif self.target.y >= p('stop_size'):
            self.done = True
            self.get_logger().info('Reached the object, stopping.')
        else:
            cmd.linear.x = p('drive_speed')
            cmd.angular.z = -p('turn_gain') * self.target.x  # object on right -> turn right
            self.get_logger().info('Driving to object', throttle_duration_sec=2.0)

        self.pub.publish(cmd)


def main():
    rclpy.init()
    rclpy.spin(Approach())
    rclpy.shutdown()
