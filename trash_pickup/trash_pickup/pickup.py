"""Pick up the box directly in front of Stretch's arm with hardcoded joint values.

For each step: send a goal to /stretch_controller/follow_joint_trajectory,
then wait (on /stretch/joint_states) until that joint stops moving.
"""
import rclpy
from control_msgs.action import FollowJointTrajectory
from rclpy.action import ActionClient
from rclpy.node import Node
from rclpy.parameter import Parameter
from sensor_msgs.msg import JointState
from trajectory_msgs.msg import JointTrajectoryPoint

# (joint name, position). Lift/arm in meters, gripper: 0.5 = open, -0.376 = fully closed
STEPS = [
    ('joint_arm', 0.0),                     # retract the arm
    ('joint_gripper_finger_left', 0.5),     # open the gripper
    ('joint_lift', 0.42),                   # lower to object height
    ('joint_arm', 0.14),                    # reach out around the object
    ('joint_gripper_finger_left', -0.376),  # close the gripper
    ('joint_lift', 0.8),                    # lift the object
    ('joint_arm', 0.0),                     # pull it back in
]


class Pickup(Node):
    def __init__(self):
        # Use the sim's clock: the simulation can run slower than real time
        super().__init__('pickup', parameter_overrides=[Parameter('use_sim_time', value=True)])
        self.client = ActionClient(self, FollowJointTrajectory,
                                   '/stretch_controller/follow_joint_trajectory')
        self.joints = {}
        self.create_subscription(JointState, '/stretch/joint_states', self.on_joints, 10)

    def on_joints(self, msg):
        self.joints = dict(zip(msg.name, msg.position))

    def position(self, joint):
        if joint == 'joint_arm':  # the arm is 4 telescoping segments
            return sum(self.joints[f'joint_arm_l{i}'] for i in range(4))
        return self.joints[joint]

    def wait(self, seconds):
        end = self.get_clock().now().nanoseconds + seconds * 1e9
        while self.get_clock().now().nanoseconds < end:
            rclpy.spin_once(self, timeout_sec=0.1)

    def move(self, joint, position):
        goal = FollowJointTrajectory.Goal()
        goal.trajectory.joint_names = [joint]
        goal.trajectory.points = [JointTrajectoryPoint(positions=[position])]
        # Wait for the driver to report the step finished, so steps never overlap.
        # The driver finishes once the joint is within 0.05 of the target, or after
        # a 5 s timeout if something (e.g. the object in the gripper) blocks it.
        send = self.client.send_goal_async(goal)
        rclpy.spin_until_future_complete(self, send)
        handle = send.result()
        if not handle.accepted:
            self.get_logger().error(f'{joint} goal rejected')
            return
        rclpy.spin_until_future_complete(self, handle.get_result_async())
        self.wait(0.5)  # let the latest joint state arrive
        self.get_logger().info(f'{joint} stopped at {self.position(joint):.4f}')

    def run(self):
        self.get_logger().info('Waiting for the Stretch driver...')
        self.client.wait_for_server()
        while not self.joints:
            rclpy.spin_once(self, timeout_sec=0.1)
        for joint, position in STEPS:
            self.get_logger().info(f'{joint} -> {position}')
            self.move(joint, position)
        self.get_logger().info('Done, object picked up!')


def main():
    rclpy.init()
    Pickup().run()
    rclpy.shutdown()
