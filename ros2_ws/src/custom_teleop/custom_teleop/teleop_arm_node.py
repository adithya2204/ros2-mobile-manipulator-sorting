import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64MultiArray
from boeing_gazebo_model_attachment_plugin_msgs.srv import Attach, Detach
import sys
import select
import termios
import tty
import time

class TeleopRobot(Node):
    def __init__(self):
        super().__init__('teleop_robot')

        # Publishers
        self.shoulder_pub = self.create_publisher(Float64MultiArray, '/shoulder_joint_position_controller/commands', 10)
        self.elbow_pub = self.create_publisher(Float64MultiArray, '/elbow_joint_position_controller/commands', 10)
        self.left_gripper_pub = self.create_publisher(Float64MultiArray, '/left_gripper_position_controller/commands', 10)
        self.right_gripper_pub = self.create_publisher(Float64MultiArray, '/right_gripper_position_controller/commands', 10)

        # Services
        self.attach_client = self.create_client(Attach, '/gazebo/attach')
        self.detach_client = self.create_client(Detach, '/gazebo/detach')

        # Cylinder mappings
        self.attach_keys = {'1': 'cube1', '2': 'cube2', '3': 'cube3'}
        self.detach_keys = {'!': 'cube1', '@': 'cube2', '#': 'cube3',}

        self.get_logger().info('Manipulator Teleop Node Started. Press "h" for help.')

    def get_key(self):
        """Non-blocking key press reader"""
        fd = sys.stdin.fileno()
        old_settings = termios.tcgetattr(fd)
        try:
            tty.setraw(fd)
            if select.select([sys.stdin], [], [], 0.1)[0]:
                return sys.stdin.read(1)
            else:
                return ''
        finally:
            termios.tcsetattr(fd, termios.TCSADRAIN, old_settings)

    def clamp(self, value, min_val, max_val):
        return max(min(value, max_val), min_val)

    def control_arm(self, shoulder_pos, elbow_pos, left_gripper_pos, right_gripper_pos):
        self.shoulder_pub.publish(Float64MultiArray(data=[shoulder_pos]))
        self.elbow_pub.publish(Float64MultiArray(data=[elbow_pos]))
        self.left_gripper_pub.publish(Float64MultiArray(data=[left_gripper_pos]))
        self.right_gripper_pub.publish(Float64MultiArray(data=[right_gripper_pos]))

    def attach_cylinder(self, cube_name):
        req = Attach.Request()
        req.joint_name = 'right_gripper_cylinder_joint'
        req.model_name_1 = 'robot'
        req.link_name_1 = 'right_gripper'
        req.model_name_2 = cube_name
        req.link_name_2 = 'cube'

        while not self.attach_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Waiting for Attach service...')

        future = self.attach_client.call_async(req)
        rclpy.spin_until_future_complete(self, future)

        if future.result() is not None:
            self.get_logger().info(f'{cube_name} Attached Successfully!')
        else:
            self.get_logger().error(f'Failed to Attach {cube_name}.')

    def detach_cylinder(self, cube_name):
        req = Detach.Request()
        req.joint_name = 'right_gripper_cylinder_joint'
        req.model_name_1 = 'robot'
        req.model_name_2 = cube_name

        while not self.detach_client.wait_for_service(timeout_sec=1.0): 
            self.get_logger().info('Waiting for Detach service...')

        future = self.detach_client.call_async(req)
        rclpy.spin_until_future_complete(self, future)

        if future.result() is not None:
            self.get_logger().info(f'{cube_name} Detached Successfully!')
        else:
            self.get_logger().error(f'Failed to Detach {cube_name}.')

    def teleop_loop(self):
        shoulder_pos = -0.6981317
        elbow_pos = -1.0
        left_gripper_pos = 0.2617994
        right_gripper_pos = -0.2617994

        last_command = None

        try:
            while rclpy.ok():
                rclpy.spin_once(self, timeout_sec=0.01)
                key = self.get_key()

                if key == 'q':
                    self.get_logger().info("Exiting...")
                    break
                elif key == 'h':
                    print("\nControls:\n"
                          "  w/s: Shoulder +/−\n"
                          "  e/d: Elbow +/−\n"
                          "  r/f: Gripper Open/Close\n"
                          "  1-3: Attach objects\n"
                          "  ! @ #  Detach objects\n"
                          "  SPACE: Reset all to default\n"
                          "  q: Quit\n")
                elif key == 'w':
                    shoulder_pos += 0.1
                elif key == 's':
                    shoulder_pos -= 0.1
                elif key == 'e':
                    elbow_pos += 0.1
                elif key == 'd':
                    elbow_pos -= 0.1
                elif key == 'r':
                    left_gripper_pos += 0.1
                    right_gripper_pos -= 0.1
                elif key == 'f':
                    left_gripper_pos -= 0.1
                    right_gripper_pos += 0.1
                elif key in self.attach_keys:
                    left_gripper_pos = 0.2617994
                    right_gripper_pos = -0.2617994
                    self.control_arm(shoulder_pos, elbow_pos, left_gripper_pos, right_gripper_pos)
                    time.sleep(1)
                    self.attach_cylinder(self.attach_keys[key])
                elif key in self.detach_keys:
                    left_gripper_pos = 0.5235988
                    right_gripper_pos = -0.5235988
                    self.control_arm(shoulder_pos, elbow_pos, left_gripper_pos, right_gripper_pos)
                    time.sleep(1)
                    self.detach_cylinder(self.detach_keys[key])
                elif key == ' ':
                    shoulder_pos = -0.6981317
                    elbow_pos = -1.0
                    left_gripper_pos = 0.2617994
                    right_gripper_pos = -0.2617994
                    self.get_logger().info("Resetting all joint positions to default")

                shoulder_pos = self.clamp(shoulder_pos, -1.396263, 1.047198)
                elbow_pos = self.clamp(elbow_pos, -2.617994, -1.047)
                left_gripper_pos = self.clamp(left_gripper_pos, 0.0, 1.047)
                right_gripper_pos = self.clamp(right_gripper_pos, -1.047, 0.0)

                current_command = (shoulder_pos, elbow_pos, left_gripper_pos, right_gripper_pos)
                if current_command != last_command:
                    self.control_arm(*current_command)
                    last_command = current_command

        except KeyboardInterrupt:
            self.get_logger().info("Teleop interrupted via Ctrl+C")

def main(args=None):
    rclpy.init(args=args)
    node = TeleopRobot()
    node.teleop_loop()
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
