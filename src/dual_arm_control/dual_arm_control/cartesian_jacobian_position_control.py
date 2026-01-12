import rclpy
from rclpy.node import Node

import numpy as np

from sensor_msgs.msg import JointState
from std_msgs.msg import Float64MultiArray


class DualArmCartesianJacobianPositionController(Node):

    def __init__(self):
        super().__init__('dual_arm_cartesian_jacobian_position_controller')

        # ---------------- Publishers ----------------
        self.left_pub = self.create_publisher(
            Float64MultiArray,
            '/left_arm_position_controller/commands',
            10
        )

        self.right_pub = self.create_publisher(
            Float64MultiArray,
            '/right_arm_position_controller/commands',
            10
        )

        # ---------------- Joint States ----------------
        self.create_subscription(
            JointState,
            '/joint_states',
            self.joint_state_cb,
            10
        )

        self.left_q = None
        self.right_q = None

        self.dt = 0.05

        
        self.x_dot_shared = np.array([0.02, 0.0, 0.0])  # +X together

        self.timer = self.create_timer(self.dt, self.control_loop)

        self.get_logger().info(
            "SYNCHRONIZED Dual-arm Jacobian Cartesian Position Controller started"
        )

    # ------------------------------------------------
    def joint_state_cb(self, msg: JointState):
        names = msg.name
        pos = msg.position

        try:
            self.left_q = np.array([
                pos[names.index('left_joint_1')],
                pos[names.index('left_joint_2')],
                pos[names.index('left_joint_3')],
                pos[names.index('left_joint_4')],
                pos[names.index('left_joint_5')],
                pos[names.index('left_joint_6')],
            ])

            self.right_q = np.array([
                pos[names.index('right_joint_1')],
                pos[names.index('right_joint_2')],
                pos[names.index('right_joint_3')],
                pos[names.index('right_joint_4')],
                pos[names.index('right_joint_5')],
                pos[names.index('right_joint_6')],
            ])
        except ValueError:
            return

    # ------------------------------------------------
    def forward_kinematics(self, q):
        """
        SAME simplified FK (demo)
        """
        x = 0.3 * np.cos(q[0])
        y = 0.3 * np.sin(q[1])
        z = 0.2 + 0.1 * q[2]
        return np.array([x, y, z])

    # ------------------------------------------------
    def compute_jacobian(self, q):
        eps = 1e-4
        J = np.zeros((3, 6))

        f0 = self.forward_kinematics(q)

        for i in range(6):
            dq = np.zeros(6)
            dq[i] = eps
            f1 = self.forward_kinematics(q + dq)
            J[:, i] = (f1 - f0) / eps

        return J

    # ------------------------------------------------
    def control_loop(self):
        if self.left_q is None or self.right_q is None:
            return

        # --- Compute Jacobians ---
        J_left = self.compute_jacobian(self.left_q)
        J_right = self.compute_jacobian(self.right_q)

       
        J_stack = np.block([
            [J_left,  np.zeros((3, 6))],
            [np.zeros((3, 6)), J_right]
        ])

        # --- SHARED TASK ---
        x_dot_stack = np.hstack([
            self.x_dot_shared,
            self.x_dot_shared
        ])

        # --- Solve once (synchronized) ---
        q_dot_stack = np.linalg.pinv(J_stack) @ x_dot_stack

        q_dot_left = q_dot_stack[:6]
        q_dot_right = q_dot_stack[6:]

        q_left_new = self.left_q + q_dot_left * self.dt
        q_right_new = self.right_q + q_dot_right * self.dt

        # --- Publish together ---
        left_cmd = Float64MultiArray()
        left_cmd.data = q_left_new.tolist()

        right_cmd = Float64MultiArray()
        right_cmd.data = q_right_new.tolist()

        self.left_pub.publish(left_cmd)
        self.right_pub.publish(right_cmd)


def main():
    rclpy.init()
    node = DualArmCartesianJacobianPositionController()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()