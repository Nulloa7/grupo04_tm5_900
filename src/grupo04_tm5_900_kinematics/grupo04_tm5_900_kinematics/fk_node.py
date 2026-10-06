import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState

import numpy as np


def dh_matrix(theta, d, a, alpha):
    ct = np.cos(theta)
    st = np.sin(theta)
    ca = np.cos(alpha)
    sa = np.sin(alpha)

    return np.array([
        [ct, -st * ca,  st * sa, a * ct],
        [st,  ct * ca, -ct * sa, a * st],
        [0.0,      sa,       ca,      d],
        [0.0,     0.0,      0.0,    1.0]
    ])


class FKNode(Node):

    def __init__(self):
        super().__init__('fk_node')

        self.subscription = self.create_subscription(
            JointState,
            '/joint_states',
            self.joint_callback,
            10
        )

        self.get_logger().info('Nodo FK iniciado. Esperando /joint_states...')

    def joint_callback(self, msg):

        if len(msg.position) < 6:
            return

        q1, q2, q3, q4, q5, q6 = msg.position[:6]

        pi_2 = np.pi / 2.0

        dh_params = [
            (q1,         0.1452,  0.0,    -pi_2),
            (q2-pi_2,    0.0,     0.429,   0.0),
            (q3,         0.1223,  0.4115,  0.0),
            (q4+pi_2,    0.106,   0.0,     pi_2),
            (q5,         0.11315, 0.0,    -pi_2),
            (q6,         0.0,     0.0,     0.0)
        ]

        T = np.eye(4)

        for theta, d, a, alpha in dh_params:
            T = T @ dh_matrix(theta, d, a, alpha)

        x = T[0, 3]
        y = T[1, 3]
        z = T[2, 3]

        self.get_logger().info(
            f'Posición calculada FK: x={x:.4f}, y={y:.4f}, z={z:.4f}'
        )


def main(args=None):
    rclpy.init(args=args)

    node = FKNode()

    rclpy.spin(node)

    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
