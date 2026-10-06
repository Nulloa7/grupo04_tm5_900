import rclpy
from rclpy.node import Node

from geometry_msgs.msg import Point
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


def fk(q):
    pi_2 = np.pi / 2.0

    dh_params = [
        (q[0],         0.1452,  0.0,    -pi_2),
        (q[1] - pi_2,  0.0,     0.429,   0.0),
        (q[2],         0.1223,  0.4115,  0.0),
        (q[3] + pi_2,  0.106,   0.0,     pi_2),
        (q[4],         0.11315, 0.0,    -pi_2),
        (q[5],         0.0,     0.0,     0.0)
    ]

    T = np.eye(4)

    for theta, d, a, alpha in dh_params:
        T = T @ dh_matrix(theta, d, a, alpha)

    return T


def numerical_jacobian(q):
    h = 1e-6

    p0 = fk(q)[0:3, 3]

    J = np.zeros((3, 6))

    for i in range(6):
        q_temp = q.copy()
        q_temp[i] += h

        p_temp = fk(q_temp)[0:3, 3]

        J[:, i] = (p_temp - p0) / h

    return J


class IKNode(Node):

    def __init__(self):
        super().__init__('ik_node')

        self.subscription = self.create_subscription(
            Point,
            '/target',
            self.target_callback,
            10
        )

        self.publisher = self.create_publisher(
            JointState,
            '/joint_states',
            10
        )

        self.q_solution = np.zeros(6)

        self.timer = self.create_timer(
            0.1,
            self.publish_joint_states
        )

        self.get_logger().info(
            'Nodo IK con Timer iniciado. Esperando objetivo en /target...'
        )

    def target_callback(self, msg):

        target = np.array([
            msg.x,
            msg.y,
            msg.z
        ])

        self.get_logger().info(
            f'Objetivo recibido: {target}'
        )

        q = np.zeros(6)

        gain = 0.10
        tolerance = 1e-3
        max_iterations = 200

        q_min = np.radians([
            -270,
            -180,
            -155,
            -180,
            -180,
            -270
        ])

        q_max = np.radians([
           270,
           180,
           155,
           180,
           180,
           270
        ])

        for k in range(max_iterations):

            T = fk(q)

            position = T[0:3, 3]

            error = target - position

            error_norm = np.linalg.norm(error)

            if error_norm < tolerance:

                self.q_solution = q.copy()

                self.get_logger().info(
                    f'¡Convergencia en {k} iteraciones! '
                    f'Error: {error_norm:.4f} m'
                )

                self.get_logger().info(
                    f'Solución q: {q}'
                )

                return

            J = numerical_jacobian(q)

            damping = 0.05

            JJT = J @ J.T

            dq = J.T @ np.linalg.solve(
            JJT + (damping ** 2) * np.eye(3),
            error
            )

            q = q + gain * dq

            q = np.clip(q, q_min, q_max)

        self.get_logger().warning(
            'No se alcanzó convergencia dentro del máximo de iteraciones.'
        )

    def publish_joint_states(self):

        msg = JointState()

        msg.header.stamp = self.get_clock().now().to_msg()

        msg.name = [
            'joint_1',
            'joint_2',
            'joint_3',
            'joint_4',
            'joint_5',
            'joint_6'
        ]

        msg.position = self.q_solution.tolist()

        self.publisher.publish(msg)


def main(args=None):

    rclpy.init(args=args)

    node = IKNode()

    rclpy.spin(node)

    node.destroy_node()

    rclpy.shutdown()


if __name__ == '__main__':
    main()
