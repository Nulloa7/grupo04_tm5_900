import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Point
from sensor_msgs.msg import JointState
import numpy as np


class IKNode(Node):
    def __init__(self):
        super().__init__('ik_node_pinv')

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

        # Posicion inicial
        self.q_k = np.array([0.0, 0.0, 0.0, 0.0, 0.0, 0.0])

        # Timer que publica los angulos 10 veces por segundo
        self.timer = self.create_timer(0.1, self.timer_callback)

        self.get_logger().info(
            'Nodo IK con pseudoinversa iniciado. Esperando objetivo en /target...'
        )

    def dh_matrix(self, theta, d, a, alpha):
        return np.array([
            [
                np.cos(theta),
                -np.sin(theta) * np.cos(alpha),
                np.sin(theta) * np.sin(alpha),
                a * np.cos(theta)
            ],
            [
                np.sin(theta),
                np.cos(theta) * np.cos(alpha),
                -np.cos(theta) * np.sin(alpha),
                a * np.sin(theta)
            ],
            [
                0,
                np.sin(alpha),
                np.cos(alpha),
                d
            ],
            [
                0,
                0,
                0,
                1
            ]
        ])

    def get_fk_position(self, q):
        pi_2 = np.pi / 2

        A1 = self.dh_matrix(q[0], 0.1452, 0, -pi_2)
        A2 = self.dh_matrix(q[1] - pi_2, 0, 0.429, 0)
        A3 = self.dh_matrix(q[2], 0.1223, 0.4115, 0)
        A4 = self.dh_matrix(q[3] + pi_2, 0.106, 0, pi_2)
        A5 = self.dh_matrix(q[4], 0.11315, 0, -pi_2)
        A6 = self.dh_matrix(q[5], 0, 0, 0)

        T0_6 = A1 @ A2 @ A3 @ A4 @ A5 @ A6

        return np.array([
            T0_6[0, 3],
            T0_6[1, 3],
            T0_6[2, 3]
        ])

    def get_jacobian_numeric(self, q, delta=1e-5):
        J = np.zeros((3, 6))
        p0 = self.get_fk_position(q)

        for i in range(6):
            q_temp = q.copy()
            q_temp[i] += delta

            p_temp = self.get_fk_position(q_temp)

            J[:, i] = (p_temp - p0) / delta

        return J

    def target_callback(self, msg):
        p_d = np.array([msg.x, msg.y, msg.z])

        self.get_logger().info(f'Objetivo recibido: {p_d}')

        q_current = self.q_k.copy()

        for k in range(100):
            e_k = p_d - self.get_fk_position(q_current)
            error_final = np.linalg.norm(e_k)

            if error_final < 1e-3:
                self.get_logger().info(
                    f'Convergencia en {k} iteraciones. '
                    f'Error: {error_final:.4f} m'
                )

                self.get_logger().info(
                    f'Solucion q: {q_current}'
                )

                self.q_k = q_current
                return

            J = self.get_jacobian_numeric(q_current)
            J_pinv = np.linalg.pinv(J)

            q_current = q_current + 0.1 * (J_pinv @ e_k)

        error_final = np.linalg.norm(
            p_d - self.get_fk_position(q_current)
        )

        self.get_logger().warn(
            f'No convergio. Error final: {error_final:.4f} m'
        )

        self.get_logger().info(
            f'Ultima solucion q: {q_current}'
        )

        self.q_k = q_current

    def timer_callback(self):
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

        msg.position = self.q_k.tolist()

        self.publisher.publish(msg)


def main(args=None):
    rclpy.init(args=args)

    node = IKNode()

    rclpy.spin(node)

    node.destroy_node()

    rclpy.shutdown()


if __name__ == '__main__':
    main()
