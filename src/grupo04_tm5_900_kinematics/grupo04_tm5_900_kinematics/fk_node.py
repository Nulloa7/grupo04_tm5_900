import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState

import numpy as np


# ---------------------------------------------------------
# MATRICES BÁSICAS DE TRANSFORMACIÓN
# ---------------------------------------------------------

def translation_matrix(x, y, z):
    """Matriz homogénea de traslación."""
    return np.array([
        [1.0, 0.0, 0.0, x],
        [0.0, 1.0, 0.0, y],
        [0.0, 0.0, 1.0, z],
        [0.0, 0.0, 0.0, 1.0]
    ])


def rotation_x(angle):
    """Rotación alrededor del eje X."""
    c = np.cos(angle)
    s = np.sin(angle)

    return np.array([
        [1.0, 0.0, 0.0, 0.0],
        [0.0, c,   -s,   0.0],
        [0.0, s,    c,   0.0],
        [0.0, 0.0, 0.0, 1.0]
    ])


def rotation_y(angle):
    """Rotación alrededor del eje Y."""
    c = np.cos(angle)
    s = np.sin(angle)

    return np.array([
        [c,   0.0, s,   0.0],
        [0.0, 1.0, 0.0, 0.0],
        [-s,  0.0, c,   0.0],
        [0.0, 0.0, 0.0, 1.0]
    ])


def rotation_z(angle):
    """Rotación alrededor del eje Z."""
    c = np.cos(angle)
    s = np.sin(angle)

    return np.array([
        [c,   -s,  0.0, 0.0],
        [s,    c,  0.0, 0.0],
        [0.0, 0.0, 1.0, 0.0],
        [0.0, 0.0, 0.0, 1.0]
    ])


def rpy_matrix(roll, pitch, yaw):
    """
    Convención URDF:
    R = Rz(yaw) @ Ry(pitch) @ Rx(roll)
    """
    return (
        rotation_z(yaw)
        @ rotation_y(pitch)
        @ rotation_x(roll)
    )


def joint_transform(xyz, rpy, q):
    """
    Transformación de una articulación revoluta según URDF.

    Primero se aplica el origin fijo del joint:
    traslación + orientación RPY.

    Después se aplica el giro articular alrededor de Z,
    porque en el URDF los joints tienen axis="0 0 1".
    """

    x, y, z = xyz
    roll, pitch, yaw = rpy

    T_origin = (
        translation_matrix(x, y, z)
        @ rpy_matrix(roll, pitch, yaw)
    )

    return T_origin @ rotation_z(q)


# ---------------------------------------------------------
# CINEMÁTICA DIRECTA DEL TM5-900 BASADA EN SU URDF
# ---------------------------------------------------------

def fk_tm5_900(q):

    pi_2 = np.pi / 2.0

    # Datos tomados del archivo:
    # tm_description/urdf/tm5-900-nominal.urdf

    joint_data = [

        # joint_1
        (
            (0.0, 0.0, 0.1452),
            (0.0, 0.0, 0.0)
        ),

        # joint_2
        (
            (0.0, 0.0, 0.0),
            (-pi_2, -pi_2, 0.0)
        ),

        # joint_3
        (
            (0.429, 0.0, 0.0),
            (0.0, 0.0, 0.0)
        ),

        # joint_4
        (
            (0.4115, 0.0, -0.1223),
            (0.0, 0.0, pi_2)
        ),

        # joint_5
        (
            (0.0, -0.106, 0.0),
            (pi_2, 0.0, 0.0)
        ),

        # joint_6
        (
            (0.0, -0.11315, 0.0),
            (pi_2, 0.0, 0.0)
        )
    ]

    # Matriz identidad inicial
    T = np.eye(4)

    # Multiplicamos las 6 transformaciones
    for i in range(6):

        xyz, rpy = joint_data[i]

        T = T @ joint_transform(
            xyz,
            rpy,
            q[i]
        )

    return T


# ---------------------------------------------------------
# NODO ROS 2 DE CINEMÁTICA DIRECTA
# ---------------------------------------------------------

class FKNode(Node):

    def __init__(self):
        super().__init__('fk_node')

        # El nodo escucha los estados articulares
        self.subscription = self.create_subscription(
            JointState,
            '/joint_states',
            self.joint_callback,
            10
        )

        self.get_logger().info(
            'Nodo FK iniciado. Esperando /joint_states...'
        )

    def joint_callback(self, msg):

        # Orden esperado de joints
        joint_names = [
            'joint_1',
            'joint_2',
            'joint_3',
            'joint_4',
            'joint_5',
            'joint_6'
        ]

        # -------------------------------------------------
        # ORDENAR LAS POSICIONES POR NOMBRE
        # -------------------------------------------------
        # Así evitamos depender del orden en que ROS
        # publique los joints.
        positions = dict(
            zip(msg.name, msg.position)
        )

        # Si falta algún joint, no calculamos
        if not all(
            name in positions
            for name in joint_names
        ):
            return

        # Vector articular q
        q = np.array([
            positions[name]
            for name in joint_names
        ])

        # -------------------------------------------------
        # CALCULAR CINEMÁTICA DIRECTA
        # -------------------------------------------------
        T = fk_tm5_900(q)

        # La última columna contiene la posición
        # del origen de link_6 respecto a la base.
        x = T[0, 3]
        y = T[1, 3]
        z = T[2, 3]

        self.get_logger().info(
            f'FK link_6: '
            f'x={x:.4f}, '
            f'y={y:.4f}, '
            f'z={z:.4f}'
        )


def main(args=None):

    # Inicializar ROS 2
    rclpy.init(args=args)

    # Crear el nodo
    node = FKNode()

    # Mantenerlo activo escuchando /joint_states
    rclpy.spin(node)

    # Cerrar correctamente
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
