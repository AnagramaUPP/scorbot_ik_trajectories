#!/usr/bin/env python3

# ============================================================
#                    IMPORTACIÓN DE LIBRERÍAS
# ============================================================

import rclpy
from rclpy.node import Node

from sensor_msgs.msg import JointState
# Mensaje usado para publicar las posiciones articulares j1...j5

from std_msgs.msg import Float64MultiArray
# Mensaje usado para recibir [x, y, z, delta] desde terminal

import numpy as np
# Librería para operaciones matemáticas y matrices


# ============================================================
#              FUNCIÓN DE CINEMÁTICA INVERSA
# ============================================================

def CI(Pefx, Pefy, Pefz, delta):

    # Longitudes del robot Scorbot en metros
    L1 = 0.3520
    L2 = -0.02437
    L3 = 0.220
    L4 = 0.220
    L5 = 0.1479

    # Ángulos auxiliares de orientación
    Phiy = np.pi / 2
    Phiz = np.pi
    gamma = np.pi

    # Cálculo de la primera articulación
    Theta1 = np.arctan2(Pefy, Pefx)

    # Matriz de rotación alrededor de Y
    Ry = np.array([
        [np.cos(Phiy), 0, np.sin(Phiy)],
        [0, 1, 0],
        [-np.sin(Phiy), 0, np.cos(Phiy)]
    ])

    # Matriz de rotación alrededor de Z
    Rz = np.array([
        [np.cos(Phiz), -np.sin(Phiz), 0],
        [np.sin(Phiz),  np.cos(Phiz), 0],
        [0, 0, 1]
    ])

    # Matriz relacionada con Theta1
    RT = np.array([
        [1, 0, 0],
        [0, np.cos(Theta1), -np.sin(Theta1)],
        [0, np.sin(Theta1),  np.cos(Theta1)]
    ])

    # Matriz asociada al ángulo delta
    RDelta = np.array([
        [np.cos(delta), 0, np.sin(delta)],
        [0, 1, 0],
        [-np.sin(delta), 0, np.cos(delta)]
    ])

    # Matriz de rotación auxiliar gamma
    Rgamma = np.array([
        [np.cos(gamma), -np.sin(gamma), 0],
        [np.sin(gamma),  np.cos(gamma), 0],
        [0, 0, 1]
    ])

    # Orientación deseada del efector final respecto a la base
    R6_0 = Ry @ Rz @ RT @ RDelta @ Rgamma

    # Rotación identidad entre el sistema 5 y 6
    R6_5 = np.eye(3)

    # Orientación del sistema 5 respecto a la base
    R5_0 = R6_0 @ R6_5.T

    # Posición deseada del efector final
    P6_0 = np.array([[Pefx], [Pefy], [Pefz]])

    # Distancia de la muñeca al efector final
    P6_5 = np.array([[0], [0], [L5]])

    # Posición del centro de muñeca
    PMu_0 = P6_0 - R5_0 @ P6_5

    Xc = PMu_0[0]
    Yc = PMu_0[1]
    Zc = PMu_0[2]

    # Distancia radial en el plano XY
    R = np.sqrt(Xc**2 + Yc**2)

    # Altura respecto a L1
    h = Zc - L1

    # Distancia desde la articulación 2 al centro de muñeca
    m = np.sqrt(h**2 + (R - L2)**2)

    # Ángulo auxiliar alpha
    alpha = np.arctan2(h, (R - L2))

    # Cálculo de beta mediante ley de cosenos
    beta_cos = (m**2 + L3**2 - L4**2) / (2 * m * L3)
    beta_cos = np.clip(beta_cos, -1.0, 1.0)

    beta = np.arctan2(np.sqrt(1 - beta_cos**2), beta_cos)

    # Segunda articulación
    Theta2 = alpha + beta
    Theta2 = float(Theta2[0])

    # Cálculo de phi mediante ley de cosenos
    phi_cos = (L3**2 + L4**2 - m**2) / (2 * L3 * L4)
    phi_cos = np.clip(phi_cos, -1.0, 1.0)

    phi = np.arctan2(np.sqrt(1 - phi_cos**2), phi_cos)

    # Tercera articulación
    Theta3 = -np.pi + phi
    Theta3 = float(Theta3[0])

    # Matriz de rotación del sistema 1 respecto a 0
    R1_0 = np.array([
        [np.cos(Theta1), -np.sin(Theta1), 0],
        [np.sin(Theta1),  np.cos(Theta1), 0],
        [0, 0, 1]
    ])

    # Matriz de rotación del sistema 2 respecto a 1
    R2_1 = np.array([
        [np.cos(Theta2), -np.sin(Theta2), 0],
        [0, 0, -1],
        [np.sin(Theta2),  np.cos(Theta2), 0]
    ])

    # Matriz de rotación del sistema 3 respecto a 2
    R3_2 = np.array([
        [np.cos(Theta3), -np.sin(Theta3), 0],
        [np.sin(Theta3),  np.cos(Theta3), 0],
        [0, 0, 1]
    ])

    # Orientación del sistema 3 respecto a la base
    R3_0 = R1_0 @ R2_1 @ R3_2

    # Orientación de la muñeca respecto al sistema 3
    R6_3 = R3_0.T @ R6_0 @ R6_5.T

    # Quinta articulación
    Theta5 = np.arctan2(
        np.sqrt(R6_3[2, 0]**2 + R6_3[2, 1]**2),
        R6_3[2, 2]
    )

    # Cuarta articulación
    Theta4 = np.arctan2(R6_3[1, 2], R6_3[0, 2])

    return float(Theta1), float(Theta2), float(Theta3), float(Theta4), float(Theta5)


# ============================================================
#                   CLASE PRINCIPAL DEL NODO
# ============================================================

class ScorbotIKPublisher(Node):

    def __init__(self):

        # Nombre del nodo ROS 2
        super().__init__('scorbot_ik_publisher')

        # Publicador hacia /joint_states
        self.publisher = self.create_publisher(
            JointState,
            '/joint_states',
            10
        )

        # Suscriptor al tópico donde se manda la posición deseada
        self.subscription = self.create_subscription(
            Float64MultiArray,
            '/scorbot/desired_pose',
            self.desired_pose_callback,
            10
        )

        # Valores iniciales del efector final
        self.Pefx = 0.51579628
        self.Pefy = 0.0
        self.Pefz = 0.2718165
        self.delta = 0

    

        # Temporizador: ejecuta publish_joint_states cada 0.1 s
        self.timer = self.create_timer(
            0.1,
            self.publish_joint_states
        )


    # ========================================================
    #          CALLBACK PARA RECIBIR LA POSICIÓN DESEADA
    # ========================================================

    def desired_pose_callback(self, msg):

        # El mensaje debe tener exactamente:
        # [x, y, z, delta]
        if len(msg.data) != 4:
            self.get_logger().warn(
                'El mensaje debe tener 4 valores: [x, y, z, delta]'
            )
            return

        # Se actualizan las variables internas del nodo
        self.Pefx = msg.data[0]
        self.Pefy = msg.data[1]
        self.Pefz = msg.data[2]
        self.delta = msg.data[3]

        self.get_logger().info(
            f'Nueva posición recibida: '
            f'x={self.Pefx:.3f}, y={self.Pefy:.3f}, '
            f'z={self.Pefz:.3f}, delta={self.delta:.3f}'
        )


    # ========================================================
    #             PUBLICACIÓN DE LOS JOINT STATES
    # ========================================================

    def publish_joint_states(self):

        # Se calcula la cinemática inversa
        theta1, theta2, theta3, theta4, theta5 = CI(
            self.Pefx,
            self.Pefy,
            self.Pefz,
            self.delta
        )

        # Se crea el mensaje JointState
        msg = JointState()

        # Tiempo actual del nodo
        msg.header.stamp = self.get_clock().now().to_msg()

        # Nombres de las articulaciones según el URDF
        msg.name = [
            'j1',
            'j2',
            'j3',
            'j4',
            'j5'
        ]

        # Posiciones articulares calculadas por la cinemática inversa
        msg.position = [
            theta1,
            theta2,
            theta3,
            theta4,
            theta5
        ]

        # Se publica el mensaje en /joint_states
        self.publisher.publish(msg)

        self.get_logger().info(
            f'Pef=({self.Pefx:.3f}, {self.Pefy:.3f}, {self.Pefz:.3f}), '
            f'delta={self.delta:.3f} | '
            f'j=[{theta1:.3f}, {theta2:.3f}, {theta3:.3f}, '
            f'{theta4:.3f}, {theta5:.3f}]'
        )


# ============================================================
#                       FUNCIÓN MAIN
# ============================================================

def main(args=None):

    # Inicializa ROS 2
    rclpy.init(args=args)

    # Crea el nodo
    node = ScorbotIKPublisher()

    # Mantiene el nodo ejecutándose
    rclpy.spin(node)

    # Destruye el nodo al cerrar
    node.destroy_node()

    # Cierra ROS 2
    rclpy.shutdown()


# ============================================================
#                    EJECUCIÓN DEL SCRIPT
# ============================================================

if __name__ == '__main__':
    main()