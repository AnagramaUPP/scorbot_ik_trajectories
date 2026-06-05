# ============================================================
#                    LIBRERÍAS NECESARIAS
# ============================================================

from launch import LaunchDescription
from launch_ros.actions import Node

# Permiten localizar archivos dentro del paquete ROS
from launch.substitutions import Command
from launch_ros.substitutions import FindPackageShare
from launch.substitutions import PathJoinSubstitution
from launch_ros.parameter_descriptions import ParameterValue

# ============================================================
#                FUNCIÓN PRINCIPAL DEL LAUNCH
# ============================================================

def generate_launch_description():

    # ========================================================
    #              CARGA DEL ARCHIVO URDF
    # ========================================================

    robot_description_content = Command([
        'cat ',  # Lee el contenido del archivo URDF

        PathJoinSubstitution([
            FindPackageShare('scorbot_ik_trajectories'),
            # Localiza la carpeta share del paquete

            'urdf',
            # Carpeta donde se almacena el URDF

            'scorbot.urdf'
            # Archivo URDF principal del robot
        ])
    ])
    rviz_config_path = PathJoinSubstitution([
        FindPackageShare('scorbot_ik_trajectories'),
        'rviz',
        'scorbot.rviz'
    ])
    
    # ========================================================
    #               DEFINICIÓN DE LOS NODOS
    # ========================================================

    return LaunchDescription([



        # ====================================================
        #            ROBOT STATE PUBLISHER
        # ====================================================

        Node(
            package='robot_state_publisher',
            # Paquete encargado de publicar los TF https://wiki.ros.org/robot_state_publisher

            executable='robot_state_publisher',
            # Ejecutable principal

            name='robot_state_publisher',
            # Nombre del nodo

            output='screen',
            # Muestra mensajes en la terminal

            parameters=[{
                'robot_description': ParameterValue(
                    robot_description_content,
                    value_type=str
                )
                # Parámetro que contiene el URDF del robot
            }]
        ),

        # ====================================================
        #                     RVIZ2
        # ====================================================

        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            arguments=['-d', rviz_config_path],
            output='screen'
        )


    ])