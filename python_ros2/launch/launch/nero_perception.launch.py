# nero_perception.launch.py
#
# Sobe os 3 nós de percepção na máquina Ubuntu com um único comando:
#   - apriltag_detector_node  (2 câmeras de parede -> /apriltag/*/pose_raw)
#   - pose_estimator_node     (encadeamento + Kalman -> /robot_pose)
#   - gesture_emotion_node    (webcam PCYes -> /Gesture)
#
# Uso:
#   ros2 launch nero_perception.launch.py \
#       config_file:=/home/cleide/repo/config/cameras.yaml \
#       gesture_camera:=/dev/v4l/by-id/usb-PCYes_...-video-index0
#
# Para depurar um nó isoladamente, rode-o com ros2 run (READMEs das partes
# 1–3); este launch é para a operação integrada.
#
# Instalação: este arquivo pode ficar em qualquer pasta (ros2 launch aceita
# caminho direto de arquivo .launch.py), p.ex. python_ros2/launch/ do repo:
#   ros2 launch python_ros2/launch/nero_perception.launch.py config_file:=...

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    config_file = LaunchConfiguration("config_file")
    gesture_camera = LaunchConfiguration("gesture_camera")

    return LaunchDescription([
        DeclareLaunchArgument(
            "config_file",
            description="Caminho ABSOLUTO do config/cameras.yaml (com "
                        "intrínsecos, extrínsecos e robot_T_tag preenchidos).",
        ),
        DeclareLaunchArgument(
            "gesture_camera",
            default_value="0",
            description="Fonte de vídeo da webcam de gestos (PCYes): caminho "
                        "/dev/v4l/by-id/... (recomendado) ou índice.",
        ),

        Node(
            package="apriltag_detector",
            executable="apriltag_detector_node",
            name="apriltag_detector",
            output="screen",
            parameters=[{"config_file": config_file}],
            respawn=True,
            respawn_delay=2.0,
        ),

        Node(
            package="pose_estimator",
            executable="pose_estimator_node",
            name="pose_estimator",
            output="screen",
            parameters=[{"config_file": config_file}],
            respawn=True,
            respawn_delay=2.0,
        ),

        Node(
            package="gesture_emotion",
            executable="gesture_emotion_node",
            name="gesture_emotion",
            output="screen",
            parameters=[{"video_source": gesture_camera}],
            # sem respawn: tem janela de vídeo interativa; se cair, investigar
        ),
    ])
