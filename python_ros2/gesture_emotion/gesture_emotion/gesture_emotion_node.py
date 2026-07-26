# gesture_emotion_node.py
#
# Nó ROS 2 do sistema de reconhecimento de gestos + emoções (dupla validação).
# Substitui o antigo main_ros.py, com as seguintes mudanças:
#   1. Fonte de vídeo deixa de ser uma URL RTSP hardcoded e vira um PARÂMETRO
#      ROS 2 ('video_source'), com default apontando para caminho estável
#      /dev/v4l/by-id/ (webcam PCYes) — ajuste via launch file ou linha de comando.
#   2. Sem relay UDP: o gesto é publicado APENAS no tópico ROS2 /Gesture
#      (std_msgs/String), que o MATLAB assina nativamente via ros2subscriber.
#   3. Parâmetros de modelo (k do KNN, nome do modelo ONNX, backend) expostos
#      como parâmetros ROS 2 para facilitar experimentos sem editar código.
#
# Uso:
#   ros2 run gesture_emotion gesture_emotion_node
#   ros2 run gesture_emotion gesture_emotion_node --ros-args \
#       -p video_source:="/dev/v4l/by-id/usb-PCYes_HD_Webcam-video-index0"
#   ros2 run gesture_emotion gesture_emotion_node --ros-args -p video_source:="0"

import os

os.environ["TF_ENABLE_ONEDNN_OPTS"] = "0"

import rclpy
from rclpy.node import Node
from std_msgs.msg import String

from .Emotion_GestureDetector.EmotionGestureCompiler import EmotionGestureCompiler


class GestureEmotionNode(Node):
    def __init__(self):
        super().__init__("gesture_emotion_node")

        # --- Parâmetros ROS 2 (ajustáveis por launch file ou -p na CLI) ---
        self.declare_parameter(
            "video_source",
            "/dev/v4l/by-id/CHANGE_ME-PCYes-webcam",  # rode `ls -l /dev/v4l/by-id/` e cole o caminho real
        )
        self.declare_parameter("model_name", "resnet18.onnx")
        self.declare_parameter("model_option", "onnx")
        self.declare_parameter("backend_option", 1)  # 1 = CPU (mais portátil); 2 = CUDA
        self.declare_parameter("providers", 1)       # 1 = CPUExecutionProvider
        self.declare_parameter("num_faces", 1)
        self.declare_parameter("knn_k", 7)
        self.declare_parameter("train_path", "Base_de_dados")

        # --- Publisher único do sistema: /Gesture ---
        self.publisher_ = self.create_publisher(String, "/Gesture", 10)
        self.get_logger().info("Publisher /Gesture criado (std_msgs/String).")

    def resolve_video_source(self):
        """Converte o parâmetro em algo que o cv2.VideoCapture aceite.

        - "0", "1", ... -> índice inteiro (fallback rápido para testes)
        - "/dev/..."    -> caminho de dispositivo estável (recomendado)
        - "rtsp://..."  -> stream de rede (legado, ainda suportado)
        """
        src = self.get_parameter("video_source").get_parameter_value().string_value
        if src.isdigit():
            return int(src)
        if src.startswith("/dev/") and not os.path.exists(src):
            self.get_logger().error(
                f"Dispositivo de vídeo '{src}' não existe. "
                "Rode `ls -l /dev/v4l/by-id/` e ajuste o parâmetro video_source."
            )
        return src

    def run(self):
        video_source = self.resolve_video_source()
        self.get_logger().info(f"Fonte de vídeo: {video_source}")

        model = EmotionGestureCompiler(
            com_ros=self.publisher_,
            model_name=self.get_parameter("model_name").value,
            model_option=self.get_parameter("model_option").value,
            backend_option=self.get_parameter("backend_option").value,
            providers=self.get_parameter("providers").value,
            fp16=False,
            num_faces=self.get_parameter("num_faces").value,
            train_path=self.get_parameter("train_path").value,
            k=self.get_parameter("knn_k").value,
        )

        self.get_logger().info(
            "Sistema de reconhecimento iniciado. Gestos validados por emoção "
            "GOOD serão publicados em /Gesture. Pressione 'q' na janela de "
            "vídeo para encerrar."
        )
        # O loop de vídeo é bloqueante por design: este nó não precisa de spin,
        # pois só publica (não tem subscriber nem timer ROS).
        model.video(video_path=video_source)


def main(args=None):
    rclpy.init(args=args)
    node = GestureEmotionNode()
    try:
        node.run()
    except KeyboardInterrupt:
        pass
    finally:
        node.get_logger().info("Finalizando o nó gesture_emotion.")
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
