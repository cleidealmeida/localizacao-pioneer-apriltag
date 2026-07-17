# apriltag_detector_node.py
#
# Nó ROS 2 de detecção da AprilTag do robô pelas 2 câmeras de parede.
#
# Papel na arquitetura (definido no planejamento):
#   - publica a leitura CRUA de cada câmera separadamente (Ci_T_AR, a pose da
#     tag do robô no referencial da câmera i), SEM fundir;
#   - a fusão (encadeamento com W_T_Ci, triangulação e Filtro de Kalman) é
#     responsabilidade exclusiva do pose_estimator (parte 3).
#
# Correções em relação ao apriltag_reader.py antigo:
#   - família tag36h11 (o 'tag36h1' era typo e quebrava a detecção);
#   - intrínsecos POR CÂMERA lidos do cameras.yaml (antes: um calib.npz p/ ambas);
#   - extrínsecos não são mais hardcoded (nem usados aqui — ficam p/ o estimador);
#   - devices por /dev/v4l/by-id (estável entre boots) em vez de índices 0/1;
#   - filtra pela robot_tag_id (ignora a tag de referência se visível);
#   - mensagens ROS padrão (PoseStamped) com timestamp e frame_id, em vez de
#     Float64MultiArray sem semântica.
#
# Tópicos publicados:
#   /apriltag/camera_1/pose_raw  (geometry_msgs/PoseStamped, frame_id=camera_1)
#   /apriltag/camera_2/pose_raw  (geometry_msgs/PoseStamped, frame_id=camera_2)
#
# Uso:
#   ros2 run apriltag_detector apriltag_detector_node --ros-args \
#       -p config_file:=/caminho/para/config/cameras.yaml

import cv2
import numpy as np
import rclpy
import yaml
from geometry_msgs.msg import PoseStamped
from pupil_apriltags import Detector
from rclpy.node import Node
from scipy.spatial.transform import Rotation as R


class CameraUnit:
    """Encapsula uma câmera física: captura, intrínsecos e detector próprio."""

    def __init__(self, name: str, cam_cfg: dict, tag_cfg: dict, logger):
        self.name = name
        self.logger = logger

        intr = cam_cfg.get("intrinsics") or {}
        if not intr.get("fx"):
            raise RuntimeError(
                f"[{name}] intrínsecos ausentes no cameras.yaml — rode calib.py."
            )
        self.cam_params = [intr["fx"], intr["fy"], intr["cx"], intr["cy"]]

        self.tag_size = float(tag_cfg["robot_tag_size"])
        self.robot_tag_id = int(tag_cfg["robot_tag_id"])
        # Detector próprio por câmera (instâncias não são thread-safe entre si)
        self.detector = Detector(families=tag_cfg.get("family", "tag36h11"))

        device = cam_cfg["device"]
        self.cap = cv2.VideoCapture(device)
        if not self.cap.isOpened():
            raise RuntimeError(
                f"[{name}] não abriu '{device}'. Confira: ls -l /dev/v4l/by-id/"
            )
        self.cap.set(cv2.CAP_PROP_FRAME_WIDTH, cam_cfg.get("width", 640))
        self.cap.set(cv2.CAP_PROP_FRAME_HEIGHT, cam_cfg.get("height", 480))
        self.cap.set(cv2.CAP_PROP_BUFFERSIZE, 1)  # sempre o frame mais recente

        self.n_frames = 0
        self.n_detections = 0

    def read_tag_pose(self):
        """Retorna a matriz 4x4 Ci_T_AR (tag do robô no frame da câmera) ou None."""
        ret, frame = self.cap.read()
        if not ret:
            return None
        self.n_frames += 1

        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        results = self.detector.detect(
            gray, estimate_tag_pose=True,
            camera_params=self.cam_params, tag_size=self.tag_size,
        )
        for tag in results:
            if tag.tag_id != self.robot_tag_id:
                continue
            T = np.eye(4)
            T[:3, :3] = tag.pose_R
            T[:3, 3] = tag.pose_t.flatten()
            self.n_detections += 1
            return T
        return None

    def release(self):
        self.cap.release()


class AprilTagDetectorNode(Node):
    def __init__(self):
        super().__init__("apriltag_detector_node")

        self.declare_parameter("config_file", "config/cameras.yaml")
        self.declare_parameter("rate_hz", 20.0)

        config_file = self.get_parameter("config_file").value
        with open(config_file) as f:
            cfg = yaml.safe_load(f)

        tag_cfg = cfg["tag"]
        self.get_logger().info(
            f"Tag do robô: id={tag_cfg['robot_tag_id']}, "
            f"lado={tag_cfg['robot_tag_size']} m, família={tag_cfg['family']}"
        )

        self.cameras = {}
        self.publishers_ = {}
        for name in ("camera_1", "camera_2"):
            self.cameras[name] = CameraUnit(
                name, cfg["cameras"][name], tag_cfg, self.get_logger()
            )
            self.publishers_[name] = self.create_publisher(
                PoseStamped, f"/apriltag/{name}/pose_raw", 10
            )
            self.get_logger().info(f"[{name}] pronta -> /apriltag/{name}/pose_raw")

        rate = float(self.get_parameter("rate_hz").value)
        self.timer = self.create_timer(1.0 / rate, self.tick)
        self.stats_timer = self.create_timer(10.0, self.report_stats)

    def tick(self):
        stamp = self.get_clock().now().to_msg()
        for name, cam in self.cameras.items():
            T = cam.read_tag_pose()
            if T is None:
                continue  # ausência de mensagem == sem detecção neste ciclo
            msg = PoseStamped()
            msg.header.stamp = stamp
            msg.header.frame_id = name
            msg.pose.position.x = float(T[0, 3])
            msg.pose.position.y = float(T[1, 3])
            msg.pose.position.z = float(T[2, 3])
            qx, qy, qz, qw = R.from_matrix(T[:3, :3]).as_quat()
            msg.pose.orientation.x = float(qx)
            msg.pose.orientation.y = float(qy)
            msg.pose.orientation.z = float(qz)
            msg.pose.orientation.w = float(qw)
            self.publishers_[name].publish(msg)

    def report_stats(self):
        for name, cam in self.cameras.items():
            pct = 100.0 * cam.n_detections / cam.n_frames if cam.n_frames else 0.0
            self.get_logger().info(
                f"[{name}] taxa de detecção nos últimos ciclos: "
                f"{cam.n_detections}/{cam.n_frames} ({pct:.0f}%)"
            )
            cam.n_frames = cam.n_detections = 0

    def destroy_node(self):
        for cam in self.cameras.values():
            cam.release()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = AprilTagDetectorNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
