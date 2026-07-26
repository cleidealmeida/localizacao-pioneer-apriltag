# pose_estimator_node.py
#
# Nó central de localização (Módulos 2 e 3 do plano de trabalho):
#   - assina as leituras cruas por câmera (/apriltag/camera_N/pose_raw) e a
#     odometria do Pioneer (/pose);
#   - encadeia W_T_R = W_T_C · C_T_tag · (robot_T_tag)^-1 por câmera;
#   - triangulação: cada medição atualiza o Filtro de Kalman com covariância
#     dependente da distância (fusão sequencial == média ponderada ótima);
#   - predição entre medições usa os incrementos da odometria (continuidade
#     em zonas de oclusão);
#   - publica /robot_pose (geometry_msgs/PoseWithCovarianceStamped) a 20 Hz.
#
# Tópicos auxiliares p/ análise (gráficos do artigo):
#   /apriltag/camera_N/pose_world — medição de cada câmera já no frame mundo
#   (permite calcular RMSE odometria vs visão vs Kalman, como no MATLAB antigo)
#
# Uso:
#   ros2 run pose_estimator pose_estimator_node --ros-args \
#       -p config_file:=/caminho/config/cameras.yaml

import numpy as np
import rclpy
import yaml
from geometry_msgs.msg import PoseStamped, PoseWithCovarianceStamped
from nav_msgs.msg import Odometry
from rclpy.node import Node

from .kalman import PlanarKalmanFilter, camera_measurement_noise
from .transforms import (
    chain_robot_pose,
    matrix_to_xy_yaw,
    pose_to_matrix,
    relative_planar_motion,
    yaw_to_quat,
)


class PoseEstimatorNode(Node):
    def __init__(self):
        super().__init__("pose_estimator_node")

        # --- Parâmetros ---
        self.declare_parameter("config_file", "config/cameras.yaml")
        self.declare_parameter("odom_topic", "/pose")
        self.declare_parameter("publish_rate_hz", 20.0)
        # Afinação do filtro (TR 4.3 do cronograma: ajuste de Q e R)
        self.declare_parameter("sigma_xy_process", 0.01)
        self.declare_parameter("sigma_theta_process", 0.01)
        self.declare_parameter("alpha_motion", 0.10)
        self.declare_parameter("sigma_xy_camera", 0.02)
        self.declare_parameter("sigma_theta_camera", 0.05)
        self.declare_parameter("k_dist_camera", 0.01)

        # --- Config geométrica (cameras.yaml) ---
        config_file = self.get_parameter("config_file").value
        with open(config_file) as f:
            cfg = yaml.safe_load(f)

        self.W_T_C = {}
        for name in ("camera_1", "camera_2"):
            ext = cfg["cameras"][name].get("extrinsics_W_T_C")
            if ext is None:
                raise RuntimeError(
                    f"extrinsics_W_T_C de {name} ausente no cameras.yaml — "
                    "rode reverse_localization.py para as duas câmeras."
                )
            self.W_T_C[name] = np.array(ext, dtype=float)
            self.get_logger().info(
                f"[{name}] W_T_C carregada, câmera em "
                f"{np.round(self.W_T_C[name][:3, 3], 3)} m"
            )

        rtt = cfg.get("robot_T_tag")
        if rtt is None:
            raise RuntimeError(
                "robot_T_tag ausente no cameras.yaml — adicione a matriz 4x4 "
                "da pose da tag no referencial do robô (ver cameras.example.yaml)."
            )
        self.robot_T_tag = np.array(rtt, dtype=float)

        # --- Filtro ---
        self.kf = PlanarKalmanFilter(
            sigma_xy_process=self.get_parameter("sigma_xy_process").value,
            sigma_theta_process=self.get_parameter("sigma_theta_process").value,
            alpha_motion=self.get_parameter("alpha_motion").value,
        )
        self.s_xy_cam = self.get_parameter("sigma_xy_camera").value
        self.s_th_cam = self.get_parameter("sigma_theta_camera").value
        self.k_dist = self.get_parameter("k_dist_camera").value

        self.last_odom_T = None
        self.n_updates = {"camera_1": 0, "camera_2": 0}

        # --- Subscribers ---
        for name in ("camera_1", "camera_2"):
            self.create_subscription(
                PoseStamped, f"/apriltag/{name}/pose_raw",
                lambda msg, n=name: self.camera_callback(msg, n), 10,
            )
        odom_topic = self.get_parameter("odom_topic").value
        self.create_subscription(Odometry, odom_topic, self.odom_callback, 10)
        self.get_logger().info(f"Odometria: assinando {odom_topic}")

        # --- Publishers ---
        self.pub_pose = self.create_publisher(
            PoseWithCovarianceStamped, "/robot_pose", 10
        )
        self.pub_world = {
            name: self.create_publisher(
                PoseStamped, f"/apriltag/{name}/pose_world", 10
            )
            for name in ("camera_1", "camera_2")
        }

        rate = float(self.get_parameter("publish_rate_hz").value)
        self.create_timer(1.0 / rate, self.publish_estimate)
        self.create_timer(10.0, self.report_stats)

    # ---------- callbacks ----------

    def odom_callback(self, msg: Odometry):
        p = msg.pose.pose
        T = pose_to_matrix(
            p.position.x, p.position.y, p.position.z,
            p.orientation.x, p.orientation.y, p.orientation.z, p.orientation.w,
        )
        if self.last_odom_T is not None:
            dx, dy, dyaw = relative_planar_motion(self.last_odom_T, T)
            self.kf.predict(dx, dy, dyaw)
        self.last_odom_T = T

    def camera_callback(self, msg: PoseStamped, camera: str):
        p = msg.pose
        C_T_tag = pose_to_matrix(
            p.position.x, p.position.y, p.position.z,
            p.orientation.x, p.orientation.y, p.orientation.z, p.orientation.w,
        )
        # Encadeamento (Módulo 2)
        W_T_R = chain_robot_pose(self.W_T_C[camera], C_T_tag, self.robot_T_tag)
        x, y, yaw = matrix_to_xy_yaw(W_T_R)

        # Publica a medição no frame mundo (análise/artigo)
        wm = PoseStamped()
        wm.header = msg.header
        wm.header.frame_id = "world"
        wm.pose.position.x, wm.pose.position.y = x, y
        qx, qy, qz, qw = yaw_to_quat(yaw)
        wm.pose.orientation.x, wm.pose.orientation.y = float(qx), float(qy)
        wm.pose.orientation.z, wm.pose.orientation.w = float(qz), float(qw)
        self.pub_world[camera].publish(wm)

        # Atualização do Kalman com ruído dependente da distância câmera→tag
        dist = float(np.linalg.norm(C_T_tag[:3, 3]))
        R = camera_measurement_noise(
            dist, self.s_xy_cam, self.s_th_cam, self.k_dist
        )
        accepted = self.kf.update(np.array([x, y, yaw]), R)
        if accepted:
            self.n_updates[camera] += 1

    # ---------- publicação ----------

    def publish_estimate(self):
        if not self.kf.initialized:
            return
        msg = PoseWithCovarianceStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = "world"
        msg.pose.pose.position.x = float(self.kf.x[0])
        msg.pose.pose.position.y = float(self.kf.x[1])
        qx, qy, qz, qw = yaw_to_quat(float(self.kf.x[2]))
        msg.pose.pose.orientation.x = float(qx)
        msg.pose.pose.orientation.y = float(qy)
        msg.pose.pose.orientation.z = float(qz)
        msg.pose.pose.orientation.w = float(qw)
        msg.pose.covariance = self.kf.covariance_6x6().ravel().tolist()
        self.pub_pose.publish(msg)

    def report_stats(self):
        if not self.kf.initialized:
            self.get_logger().warn(
                "Filtro ainda não inicializado — nenhuma câmera detectou a tag "
                "do robô até agora."
            )
            return
        sx = float(np.sqrt(self.kf.P[0, 0])) * 100
        self.get_logger().info(
            f"updates c1={self.n_updates['camera_1']} "
            f"c2={self.n_updates['camera_2']} "
            f"rejeitadas={self.kf.n_rejected} | σx≈{sx:.1f} cm"
        )
        self.n_updates = {"camera_1": 0, "camera_2": 0}


def main(args=None):
    rclpy.init(args=args)
    node = PoseEstimatorNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
