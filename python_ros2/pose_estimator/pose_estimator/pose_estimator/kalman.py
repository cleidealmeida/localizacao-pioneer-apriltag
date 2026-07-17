# kalman.py — Filtro de Kalman planar para localização do robô.
# Estado x = [x, y, theta]  (Eq. 3.6 do plano de trabalho)
# Predição por incrementos de odometria (Eq. 3.7) e atualização por medição
# das câmeras (Eq. 3.8). Puro numpy: importável e testável sem ROS.
import numpy as np

from .transforms import wrap_angle


class PlanarKalmanFilter:
    """EKF planar: predição não-linear por dead-reckoning, medição direta.

    - predict(dx, dy, dyaw): incrementos no frame do robô (vindos da odometria).
      A propagação é não-linear em theta, então usamos o jacobiano F (EKF).
    - update(z, R): medição absoluta [x, y, yaw] no frame do mundo (vinda de
      uma câmera após o encadeamento). H = I. Inclui gate de Mahalanobis para
      rejeitar detecções espúrias (reflexos, falsos positivos).
    """

    # Qui-quadrado 3 gl @ 99% — limiar do gate de outliers
    GATE_CHI2_99 = 11.345

    def __init__(
        self,
        sigma_xy_process: float = 0.01,     # [m] ruído de processo base por passo
        sigma_theta_process: float = 0.01,  # [rad]
        alpha_motion: float = 0.10,         # fração do movimento que vira incerteza
    ):
        self.x = np.zeros(3)
        self.P = np.eye(3)
        self.initialized = False
        self.sq_xy = sigma_xy_process
        self.sq_th = sigma_theta_process
        self.alpha = alpha_motion
        self.n_rejected = 0

    def initialize(self, x, y, theta, sigma_xy=0.10, sigma_theta=0.20):
        self.x = np.array([x, y, wrap_angle(theta)], dtype=float)
        self.P = np.diag([sigma_xy**2, sigma_xy**2, sigma_theta**2])
        self.initialized = True

    def predict(self, dx: float, dy: float, dyaw: float):
        """Propaga o estado com o incremento de odometria (frame do robô)."""
        if not self.initialized:
            return
        th = self.x[2]
        c, s = np.cos(th), np.sin(th)

        # f(x,u): composição do incremento local na pose global
        self.x[0] += c * dx - s * dy
        self.x[1] += s * dx + c * dy
        self.x[2] = wrap_angle(self.x[2] + dyaw)

        # Jacobiano de f em relação ao estado
        F = np.array([
            [1.0, 0.0, -s * dx - c * dy],
            [0.0, 1.0,  c * dx - s * dy],
            [0.0, 0.0,  1.0],
        ])

        # Ruído de processo: base + proporção do movimento (quanto mais anda,
        # mais incerteza a odometria acumula — deslizamento de rodas)
        motion = np.hypot(dx, dy)
        q_xy = (self.sq_xy + self.alpha * motion) ** 2
        q_th = (self.sq_th + self.alpha * abs(dyaw)) ** 2
        Q = np.diag([q_xy, q_xy, q_th])

        self.P = F @ self.P @ F.T + Q

    def update(self, z: np.ndarray, R: np.ndarray) -> bool:
        """Atualiza com medição absoluta z=[x,y,yaw]; retorna False se rejeitada.

        Se o filtro ainda não foi inicializado, a primeira medição válida
        inicializa o estado (não começamos na origem da odometria).
        """
        z = np.asarray(z, dtype=float)
        if not self.initialized:
            self.initialize(z[0], z[1], z[2])
            return True

        # Inovação com wrap no componente angular
        y = z - self.x
        y[2] = wrap_angle(y[2])

        S = self.P + R  # H = I
        S_inv = np.linalg.inv(S)

        # Gate de Mahalanobis: rejeita medições estatisticamente impossíveis
        d2 = float(y @ S_inv @ y)
        if d2 > self.GATE_CHI2_99:
            self.n_rejected += 1
            return False

        K = self.P @ S_inv
        self.x = self.x + K @ y
        self.x[2] = wrap_angle(self.x[2])
        I_KH = np.eye(3) - K
        # Forma de Joseph: mantém P simétrica/positiva mesmo com arredondamento
        self.P = I_KH @ self.P @ I_KH.T + K @ R @ K.T
        return True

    def covariance_6x6(self) -> np.ndarray:
        """Covariância no layout do PoseWithCovariance (x,y,z,rot_x,rot_y,rot_z)."""
        C = np.zeros((6, 6))
        C[0, 0] = self.P[0, 0]
        C[0, 1] = C[1, 0] = self.P[0, 1]
        C[1, 1] = self.P[1, 1]
        C[5, 5] = self.P[2, 2]
        C[0, 5] = C[5, 0] = self.P[0, 2]
        C[1, 5] = C[5, 1] = self.P[1, 2]
        C[2, 2] = C[3, 3] = C[4, 4] = 1e-6  # dof não estimados
        return C


def camera_measurement_noise(
    distance_m: float,
    sigma_xy_base: float = 0.02,
    sigma_theta_base: float = 0.05,
    k_dist: float = 0.01,
) -> np.ndarray:
    """Covariância de medição dependente da distância câmera→tag.

    O plano de trabalho (3.5.1) prevê incerteza variando com distância e ângulo
    de visada. Modelo simples e eficaz: sigma cresce com o quadrado da
    distância (a resolução angular do pixel se dilui com d²).
    """
    s_xy = sigma_xy_base + k_dist * distance_m**2
    s_th = sigma_theta_base + 0.5 * k_dist * distance_m**2
    return np.diag([s_xy**2, s_xy**2, s_th**2])
