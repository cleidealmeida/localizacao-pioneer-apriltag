# transforms.py — utilitários de transformações homogêneas SE(3)/SE(2).
# Puro numpy/scipy: importável e testável sem ROS.
import numpy as np
from scipy.spatial.transform import Rotation as R


def wrap_angle(a: float) -> float:
    """Normaliza ângulo para (-pi, pi]."""
    return float(np.arctan2(np.sin(a), np.cos(a)))


def quat_to_matrix(qx, qy, qz, qw) -> np.ndarray:
    """Quatérnio (x,y,z,w) -> matriz homogênea 4x4 (rotação pura)."""
    T = np.eye(4)
    T[:3, :3] = R.from_quat([qx, qy, qz, qw]).as_matrix()
    return T


def pose_to_matrix(x, y, z, qx, qy, qz, qw) -> np.ndarray:
    """Posição + quatérnio -> matriz homogênea 4x4."""
    T = quat_to_matrix(qx, qy, qz, qw)
    T[:3, 3] = [x, y, z]
    return T


def matrix_to_xy_yaw(T: np.ndarray):
    """Matriz 4x4 -> (x, y, yaw) no plano.

    O yaw é extraído via a convenção 'zyx' (primeiro ângulo = rotação em z),
    consistente com o quat2eul default do MATLAB usado no main_controller.m.
    """
    yaw = R.from_matrix(T[:3, :3]).as_euler("zyx")[0]
    return float(T[0, 3]), float(T[1, 3]), float(yaw)


def yaw_to_quat(yaw: float):
    """Yaw plano -> quatérnio (x,y,z,w)."""
    return R.from_euler("z", yaw).as_quat()


def chain_robot_pose(W_T_C: np.ndarray, C_T_tag: np.ndarray,
                     robot_T_tag: np.ndarray) -> np.ndarray:
    """Encadeamento do plano de trabalho (Módulo 2):

        W_T_R = W_T_C · C_T_tag · (R_T_tag)^-1

    onde robot_T_tag é a pose da tag no referencial do robô (o r_H_t do
    controlador MATLAB — a tag fica deslocada do centro do Pioneer).
    """
    return W_T_C @ C_T_tag @ np.linalg.inv(robot_T_tag)


def relative_planar_motion(T_prev: np.ndarray, T_curr: np.ndarray):
    """Movimento relativo no PLANO entre duas poses de odometria.

    Retorna (dx, dy, dyaw) expressos no frame do robô em T_prev — é o
    incremento de dead-reckoning usado na etapa de predição do Kalman.
    """
    x0, y0, th0 = matrix_to_xy_yaw(T_prev)
    x1, y1, th1 = matrix_to_xy_yaw(T_curr)
    dx_w, dy_w = x1 - x0, y1 - y0
    c, s = np.cos(th0), np.sin(th0)
    # rotaciona o deslocamento mundial para o frame local anterior
    dx = c * dx_w + s * dy_w
    dy = -s * dx_w + c * dy_w
    dyaw = wrap_angle(th1 - th0)
    return float(dx), float(dy), float(dyaw)
