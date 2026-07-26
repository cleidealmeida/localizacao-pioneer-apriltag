# core.py — funções puras de calibração de câmera (tabuleiro de xadrez) e de
# detecção/pose de AprilTag. Extraídas de calib.py e reverse_localization.py
# pra serem reaproveitadas também pelo server.py (assistente ao vivo no
# navegador). Sem argparse, sem leitura/escrita de arquivo — só a matemática.
import cv2
import numpy as np
from scipy.spatial.transform import Rotation as ScipyRotation

PATTERN_SIZE = (10, 7)  # cantos internos do tabuleiro (colunas, linhas)
CORNER_CRITERIA = (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 30, 0.001)


def make_object_points(pattern_size=PATTERN_SIZE):
    objp = np.zeros((pattern_size[0] * pattern_size[1], 3), np.float32)
    objp[:, :2] = np.mgrid[0:pattern_size[0], 0:pattern_size[1]].T.reshape(-1, 2)
    return objp


def find_chessboard_corners(gray_image, pattern_size=PATTERN_SIZE):
    """Retorna (True, corners_subpixel) se achou o tabuleiro, senão (False, None)."""
    ret, corners = cv2.findChessboardCorners(gray_image, pattern_size, None)
    if not ret:
        return False, None
    corners2 = cv2.cornerSubPix(
        gray_image, corners, (11, 11), (-1, -1), CORNER_CRITERIA
    )
    return True, corners2


def reprojection_quality(reproj_px):
    if reproj_px < 0.5:
        return "BOM"
    if reproj_px < 1.0:
        return "ACEITÁVEL"
    return "RUIM — refazer fotos"


def calibrate_camera(objpoints, imgpoints, image_size):
    """objpoints/imgpoints: listas acumuladas por find_chessboard_corners
    (um make_object_points() e um corners_subpixel por imagem aceita).
    image_size: (largura, altura) em pixels, ex. gray.shape[::-1].
    """
    if not objpoints:
        raise ValueError("Tabuleiro não encontrado em nenhuma imagem.")

    ret, mtx, dist, rvecs, tvecs = cv2.calibrateCamera(
        objpoints, imgpoints, image_size, None, None
    )

    # Erro de reprojeção. Usa np.linalg.norm (equivalente a cv2.norm(...,
    # NORM_L2) sobre os mesmos dados) em vez de cv2.norm diretamente: algumas
    # versões do OpenCV retornam cornerSubPix como (N,2) em vez de (N,1,2),
    # o que faz cv2.norm falhar por incompatibilidade de "tipo" (canais).
    total_err = 0.0
    for i in range(len(objpoints)):
        proj, _ = cv2.projectPoints(objpoints[i], rvecs[i], tvecs[i], mtx, dist)
        diff = imgpoints[i].reshape(-1, 2) - proj.reshape(-1, 2)
        total_err += float(np.linalg.norm(diff)) / len(proj)
    reproj = total_err / len(objpoints)

    return {
        "fx": float(mtx[0, 0]),
        "fy": float(mtx[1, 1]),
        "cx": float(mtx[0, 2]),
        "cy": float(mtx[1, 2]),
        "dist": [float(d) for d in dist.ravel()],
        "reprojection_error_px": float(reproj),
    }


def detect_tag_pose(detector, gray_image, cam_params, tag_size, tag_id):
    """Detecta a tag `tag_id` em gray_image e retorna a pose 4x4 C_T_tag
    (câmera->tag), ou None se não detectada nesse frame. `detector` é uma
    instância de pupil_apriltags.Detector (cabe ao chamador criar/reaproveitar
    — não é thread-safe, então cada câmera precisa da sua própria instância,
    igual já é feito em apriltag_detector_node.py)."""
    results = detector.detect(
        gray_image, estimate_tag_pose=True,
        camera_params=cam_params, tag_size=tag_size,
    )
    for tag in results:
        if tag.tag_id != tag_id:
            continue
        T = np.eye(4)
        T[:3, :3] = tag.pose_R
        T[:3, 3] = tag.pose_t.flatten()
        return T
    return None


def average_poses(transforms_C_T_ref):
    """Recebe uma lista de poses C_T_ref (câmera->tag de referência, 4x4) e
    retorna (W_T_C, translation_std): a pose média da câmera no mundo
    (assumindo a tag de referência como origem, W_T_C = inv(C_T_ref)) e o
    desvio-padrão [x,y,z] da translação entre as amostras, em metros —
    mesma lógica de reverse_localization.py (translação: média aritmética;
    rotação: média de quatérnios alinhados por hemisfério)."""
    if len(transforms_C_T_ref) < 3:
        raise ValueError(
            f"Só {len(transforms_C_T_ref)} detecções — mínimo de 3 pra estimar a pose."
        )
    translations, quats = [], []
    for C_T_ref in transforms_C_T_ref:
        W_T_C = np.linalg.inv(C_T_ref)
        translations.append(W_T_C[:3, 3])
        quats.append(ScipyRotation.from_matrix(W_T_C[:3, :3]).as_quat())

    t_mean = np.mean(translations, axis=0)
    Q = np.array(quats)
    Q[Q[:, 3] < 0] *= -1.0
    q_mean = Q.mean(axis=0)
    q_mean /= np.linalg.norm(q_mean)

    W_T_C = np.eye(4)
    W_T_C[:3, :3] = ScipyRotation.from_quat(q_mean).as_matrix()
    W_T_C[:3, 3] = t_mean

    t_std = np.std(translations, axis=0)
    return W_T_C, t_std


def pose_stability_label(translation_std, threshold_m=0.01):
    return "ESTÁVEL" if float(np.max(translation_std)) < threshold_m else "INSTÁVEL — repetir com melhor iluminação"
