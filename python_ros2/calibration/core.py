# core.py — funções puras de calibração de câmera (tabuleiro de xadrez).
# Extraídas de calib.py pra serem reaproveitadas também pelo server.py
# (assistente de calibração ao vivo no navegador). Sem argparse, sem
# leitura/escrita de arquivo — só a matemática do OpenCV.
import cv2
import numpy as np

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
