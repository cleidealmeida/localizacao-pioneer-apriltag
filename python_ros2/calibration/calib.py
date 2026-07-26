#!/usr/bin/env python3
"""calib.py — calibração intrínseca por câmera, gravando direto no cameras.yaml.

Uso (uma vez POR CÂMERA, após o capture.py):
    python3 calib.py --config ../config/cameras.yaml --camera camera_1
    python3 calib.py --config ../config/cameras.yaml --camera camera_2

Diferenças da versão antiga:
  - cada câmera tem SEU conjunto de imagens e SEUS intrínsecos (antes um único
    calib.npz era usado para as duas câmeras — errado);
  - o resultado é escrito automaticamente no cameras.yaml, eliminando o
    copiar/colar de matrizes.
"""
import argparse
import glob
import os

import cv2
import numpy as np
import yaml

PADRAO = (10, 7)  # cantos internos do tabuleiro (colunas, linhas)
criteria = (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 30, 0.001)


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--config", required=True)
    ap.add_argument("--camera", required=True, choices=["camera_1", "camera_2"])
    ap.add_argument("--show", action="store_true", help="mostrar detecção de cada foto")
    args = ap.parse_args()

    objp = np.zeros((PADRAO[0] * PADRAO[1], 3), np.float32)
    objp[:, :2] = np.mgrid[0:PADRAO[0], 0:PADRAO[1]].T.reshape(-1, 2)

    objpoints, imgpoints = [], []
    images = sorted(glob.glob(os.path.join("imagens", args.camera, "*.jpg")))
    if not images:
        raise SystemExit(f"Nenhuma imagem em imagens/{args.camera}/ — rode capture.py antes.")

    print(f"[{args.camera}] Processando {len(images)} imagens...")
    gray = None
    for fname in images:
        img = cv2.imread(fname)
        gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
        ret, corners = cv2.findChessboardCorners(gray, PADRAO, None)
        if ret:
            objpoints.append(objp)
            corners2 = cv2.cornerSubPix(gray, corners, (11, 11), (-1, -1), criteria)
            imgpoints.append(corners2)
            if args.show:
                cv2.drawChessboardCorners(img, PADRAO, corners2, ret)
                cv2.imshow("Verificando", img)
                cv2.waitKey(100)
    cv2.destroyAllWindows()

    if not objpoints:
        raise SystemExit("Tabuleiro não encontrado em nenhuma imagem.")

    ret, mtx, dist, rvecs, tvecs = cv2.calibrateCamera(
        objpoints, imgpoints, gray.shape[::-1], None, None
    )

    # Erro de reprojeção — métrica de qualidade da calibração
    total_err = 0.0
    for i in range(len(objpoints)):
        proj, _ = cv2.projectPoints(objpoints[i], rvecs[i], tvecs[i], mtx, dist)
        total_err += cv2.norm(imgpoints[i], proj, cv2.NORM_L2) / len(proj)
    reproj = total_err / len(objpoints)

    print(f"\n✅ [{args.camera}] Calibração concluída "
          f"({len(objpoints)}/{len(images)} imagens usadas)")
    print(f"Erro de reprojeção médio: {reproj:.3f} px "
          f"({'BOM' if reproj < 0.5 else 'ACEITÁVEL' if reproj < 1.0 else 'RUIM — refazer fotos'})")
    print("K =\n", mtx)

    # Grava no cameras.yaml
    with open(args.config) as f:
        cfg = yaml.safe_load(f)
    cfg["cameras"][args.camera]["intrinsics"] = {
        "fx": float(mtx[0, 0]),
        "fy": float(mtx[1, 1]),
        "cx": float(mtx[0, 2]),
        "cy": float(mtx[1, 2]),
        "dist": [float(d) for d in dist.ravel()],
        "reprojection_error_px": float(reproj),
    }
    with open(args.config, "w") as f:
        yaml.safe_dump(cfg, f, sort_keys=False)
    print(f"Intrínsecos gravados em {args.config} -> cameras.{args.camera}.intrinsics")


if __name__ == "__main__":
    main()
