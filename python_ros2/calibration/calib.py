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
import yaml

from core import PATTERN_SIZE, calibrate_camera, find_chessboard_corners, make_object_points, reprojection_quality


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--config", required=True)
    ap.add_argument("--camera", required=True, choices=["camera_1", "camera_2"])
    ap.add_argument("--show", action="store_true", help="mostrar detecção de cada foto")
    args = ap.parse_args()

    objp = make_object_points()
    objpoints, imgpoints = [], []
    images = sorted(glob.glob(os.path.join("imagens", args.camera, "*.jpg")))
    if not images:
        raise SystemExit(f"Nenhuma imagem em imagens/{args.camera}/ — rode capture.py antes.")

    print(f"[{args.camera}] Processando {len(images)} imagens...")
    gray = None
    for fname in images:
        img = cv2.imread(fname)
        gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
        found, corners2 = find_chessboard_corners(gray)
        if found:
            objpoints.append(objp)
            imgpoints.append(corners2)
            if args.show:
                cv2.drawChessboardCorners(img, PATTERN_SIZE, corners2, found)
                cv2.imshow("Verificando", img)
                cv2.waitKey(100)
    cv2.destroyAllWindows()

    result = calibrate_camera(objpoints, imgpoints, gray.shape[::-1])
    reproj = result["reprojection_error_px"]

    print(f"\n✅ [{args.camera}] Calibração concluída "
          f"({len(objpoints)}/{len(images)} imagens usadas)")
    print(f"Erro de reprojeção médio: {reproj:.3f} px ({reprojection_quality(reproj)})")
    print(f"fx={result['fx']:.2f} fy={result['fy']:.2f} "
          f"cx={result['cx']:.2f} cy={result['cy']:.2f}")

    # Grava no cameras.yaml
    with open(args.config) as f:
        cfg = yaml.safe_load(f)
    cfg["cameras"][args.camera]["intrinsics"] = result
    with open(args.config, "w") as f:
        yaml.safe_dump(cfg, f, sort_keys=False)
    print(f"Intrínsecos gravados em {args.config} -> cameras.{args.camera}.intrinsics")


if __name__ == "__main__":
    main()
