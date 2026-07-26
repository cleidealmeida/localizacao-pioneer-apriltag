#!/usr/bin/env python3
"""reverse_localization.py — localização reversa: estima a pose da câmera no
mundo (W_T_C) lendo a AprilTag de referência posicionada na ORIGEM do sistema
de coordenadas global, e grava o resultado direto no cameras.yaml.

Uso (uma vez POR CÂMERA, após calib.py, com a tag de referência na origem):
    python3 reverse_localization.py --config ../config/cameras.yaml --camera camera_1
    python3 reverse_localization.py --config ../config/cameras.yaml --camera camera_2

Diferenças da versão antiga:
  - família corrigida: tag36h11 (o 'tag36h1' antigo era typo);
  - usa os intrínsecos DA PRÓPRIA câmera lidos do cameras.yaml;
  - filtra pela reference_tag_id (ignora a tag do robô se estiver visível);
  - média de N frames (translação + quatérnio) em vez de 1 captura única,
    reduzindo o ruído desta etapa crítica de calibração;
  - grava a matriz W_T_C automaticamente no cameras.yaml (fim do copiar/colar).
"""
import argparse
import time

import cv2
import yaml
from pupil_apriltags import Detector

from core import average_poses, detect_tag_pose, pose_stability_label

N_FRAMES = 15          # detecções usadas na média
TIMEOUT_S = 30         # tempo máximo tentando detectar


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--config", required=True)
    ap.add_argument("--camera", required=True, choices=["camera_1", "camera_2"])
    args = ap.parse_args()

    with open(args.config) as f:
        cfg = yaml.safe_load(f)

    cam_cfg = cfg["cameras"][args.camera]
    intr = cam_cfg.get("intrinsics") or {}
    if not intr.get("fx"):
        raise SystemExit(f"Intrínsecos de {args.camera} ausentes — rode calib.py antes.")

    tag_cfg = cfg["tag"]
    ref_size = float(tag_cfg["reference_tag_size"])
    ref_id = int(tag_cfg["reference_tag_id"])
    family = tag_cfg.get("family", "tag36h11")

    cam_params = [intr["fx"], intr["fy"], intr["cx"], intr["cy"]]
    detector = Detector(families=family)

    cap = cv2.VideoCapture(cam_cfg["device"])
    if not cap.isOpened():
        raise SystemExit(f"Não abriu '{cam_cfg['device']}'.")
    cap.set(cv2.CAP_PROP_FRAME_WIDTH, cam_cfg.get("width", 640))
    cap.set(cv2.CAP_PROP_FRAME_HEIGHT, cam_cfg.get("height", 480))

    print(f"[{args.camera}] Procurando tag de referência id={ref_id} "
          f"(família {family}, lado {ref_size} m) na origem do mundo...")
    print(f"Coletando {N_FRAMES} detecções para a média. Não mova nada.")

    detections = []
    t0 = time.time()
    while len(detections) < N_FRAMES and (time.time() - t0) < TIMEOUT_S:
        ret, frame = cap.read()
        if not ret:
            continue
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        C_T_A0 = detect_tag_pose(detector, gray, cam_params, ref_size, ref_id)
        if C_T_A0 is not None:
            detections.append(C_T_A0)
            print(f"  detecção {len(detections)}/{N_FRAMES}", end="\r")
    cap.release()

    if len(detections) < 3:
        raise SystemExit(
            f"\nSó {len(detections)} detecções em {TIMEOUT_S}s — verifique "
            "iluminação, foco, distância e o id/tamanho da tag no cameras.yaml."
        )

    W_T_C, t_std = average_poses(detections)

    print(f"\n\n[{args.camera}] W_T_C estimada com {len(detections)} amostras")
    print(f"Posição da câmera no mundo [x y z]: {W_T_C[:3, 3].round(4)} m")
    print(f"Desvio-padrão da posição: {(t_std * 1000).round(1)} mm "
          f"({pose_stability_label(t_std)})")

    cfg["cameras"][args.camera]["extrinsics_W_T_C"] = [
        [float(v) for v in row] for row in W_T_C
    ]
    with open(args.config, "w") as f:
        yaml.safe_dump(cfg, f, sort_keys=False)
    print(f"Extrínsecos gravados em {args.config} -> cameras.{args.camera}.extrinsics_W_T_C")


if __name__ == "__main__":
    main()
