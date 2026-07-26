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
import numpy as np
import yaml
from pupil_apriltags import Detector
from scipy.spatial.transform import Rotation as R

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

    translations, quats = [], []
    t0 = time.time()
    while len(translations) < N_FRAMES and (time.time() - t0) < TIMEOUT_S:
        ret, frame = cap.read()
        if not ret:
            continue
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        results = detector.detect(
            gray, estimate_tag_pose=True,
            camera_params=cam_params, tag_size=ref_size,
        )
        for tag in results:
            if tag.tag_id != ref_id:
                continue
            C_T_A0 = np.eye(4)
            C_T_A0[:3, :3] = tag.pose_R
            C_T_A0[:3, 3] = tag.pose_t.flatten()
            W_T_C = np.linalg.inv(C_T_A0)  # tag de referência == origem do mundo
            translations.append(W_T_C[:3, 3])
            quats.append(R.from_matrix(W_T_C[:3, :3]).as_quat())
            print(f"  detecção {len(translations)}/{N_FRAMES}", end="\r")
    cap.release()

    if len(translations) < 3:
        raise SystemExit(
            f"\nSó {len(translations)} detecções em {TIMEOUT_S}s — verifique "
            "iluminação, foco, distância e o id/tamanho da tag no cameras.yaml."
        )

    # Média: translação aritmética; rotação via média de quatérnios
    # (alinhando hemisfério e normalizando — adequado para orientações próximas)
    t_mean = np.mean(translations, axis=0)
    Q = np.array(quats)
    Q[Q[:, 3] < 0] *= -1.0  # mesmo hemisfério
    q_mean = np.mean(Q, axis=0)
    q_mean /= np.linalg.norm(q_mean)

    W_T_C = np.eye(4)
    W_T_C[:3, :3] = R.from_quat(q_mean).as_matrix()
    W_T_C[:3, 3] = t_mean

    t_std = np.std(translations, axis=0)
    print(f"\n\n[{args.camera}] W_T_C estimada com {len(translations)} amostras")
    print(f"Posição da câmera no mundo [x y z]: {np.round(t_mean, 4)} m")
    print(f"Desvio-padrão da posição: {np.round(t_std * 1000, 1)} mm "
          f"({'ESTÁVEL' if np.max(t_std) < 0.01 else 'INSTÁVEL — repetir com melhor iluminação'})")

    cfg["cameras"][args.camera]["extrinsics_W_T_C"] = [
        [float(v) for v in row] for row in W_T_C
    ]
    with open(args.config, "w") as f:
        yaml.safe_dump(cfg, f, sort_keys=False)
    print(f"Extrínsecos gravados em {args.config} -> cameras.{args.camera}.extrinsics_W_T_C")


if __name__ == "__main__":
    main()
