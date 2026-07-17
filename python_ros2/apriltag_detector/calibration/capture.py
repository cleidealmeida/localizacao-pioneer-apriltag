#!/usr/bin/env python3
"""capture.py — captura fotos do tabuleiro de xadrez para calibração intrínseca.

Uso (uma vez POR CÂMERA):
    python3 capture.py --config ../config/cameras.yaml --camera camera_1
    python3 capture.py --config ../config/cameras.yaml --camera camera_2

As fotos vão para imagens/camera_1/ e imagens/camera_2/ respectivamente,
para o calib.py calibrar cada câmera com o seu próprio conjunto.
"""
import argparse
import os
import time

import cv2
import yaml

INTERVALO = 2.0   # segundos entre fotos
LIMITE_FOTOS = 20


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--config", required=True, help="caminho do cameras.yaml")
    ap.add_argument("--camera", required=True, choices=["camera_1", "camera_2"])
    args = ap.parse_args()

    with open(args.config) as f:
        cfg = yaml.safe_load(f)
    cam_cfg = cfg["cameras"][args.camera]
    device = cam_cfg["device"]

    pasta = os.path.join("imagens", args.camera)
    os.makedirs(pasta, exist_ok=True)

    cap = cv2.VideoCapture(device)
    if not cap.isOpened():
        raise SystemExit(
            f"Não abriu '{device}'. Confira com: ls -l /dev/v4l/by-id/"
        )
    cap.set(cv2.CAP_PROP_FRAME_WIDTH, cam_cfg.get("width", 640))
    cap.set(cv2.CAP_PROP_FRAME_HEIGHT, cam_cfg.get("height", 480))

    fotos = 0
    ultima = time.time()
    print(f"[{args.camera}] {LIMITE_FOTOS} fotos a cada {INTERVALO}s. 'q' interrompe.")
    print("Mova o tabuleiro: perto/longe, inclinado, cantos do campo de visão.")

    while fotos < LIMITE_FOTOS:
        ret, frame = cap.read()
        if not ret:
            break

        preview = frame.copy()
        cv2.putText(preview, f"{args.camera}  Foto: {fotos}/{LIMITE_FOTOS}",
                    (20, 50), cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 255, 0), 2)
        cv2.imshow("Captura Chessboard", preview)

        if time.time() - ultima >= INTERVALO:
            nome = os.path.join(pasta, f"img_{fotos:02d}.jpg")
            cv2.imwrite(nome, frame)
            print(f"Salva: {nome}")
            fotos += 1
            ultima = time.time()

        if cv2.waitKey(1) & 0xFF == ord("q"):
            break

    cap.release()
    cv2.destroyAllWindows()
    print("Captura finalizada.")


if __name__ == "__main__":
    main()
