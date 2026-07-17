# calibration

Calibração intrínseca das câmeras de parede + localização reversa (obtenção de `W_T_Ci`).

- `capture.py` — captura fotos do tabuleiro de xadrez para calibração.
- `calib.py` — calibração intrínseca (matriz K + distorção), salva `calib.npz`.
- `reverse_localization.py` — lê a AprilTag de referência na origem e calcula a
  pose da câmera no mundo. Deve salvar o resultado em
  `../../config/camera_extrinsics.yaml` (não mais imprimir para copiar/colar).
