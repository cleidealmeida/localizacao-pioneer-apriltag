# Localização de Robô Móvel por Triangulação com Múltiplas Câmeras e AprilTag

Projeto PIBIC/CNPq 2025-2026 — NERO/UFV. Continuação do sistema de controle
multimodal por gestos e emoções (PIBIC 2023-2025), adicionando localização
absoluta por visão computacional (2 câmeras + AprilTag + Filtro de Kalman),
substituindo a odometria pura do Pioneer 3-DX.



## Arquitetura

Duas máquinas físicas na mesma rede:

```
[Ubuntu 22.04 · ROS 2 Humble]                     [Windows · MATLAB]
apriltag_detector ──/apriltag/cam_N/pose_raw──▶ pose_estimator
pose_estimator    ──/robot_pose───────────────────────────────▶ main_controller.m (v4)
gesture_emotion   ──/Gesture──────────────────────────────────▶ main_controller.m
                                            main_controller.m ──/cmd_vel──▶ Pioneer 3-DX
Pioneer ──/pose (odometria)──▶ pose_estimator e main_controller
OptiTrack ──(MATLAB publica /optitrack_pose)──▶ visualizador web (ground truth)
```

- Conexão com o Pioneer: **RosAria via Jetson** (`robot_connection='rosaria'`)
  ou **ARIA serial direto** (`'aria'`, com a odometria republicada em /pose).
- Visualizador web (`docs/simulador_arena_nero_v4.html`): 4 modos —
  Simulação, Ao vivo (rosbridge), Robô virtual (software-in-the-loop) e
  Replay (CSV de trajetória exportado pelo controlador).

## Estrutura

```
python_ros2/
├── calibration/         # capture.py, calib.py, reverse_localization.py
├── apriltag_detector/   # pacote ROS2: detecção crua por câmera (tag36h11)
├── pose_estimator/      # transformações + triangulação + Kalman -> /robot_pose
│   └── tests/           # test_kalman.py (roda no CI)
├── gesture_emotion/     # KNN gestos + CNN emoções -> /Gesture
└── launch/              # nero_perception.launch.py (sobe os 3 nós)
matlab_control/          # main_controller.m v4 + classes (@Pioneer3DX, @JoyControl,
                         # optitrack/, aria/, validation/@AprilTag) — ver README local
config/
└── cameras.example.yaml # config central: tags, robot_T_tag, câmeras
network/SETUP.md         # rede, firewall, chrony, rosbridge
docs/                    # plano PIBIC, visualizador web, relatórios
.github/workflows/ci.yml # roda test_kalman.py a cada push
```

## Roteiro de integração

Seguir `python_ros2/launch/README.md` (etapas 0–7): rede → calibração →
detecção → fusão → gestos → MATLAB → malha fechada suspensa → experimentos.

## Resultado de referência (simulação offline do filtro)

| Fonte | RMSE posição |
|---|---|
| Odometria pura | 7,12 cm |
| Medição crua de câmera | 2,86 cm |
| **Kalman (fusão)** | **0,97 cm** |

(60 s, robô em círculo, deslizamento 2%, 8 s de oclusão total — ver
`python_ros2/pose_estimator/tests/test_kalman.py`.)
