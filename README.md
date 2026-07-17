# Localização de Robô Móvel por Triangulação com Múltiplas Câmeras e AprilTag

Projeto PIBIC/CNPq 2025-2026 — NERO/UFV. Continuação do sistema de controle multimodal
por gestos e emoções (PIBIC 2023-2025), adicionando uma camada de localização absoluta
por visão computacional, substituindo a odometria pura do Pioneer 3-DX.

Orientadora: Kétia Soares Moreira · Bolsista: Cleide Almeida Coelho Fernandes

## Arquitetura

Duas máquinas físicas na mesma rede:

- **Percepção** — Ubuntu 22.04 + ROS 2 Humble: três nós Python publicando via ROS2.
- **Controle** — Windows + MATLAB: um único nó consumindo os tópicos e comandando o robô.

```
percepção (Ubuntu/ROS2 Humble)              controle (Windows/MATLAB)
┌─────────────────────────────┐             ┌───────────────────────┐
│ apriltag_detector            │             │                       │
│   2 câmeras + tag36h11       │             │                       │
│           │                  │             │                       │
│           v                  │  /robot_pose│                       │
│ pose_estimator ───────────────┼────────────>│  robot_controller.m   │
│   triangulação + Kalman      │             │  fila de gestos +     │
│                               │             │  controle             │
│ gesture_emotion ───────────────┼─── /Gesture ─>│                       │
│   KNN gestos + CNN emoção    │             │                       │
└─────────────────────────────┘             └──────────┬────────────┘
                                                          │  /cmd_vel
                                                          v
                                              ┌───────────────────────┐
                                              │   Pioneer 3-DX         │
                                              │   (robô físico,       │
                                              │    RosAria)            │
                                              └───────────────────────┘
                                                     │
                                                     └── /pose (odometria) ──> robot_controller.m
```

Ver `network/SETUP.md` para configuração de rede entre as duas máquinas.

## Estrutura do repositório

```
projeto-localizacao-pioneer/
├── python_ros2/            # roda inteira na máquina Ubuntu 22.04 / ROS Humble
│   ├── calibration/        # calib.py, capture.py, reverse_localization.py
│   ├── apriltag_detector/  # leitura das 2 câmeras + detecção AprilTag
│   ├── pose_estimator/     # transformações homogêneas + triangulação + Kalman
│   ├── gesture_emotion/    # classificador de gestos (KNN) + emoções (CNN)
│   └── launch/             # launch file único sobe os 3 nós juntos
├── matlab_control/         # roda na máquina Windows
│   ├── main_controller.m
│   └── Pioneer3DX.m / cNewController.m / iProcessarEntrada.m
├── network/
│   └── SETUP.md            # IPs, ROS_DOMAIN_ID, firewall, teste de multicast
├── config/
│   └── camera_extrinsics.example.yaml
├── docs/
│   └── (plano de trabalho PIBIC, artigos, relatórios)
└── README.md
```

## Status

- [ ] Rede validada entre as duas máquinas (talker/listener de teste)
- [ ] Calibração intrínseca das 2 câmeras de parede
- [ ] Localização reversa (extrínsecos `W_T_Ci`)
- [ ] `apriltag_detector` publicando leituras cruas por câmera
- [ ] `pose_estimator` — transformações homogêneas + triangulação
- [ ] `pose_estimator` — Filtro de Kalman
- [ ] `gesture_emotion` — `utils/utils.py` recuperado e pacote validado sozinho
- [ ] Integração `/Gesture` + `/robot_pose` → `robot_controller.m`
- [ ] Testes físicos com o Pioneer 3-DX

## Cronograma

Alinhado aos Planos de Ação (PA1–PA5) do plano de trabalho PIBIC — ver `docs/`.
