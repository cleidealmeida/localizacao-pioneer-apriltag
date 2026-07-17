# Parte 3 — pose_estimator (triangulação + Filtro de Kalman)

O cérebro da localização. Implementa os Módulos 2 e 3 do plano de trabalho:

```
/apriltag/camera_1/pose_raw ─┐
                              ├─> encadeamento W_T_C·C_T_tag·(robot_T_tag)⁻¹
/apriltag/camera_2/pose_raw ─┘        │
                                      v
/pose (odometria Pioneer) ──> [ Filtro de Kalman ] ──> /robot_pose (20 Hz)
        predição                 atualização            PoseWithCovarianceStamped
```

## Como a triangulação e o Kalman se relacionam aqui

O plano prevê "média ponderada pelas incertezas" das duas câmeras seguida do
FK. A implementação faz a fusão de forma **sequencial**: cada medição de cada
câmera atualiza o filtro com sua própria covariância R (dependente da
distância câmera→tag). Matematicamente, duas atualizações sequenciais com R₁ e
R₂ equivalem à média ponderada por inverso da variância — é a triangulação do
plano, executada dentro do próprio filtro, com o bônus de funcionar igual
quando só uma câmera detecta (oclusão parcial).

## Validação já executada (sem hardware)

`tests/test_kalman.py` simula 60 s de robô em círculo com odometria derivando
(deslizamento de 2%), duas câmeras ruidosas a distâncias diferentes e **8 s de
oclusão total**. Resultado desta implementação:

| Fonte | RMSE posição |
|---|---|
| Odometria pura | 7,12 cm |
| Medição crua de câmera | 2,86 cm |
| **Kalman (fusão)** | **0,97 cm** |

Erro máximo durante a oclusão total: 2,58 cm (continuidade por predição ✓);
reconvergência após oclusão: 0,05 s. Rode você mesma:

```bash
cd tests && python3 test_kalman.py
```

## Recursos do filtro (arquivo kalman.py)

- EKF planar, estado `[x, y, θ]` (Eq. 3.6 do plano), predição não-linear por
  incrementos de odometria com jacobiano (Eq. 3.7), atualização direta (Eq. 3.8);
- ruído de processo **proporcional ao movimento** (parado não acumula incerteza);
- ruído de medição **cresce com a distância²** câmera→tag (Seção 3.5.1);
- **gate de Mahalanobis** (χ², 99%): rejeita detecções espúrias/reflexos;
- forma de Joseph na covariância (estabilidade numérica);
- inicialização pela primeira medição de câmera (não pela origem da odometria).

## Instalação e execução (máquina Ubuntu)

```bash
# 1. Acrescentar robot_T_tag ao cameras.yaml
cat config/robot_T_tag_addition.yaml >> /caminho/config/cameras.yaml
# (e MEDIR com trena se a tag ainda está a -0.165 m/+0.145 m do centro do robô)

# 2. Compilar
cp -r pose_estimator ~/ros2_ws/src/
cd ~/ros2_ws && colcon build --packages-select pose_estimator --symlink-install
source install/setup.bash

# 3. Rodar (exige apriltag_detector no ar e extrínsecos já calibrados)
ros2 run pose_estimator pose_estimator_node --ros-args \
    -p config_file:=/caminho/absoluto/config/cameras.yaml
```

Parâmetros de afinação (TR 4.3 do cronograma) expostos via ROS:
`sigma_xy_process`, `sigma_theta_process`, `alpha_motion`,
`sigma_xy_camera`, `sigma_theta_camera`, `k_dist_camera`, `odom_topic`.

## Validar SOZINHO (antes da parte 4)

```bash
ros2 topic echo /robot_pose --field pose.pose.position
ros2 topic hz /robot_pose            # ~20 Hz
ros2 topic echo /apriltag/camera_1/pose_world   # medição por câmera no mundo
```

Testes físicos sugeridos:
1. **Estático**: robô parado num ponto medido com trena — a média de
   /robot_pose deve bater com a trena em ~1–3 cm; o σx logado a cada 10 s
   deve ser estável.
2. **Oclusão**: tape uma câmera — /robot_pose continua (só a outra atualiza);
   tape as duas — a pose passa a evoluir só pela odometria, sem saltos.
3. **Consistência entre câmeras**: /apriltag/camera_1/pose_world e
   /apriltag/camera_2/pose_world devem concordar entre si em poucos cm; se
   discordarem muito, os extrínsecos de uma delas estão ruins — refazer
   reverse_localization.

## Tópicos para os gráficos do artigo

Grave tudo com `ros2 bag record /pose /robot_pose /apriltag/camera_1/pose_world
/apriltag/camera_2/pose_world` e compare offline: odometria pura vs medições
de visão vs estimativa fundida — a mesma análise de RMSE que o script MATLAB
antigo fazia, agora com dados de tópicos padronizados.

## Onde colocar no repositório

```
python_ros2/pose_estimator/   <- pacote
python_ros2/pose_estimator/tests/ (ou tests/ na raiz)
config/cameras.yaml           <- acrescentar robot_T_tag
```
