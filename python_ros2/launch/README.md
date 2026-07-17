# Parte 5 — Launch unificado + Roteiro de Integração Ponta a Ponta

Última parte: como subir o sistema COMPLETO e verificar cada elo da cadeia,
na ordem certa, isolando falhas de rede, calibração, percepção e controle.

## Visão final dos tópicos

```
[Ubuntu 22.04 / ROS 2 Humble]                       [Windows / MATLAB]
apriltag_detector ──/apriltag/camera_N/pose_raw──> pose_estimator
pose_estimator    ──/robot_pose──────────────────────────────────> main_controller.m
gesture_emotion   ──/Gesture─────────────────────────────────────> main_controller.m
                                                     main_controller.m ──/cmd_vel──> Pioneer (RosAria)
Pioneer (RosAria) ──/pose (odometria)──> pose_estimator  e  main_controller.m
```

## Subir tudo (operação normal)

```bash
# Máquina Ubuntu — terminal 1
cd ~/ros2_ws && source install/setup.bash
ros2 launch /caminho/repo/python_ros2/launch/nero_perception.launch.py \
    config_file:=/caminho/repo/config/cameras.yaml \
    gesture_camera:=/dev/v4l/by-id/usb-PCYes_...-video-index0
```

```matlab
% Máquina Windows — MATLAB
% (RosAria/driver do Pioneer já rodando conforme seu setup atual)
main_controller
```

Os nós de visão têm `respawn=true`: se um cair (câmera desconectada etc.), o
launch o reinicia em 2 s. O nó de gestos não tem respawn de propósito — se a
janela de vídeo fechar, é ação do operador ou erro que merece investigação.

## Roteiro de integração (primeira vez / depois de mudanças)

Execute NA ORDEM. Cada etapa só faz sentido com a anterior verde.
Tempo estimado total na primeira vez: 1 tarde.

### Etapa 0 — Rede (network/SETUP.md)
```bash
# Ubuntu
ros2 topic pub /teste std_msgs/String "data: 'ola'" --rate 1
```
```matlab
% MATLAB
node = ros2node('/sanity'); ros2("topic","list")
```
✅ `/teste` aparece na lista e mensagens chegam.
❌ Não aparece → firewall do Windows / multicast do roteador / ROS_DOMAIN_ID
   / ROS_LOCALHOST_ONLY. Não avance sem resolver.

### Etapa 1 — Calibração (parte 2, uma vez)
✅ `cameras.yaml` com intrínsecos (erro reprojeção < 1 px), extrínsecos
   (desvio < 10 mm no reverse_localization) e `robot_T_tag` preenchidos;
   tamanhos das tags MEDIDOS com régua.

### Etapa 2 — Detecção crua
```bash
ros2 run apriltag_detector apriltag_detector_node --ros-args -p config_file:=...
ros2 topic hz /apriltag/camera_1/pose_raw     # ~20 Hz com tag visível
```
✅ Taxa de detecção logada > 90% com a tag parada e visível; `z` do pose_raw
   bate com a trena (sanidade do tamanho da tag).

### Etapa 3 — Fusão
```bash
ros2 run pose_estimator pose_estimator_node --ros-args -p config_file:=...
ros2 topic echo /robot_pose --field pose.pose.position
```
✅ Posição estável (~cm) com robô parado; as duas `pose_world` concordam;
   tapando uma câmera a estimativa continua.
⚠️ Sem odometria ainda (RosAria desligado), a predição não roda — normal: a
   pose só atualiza quando há detecção. Com o Pioneer ligado, o teste de
   oclusão total passa a manter a pose evoluindo.

### Etapa 4 — Gestos
```bash
ros2 run gesture_emotion gesture_emotion_node --ros-args -p video_source:=...
ros2 topic echo /Gesture
```
✅ Gesto + emoção GOOD sustentada => mensagem no echo.

### Etapa 5 — MATLAB recebendo (sem robô)
Rode só a Seção 1 do main_controller.m.
✅ Console imprime `Gesto "A" recebido. Fila: 1` ao gesticular;
   `sub_robot_pose.LatestMessage` não-vazio.

### Etapa 6 — Malha fechada segura
Pioneer SUSPENSO (rodas fora do chão), script completo.
✅ /cmd_vel reage aos gestos; robô "para" ao esvaziar a fila.

### Etapa 7 — Experimentos do artigo
Robô no chão. Mesma sequência de gestos em dois cenários:
`use_fused_pose=false` e depois `=true`. Gravar em paralelo:
```bash
ros2 bag record /pose /robot_pose /apriltag/camera_1/pose_world \
    /apriltag/camera_2/pose_world /Gesture /cmd_vel
```
✅ CSV + .fig do MATLAB + rosbag => material completo p/ análise de RMSE
   (odometria vs fusão vs ground truth do OptiTrack, quando usado).

## Diagnóstico rápido de problemas comuns

| Sintoma | Causa provável | Onde olhar |
|---|---|---|
| MATLAB não vê nenhum tópico | firewall/multicast/DOMAIN_ID | Etapa 0 |
| /robot_pose nunca publica | tag não detectada OU extrínsecos ausentes | logs do pose_estimator (ele acusa yaml incompleto) |
| Pose salta metros | extrínsecos ruins de UMA câmera | comparar as duas /pose_world; refazer reverse_localization |
| Pose com escala errada | robot_tag_size errado no yaml | teste da trena (Etapa 2) |
| Gestos não chegam no MATLAB | gesture_emotion caiu OU rede | echo /Gesture no Ubuntu primeiro (isola o lado) |
| Deriva mesmo com fusão | use_fused_pose=false esquecido | cabeçalho do main_controller.m |

## Encerramento do projeto de código

Com as 5 partes no repositório, os itens de software dos PAs 2–5 do plano de
trabalho estão implementados. O que resta é trabalho de laboratório
(calibração, ajuste de Q/R com dados reais — parâmetros já expostos via ROS —
e os experimentos comparativos) e a redação.
