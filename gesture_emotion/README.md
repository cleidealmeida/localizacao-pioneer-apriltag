# gesture_emotion — pacote ROS 2 (Humble)

Reconhecimento de **gestos** (MediaPipe + KNN, 5 classes: A–E) com validação
por **emoção** (CNN ResNet18/ONNX): um gesto só é publicado quando o operador
demonstra emoção `GOOD` por 10 frames consecutivos (dupla validação).

Publica: **`/Gesture`** (`std_msgs/String`) — assinado pelo `robot_controller.m`
no MATLAB via `ros2subscriber`.

## O que mudou em relação à versão anterior

| Item | Antes | Agora |
|---|---|---|
| `utils/utils.py` | vazio (perdido no .rar) | **recuperado do bytecode** do `.pyc` — `setup_logger`, `to_numpy`, `calculate_winner` |
| Fonte de vídeo | URL RTSP hardcoded no `main_ros.py` | parâmetro ROS 2 `video_source` (default: `/dev/v4l/by-id/...`) |
| Caminho p/ MATLAB | 2 caminhos (ROS2 + relay UDP porta 9999) | **só ROS2 nativo**; `matlab_relay.py` movido p/ `legacy/` |
| Estrutura | pasta solta | pacote ament_python completo (`package.xml`, `setup.py`, entry point) |
| Arquivos mortos | `main.py`, `GestureDetector.py`, `gesture_publisher.sh` (todos vazios) | removidos |

## Instalação (na máquina Ubuntu 22.04 / ROS 2 Humble)

```bash
# 1. Dependências Python (fora do rosdep)
pip install -r requirements.txt

# 2. Copiar o pacote para o workspace colcon
cp -r gesture_emotion ~/ros2_ws/src/

# 3. Compilar
cd ~/ros2_ws
colcon build --packages-select gesture_emotion --symlink-install
source install/setup.bash
```

## Descobrir o caminho estável da webcam PCYes

```bash
ls -l /dev/v4l/by-id/
# exemplo de saída:
#   usb-PCYes_HD_Webcam_PCYes_HD_Webcam-video-index0 -> ../../video2
```

Use o caminho `by-id` (não `/dev/video2`): ele **não muda** entre boots/replugs.

## Rodar

```bash
# com o caminho estável da webcam (recomendado)
ros2 run gesture_emotion gesture_emotion_node --ros-args \
    -p video_source:="/dev/v4l/by-id/usb-PCYes_HD_Webcam...-video-index0"

# ou, para um teste rápido, por índice
ros2 run gesture_emotion gesture_emotion_node --ros-args -p video_source:="0"
```

Parâmetros disponíveis: `video_source`, `model_name`, `model_option`,
`backend_option` (1=CPU, 2=CUDA), `providers`, `num_faces`, `knn_k`,
`train_path`.

## Validar SOZINHO (antes de integrar com o MATLAB)

Em um segundo terminal na mesma máquina Ubuntu:

```bash
source ~/ros2_ws/install/setup.bash
ros2 topic echo /Gesture
```

Faça um gesto (A–E) para a câmera e mantenha expressão positiva; quando o
contador de emoção `GOOD` zerar, a mensagem deve aparecer no echo:

```
data: 'A'
---
```

Se isso funciona, o lado Ubuntu está pronto. A validação seguinte (MATLAB
recebendo pela rede) usa o teste descrito em `../../network/SETUP.md`.

## Estrutura

```
gesture_emotion/
├── package.xml / setup.py / setup.cfg / resource/   # empacotamento ROS2
├── requirements.txt
├── legacy/matlab_relay.py                            # NÃO usar (ver legacy/README)
└── gesture_emotion/
    ├── gesture_emotion_node.py                       # entry point (substitui main_ros.py)
    └── Emotion_GestureDetector/
        ├── EmotionGestureCompiler.py                 # orquestra gesto+emoção
        ├── GestureDetector_rs.py                     # KNN + MediaPipe
        ├── emotion_detector.py                       # CNN ONNX + detector de face
        ├── main_face.py                              # teste isolado só de emoção
        ├── utils/ (utils.py RECUPERADO, Mconfusao.py)
        ├── models/ (resnet18.onnx, face_detector caffe)
        └── Base_de_dados/ (xlsx de treino dos gestos A–E)
```

## Notas

- `backend_option` default agora é **1 (CPU)** para rodar em qualquer máquina;
  mude para 2 se a máquina Ubuntu tiver GPU NVIDIA com CUDA configurado.
- `GestureDetector_rs.saves_to_dataBase()` grava xlsx com caminho relativo ao
  diretório de execução — só relevante se você for coletar novas amostras de
  treino; rode a partir da raiz do pacote nesse caso.
- A janela de vídeo (`cv2.imshow`) exige sessão gráfica; em execução headless
  futura, remover/condicionar o `imshow` no `EmotionGestureCompiler.video()`.
