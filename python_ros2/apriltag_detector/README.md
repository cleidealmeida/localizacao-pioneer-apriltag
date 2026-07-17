# Parte 2 — apriltag_detector + calibração

Dois entregáveis que trabalham juntos através do arquivo central
`config/cameras.yaml`:

1. **`calibration/`** — scripts standalone (não-ROS) que PREENCHEM o yaml.
2. **`apriltag_detector/`** — pacote ROS 2 que LÊ o yaml e publica as leituras
   cruas da tag do robô, por câmera.

```
cameras.yaml  <-- capture.py + calib.py     (intrínsecos, por câmera)
              <-- reverse_localization.py    (extrínsecos W_T_C, por câmera)
              --> apriltag_detector_node     (usa intrínsecos + devices)
              --> pose_estimator (parte 3)   (usará extrínsecos p/ encadeamento)
```

## Correções aplicadas sobre o código antigo

| Problema | Correção |
|---|---|
| `families='tag36h1'` (typo, família inexistente) | `tag36h11` unificado, vindo do yaml |
| Um `calib.npz` para as DUAS câmeras | calibração e intrínsecos por câmera |
| Extrínsecos copiados/colados no código | gravados automaticamente no yaml |
| Câmeras por índice 0/1 (muda a cada boot) | caminho estável `/dev/v4l/by-id/` |
| Publicava a MÉDIA das câmeras em `Float64MultiArray` | publica leitura CRUA por câmera em `PoseStamped` (fusão é da parte 3) |
| Tamanho de tag divergente (MATLAB 0.196 vs Python 0.119) | dois campos explícitos no yaml (`robot_tag_size`, `reference_tag_size`) — **medir com régua** |
| Sem filtro de id | filtra `robot_tag_id` / `reference_tag_id` (referência e robô não se confundem) |

## Passo a passo no laboratório (na máquina Ubuntu)

### 0. Preparar o yaml
```bash
cp config/cameras.example.yaml config/cameras.yaml
ls -l /dev/v4l/by-id/          # copie os caminhos das 2 câmeras de parede
nano config/cameras.yaml       # preencha device de camera_1/camera_2 + meça as tags
```

> A webcam PCYes do gesture_emotion NÃO entra aqui — são as 2 câmeras de parede.

### 1. Intrínsecos (uma vez por câmera)
```bash
cd calibration
python3 capture.py --config ../config/cameras.yaml --camera camera_1
python3 calib.py   --config ../config/cameras.yaml --camera camera_1
python3 capture.py --config ../config/cameras.yaml --camera camera_2
python3 calib.py   --config ../config/cameras.yaml --camera camera_2
```
O `calib.py` imprime o **erro de reprojeção**: abaixo de 0,5 px é bom; acima de
1 px, refaça as fotos (mais variação de ângulo/distância do tabuleiro).

### 2. Extrínsecos por localização reversa (uma vez por câmera)
Posicione a tag de referência (id e tamanho conforme o yaml) exatamente na
origem do sistema de coordenadas do mundo, visível pelas duas câmeras:
```bash
python3 reverse_localization.py --config ../config/cameras.yaml --camera camera_1
python3 reverse_localization.py --config ../config/cameras.yaml --camera camera_2
```
O script tira a média de 15 detecções e reporta o desvio-padrão — abaixo de
10 mm está estável. **Refazer sempre que uma câmera for movida.**

### 3. Compilar e rodar o nó
```bash
cp -r apriltag_detector ~/ros2_ws/src/
pip install -r apriltag_detector/requirements.txt
cd ~/ros2_ws && colcon build --packages-select apriltag_detector --symlink-install
source install/setup.bash

ros2 run apriltag_detector apriltag_detector_node --ros-args \
    -p config_file:=/caminho/absoluto/para/config/cameras.yaml
```

### 4. Validar SOZINHO (antes da parte 3)
Coloque a tag do robô (ou o robô) no campo de visão e, em outro terminal:
```bash
ros2 topic echo /apriltag/camera_1/pose_raw
ros2 topic hz   /apriltag/camera_1/pose_raw   # deve ficar próximo de 20 Hz
```
O nó também loga a cada 10 s a taxa de detecção de cada câmera (ex: `18/200
(9%)` significa oclusão ou problema de foco/iluminação — investigue antes de
seguir).

**Teste de sanidade geométrica**: com a tag parada a ~2 m da câmera, o `z` do
`pose_raw` (distância tag→câmera) deve bater com uma trena. Se estiver
proporcionalmente errado, o `robot_tag_size` do yaml está errado — é o sintoma
clássico da divergência 0.196 vs 0.119 herdada dos códigos antigos.

## Onde colocar no repositório

```
python_ros2/calibration/        <- conteúdo de calibration/
python_ros2/apriltag_detector/  <- conteúdo de apriltag_detector/
config/cameras.example.yaml     <- substitui o camera_extrinsics.example.yaml
```
E adicione `config/cameras.yaml` ao .gitignore (já coberto pelo padrão
`config/camera_extrinsics.yaml`? não — adicione a linha `config/cameras.yaml`).
