# Visualizador 3D da Arena — manual de uso

Arquivo: `docs/simulador_arena_nero.html`. Abre em qualquer navegador
(duplo-clique); precisa de internet só na primeira abertura (bibliotecas 3D).

Navegação: arrastar = orbitar · scroll = zoom · shift+arrastar = mover ·
espaço = pausa (na Simulação).

A configuração (sala, câmeras, tópicos, IP) é salva automaticamente no
navegador; "Copiar link" gera uma URL com a config embutida para compartilhar.

## Os 4 modos

### 1. Simulação (padrão — não precisa de nada instalado)
Robôs virtuais (1–6) navegam pelos alvos dos gestos com o mesmo controlador
do MATLAB. Use para validar cobertura das câmeras e geometria da sala ANTES
de montar fisicamente. Edite sala/origem/câmeras à vontade.

### 2. Ao vivo (requer o sistema real + rosbridge)
```bash
# Ubuntu (uma vez): sudo apt install ros-humble-rosbridge-suite
ros2 launch rosbridge_server rosbridge_websocket_launch.xml
```
No site: `ws://IP-DO-UBUNTU:9090` → Conectar. Mostra:
- robô ciano = fusão (/robot_pose) com elipse de covariância do Kalman;
- fantasma cinza = odometria (/pose) — a distância entre eles é a deriva;
- robô branco = OptiTrack (/optitrack_pose, publicado pelo main_controller v4);
- frustums acendem por medição, com elipse 2σ por câmera (σ = σ_base + k·d²,
  ajuste σ_base/k para bater com a afinação do pose_estimator);
- banner âmbar de POSE OBSOLETA espelhando o watchdog do MATLAB;
- badges com Hz por câmera + painel de diagnóstico de rede (Hz por tópico).

### 3. Robô virtual (SIL — testa o pipeline SEM hardware)
O site vira o Pioneer: assina /cmd_vel do MATLAB, publica /pose (odometria com
deslizamento/ruído configuráveis) e as medições pose_raw das câmeras.
Roteiro de teste completo:
1. rosbridge no ar (Ubuntu);
2. site em "Robô virtual" → Conectar;
3. `pose_estimator` rodando no Ubuntu — IMPORTANTE: com o MESMO cameras.yaml
   colado no site (senão a geometria diverge);
4. `main_controller.m` no MATLAB (use_ros_gestures=false p/ lista fixa);
5. o robô virtual deve navegar A→C→E→B→D comandado pelo MATLAB real.

### 4. Replay (revisão de experimentos)
Carrega o `trajetoria_*.csv` exportado pelo main_controller v4 e reproduz com
scrubber/velocidade: trilhas completas de odometria (cinza), fusão (ciano) e
OptiTrack (branco), mais o anel vermelho do alvo ativo em cada instante.
Formato esperado (separador , ou ;):
`t,x_od,y_od,psi_od,x_fu,y_fu,psi_fu,x_gt,y_gt,psi_gt,x_alvo,y_alvo`
(NaN é tolerado em trechos sem visão/GT/alvo.)

## Limitações conhecidas
- 1 robô nos modos Ao vivo/Virtual (multi-robô exigiria namespacing de
  tópicos nos nós Python).
- Replay não desenha elipses de covariância (o CSV não carrega covariância).
- Primeira abertura requer internet (CDN das bibliotecas).
