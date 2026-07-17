# main_controller v3 — dupla via de conexão + OptiTrack ground truth

Substitui a v2. Novidades:

## 1. Chave `robot_connection`

| Valor | Como funciona | Requisitos |
|---|---|---|
| `'rosaria'` | /pose e /cmd_vel via ROS (Jetson intermediária) — igual à v2 | RosAria rodando na Jetson |
| `'aria'` | serial direto pelos mex do ARIA (`rConnect`, `rGetSensorData`, `rSendControlSignals`) | pasta ARIA no path do MATLAB; cabo serial/USB; RosAria da Jetson DESLIGADO |

No modo `'aria'`, a odometria lida do robô é **republicada em /pose** pelo
próprio MATLAB — assim o pose_estimator (Python) continua recebendo odometria
para a predição do Kalman, independente da via escolhida. Por segurança, o
script avisa: nunca rode os dois modos ao mesmo tempo (duas fontes de /pose e
duas atuações no robô = conflito).

IMPORTANTE (modo aria): a ordem no loop importa — `rGetSensorData` escreve a
odometria em `P.pPos.X`; a pose fundida sobrescreve `P.pPos.X` DEPOIS, antes
do controlador. Não reordenar esses blocos.

## 2. OptiTrack como ground truth (`use_optitrack`)

- `OPT = OptiTrack; OPT.Initialize;` e leitura de `OPT.RigidBody` a cada ciclo;
- usa a MESMA convenção do `getOptData` do AuRoRA (mm→m; yaw = -eul(1)),
  garantindo consistência com os demais códigos do laboratório;
- `opt_rb_index` seleciona o rigid body do Pioneer (conferir número no Motive);
- se o OptiTrack não estiver disponível, o script avisa e segue sem GT
  (fora da arena, basta `use_optitrack = false`).

Novas métricas no CSV e no console:
- **RMSE Fusão vs OptiTrack** ← o resultado central do artigo;
- RMSE Odometria vs OptiTrack (mostra a deriva contra referência milimétrica);
- gráfico novo de trajetória no plano (odometria × fusão × GT).

## 3. Mantido da v2

Watchdog de /robot_pose (para o robô se a pose ficar obsoleta > 1 s),
e-stop por joystick (botão B), correção do callback de /Gesture, cenários
`use_fused_pose`, espera inicial com degradação segura.

## Classe Pioneer3DX correta

Use a classe do pacote "Pioneer 3DX" (a que tem `rConnect`/`rGetSensorData`/
`rSendControlSignals`). A versão do primeiro zip (com properties ROS) e a
cópia `!Pioneer3DX` NÃO devem ir para o repo — manter só UMA @Pioneer3DX no
path para não haver sombreamento de classe.

## Checklist antes do primeiro experimento com GT

1. Rigid body do Pioneer criado no Motive e `opt_rb_index` correto.
2. Origem/eixos do OptiTrack alinhados com a origem do MUNDO usada na
   localização reversa (a tag de referência) — senão o RMSE vs GT embute um
   offset constante de desalinhamento de frames. Dica: coloque a tag de
   referência na origem do OptiTrack.
3. `iProcessarEntrada.m` e `cNewController.m` no path (ainda não estão nos
   uploads do repositório!).
