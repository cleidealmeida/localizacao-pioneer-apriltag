# Parte 4 — matlab_control (revisão do controlador)

`main_controller.m` reescrito para a nova arquitetura: o MATLAB deixa de fazer
visão e vira **consumidor puro** de localização + orquestrador de controle.

## Bug corrigido (importante)

Na versão antiga, o comentário dizia "USANDO CALLBACK", mas o subscriber era
criado assim:

```matlab
sub_gesture = ros2subscriber(node, '/Gesture', 'std_msgs/String', qos_profile_gesture);
```

**sem passar `@gestureCallback`** — a função existia no fim do arquivo e nunca
era registrada. Resultado: no modo ROS, a fila de gestos jamais recebia nada
(só funcionava com a lista fixa de teste). Corrigido para:

```matlab
sub_gesture = ros2subscriber(node, '/Gesture', 'std_msgs/String', ...
    @gestureCallback, qos_gesture);
```

Esse era um candidato forte a causa do "não está se comunicando muito bem".

## Demais mudanças

| Antes | Agora |
|---|---|
| `webcam(1)` + `readAprilTag` no loop, `w_H_c`/`r_H_t` hardcoded, `cameraParamsAtualizado.mat` | Removidos — a visão inteira roda no Python; o MATLAB assina `/robot_pose` |
| Profile FastDDS de WSL2 apontando p/ Desktop | Removido — 2 máquinas físicas em LAN usam descoberta multicast padrão |
| `use_vision_correction` | `use_fused_pose` (cenário 1 = odometria; cenário 2 = Kalman) |
| `timing_log_gesture` (nunca preenchido, plotava zeros) | Removido; mantido só o tempo de ciclo real do controlador |
| RMSE "odometria vs visão" | RMSE "odometria vs fusão" (mede a deriva) + erro de rastreamento médio no CSV |

O que **não** mudou: a fila sequencial de gestos, `iProcessarEntrada`,
`cNewController`, `Pioneer3DX`, o limiar de chegada (3 cm), o período de
controle (0,1 s) e o formato geral dos gráficos — para manter comparabilidade
com seus experimentos anteriores.

## Comportamento de segurança adicionado

- Ao iniciar com `use_fused_pose = true`, o script espera até 15 s pela
  primeira mensagem de `/robot_pose` (o Kalman só publica após a primeira
  detecção da tag). Se não chegar, **degrada para modo odometria com aviso**
  em vez de rodar silenciosamente com pose de controle errada.
- A odometria continua sendo logada em paralelo em todos os cenários — é ela
  contra a fusão que gera a métrica de deriva do artigo.

## Pré-requisitos no Windows

- MATLAB com **ROS Toolbox** (suporte a ROS 2; Humble é suportado nas versões
  recentes da toolbox).
- Arquivos do robô no path: `Pioneer3DX.m`, `iProcessarEntrada.m`,
  `cNewController.m` (os mesmos que você já usa — não foram alterados).
- Firewall/rede conforme `network/SETUP.md` (teste talker/listener ANTES).

## Ordem de teste (integração progressiva)

1. **Rede**: teste do `network/SETUP.md` (tópico `/teste` visível no MATLAB).
2. **Gestos**: com o `gesture_emotion` rodando no Ubuntu, rode só a Seção 1
   deste script no MATLAB e faça um gesto — o `gestureCallback` deve imprimir
   `Gesto "A" recebido. Fila: 1`. Isso valida o bug corrigido.
3. **Pose**: com `apriltag_detector` + `pose_estimator` no ar e a tag visível,
   confira `sub_robot_pose.LatestMessage` não-vazio no MATLAB.
4. **Malha fechada sem robô**: rode o script completo com o Pioneer suspenso
   (rodas fora do chão) — o `/cmd_vel` deve reagir aos gestos sem risco.
5. **Experimentos do artigo**: rode uma sessão com `use_fused_pose = false` e
   outra `= true`, mesma sequência de gestos, e compare os `.fig` e o CSV.

## Onde colocar no repositório

```
matlab_control/main_controller.m   <- substitui o script antigo
```
(mantenha o antigo como main_controller_legacy.m se quiser referência)
