# matlab_control — pacote COMPLETO (roda na máquina Windows)

Esta pasta contém TUDO que o main_controller.m precisa para rodar — scripts e
classes. Substitui integralmente a matlab_control/ do repositório.

## Conteúdo

| Item | Papel | Obrigatório? |
|---|---|---|
| main_controller.m (v4) | controlador principal | sim |
| iProcessarEntrada.m / cNewController.m | mapeamento gesto→alvo e lei de controle | sim |
| @Pioneer3DX/ | classe do robô (versão com rConnect/rGetSensorData — métodos ARIA) | sim |
| controllers/ | controladores cP3DX_* do AuRoRA (referência/base) | não |
| aria/ | mex + dlls da conexão serial direta | só no modo 'aria' |
| @JoyControl/ | e-stop por joystick Xbox (requer Simulink 3D Animation) | só se use_joystick_estop |
| optitrack/ | toolbox NatNet + funções AuRoRA | só se use_optitrack |
| validation/@AprilTag/ | validação cruzada da calibração (NÃO é runtime) | não |

## Setup no MATLAB

Adicionar esta pasta (com subpastas) ao path:
```matlab
addpath(genpath('caminho/ate/matlab_control'))
```
As pastas @Pioneer3DX e @JoyControl são pastas de CLASSE: precisam estar com
o diretório PAI no path (o genpath acima resolve).

IMPORTANTE — manter só UMA @Pioneer3DX no path. Se você tiver a versão antiga
da classe (a do primeiro zip, com properties ROS) em outro lugar do path,
remova: duas classes de mesmo nome causam sombreamento silencioso.

## Higiene aplicada neste pacote

- Removida a @ArDrone que estava aninhada dentro de @Pioneer3DX (posição
  inválida para classe MATLAB; se precisar do drone, vira pasta irmã).
- Removida a cópia desativada !Pioneer3DX.
- aria/ sem artefatos de build (obj/, .pdb, .tlog).
- optitrack/ sem arquivos .cal (calibração é do ambiente físico, refazer no
  Motive local).
