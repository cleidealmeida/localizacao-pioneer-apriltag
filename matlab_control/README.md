# matlab_control

Roda na máquina Windows. `main_controller.m` assina `/robot_pose` e
`/Gesture`, publica `/cmd_vel`, e troca odometria bruta via `/pose` com o
Pioneer (RosAria). Não deve mais fazer leitura de câmera/AprilTag própria —
isso passa a ser responsabilidade exclusiva do lado Python.
