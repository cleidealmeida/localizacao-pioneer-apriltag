# apriltag_detector

Nó ROS2 que lê as 2 câmeras de parede, detecta a AprilTag (família `tag36h11`)
acoplada ao robô e publica as leituras **cruas por câmera** (não a pose já
fundida — a fusão é responsabilidade do `pose_estimator`).

Carrega os extrínsecos de `../../config/camera_extrinsics.yaml` em vez de
matrizes hardcoded no código.
