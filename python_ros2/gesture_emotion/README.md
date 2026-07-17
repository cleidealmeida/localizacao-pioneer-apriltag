# gesture_emotion

Pacote `emotion_gesture_pkg` (classificador KNN de gestos + CNN de emoções,
com validação dupla). Publica `/Gesture` como `std_msgs/String`.

Pendências antes de rodar:
- Recuperar `utils/utils.py` (vazio na versão atual).
- Trocar a fonte de vídeo (hoje RTSP hardcoded) pela webcam PCYes 720p,
  referenciada por `/dev/v4l/by-id/...` em vez de índice numérico.
- Remover `matlab_relay.py` (caminho UDP legado, substituído pelo
  `ros2subscriber` nativo no MATLAB).
