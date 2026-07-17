# pose_estimator

Recebe as leituras cruas do `apriltag_detector`, faz o encadeamento de
transformações homogêneas (`W_T_Ci · Ci_T_AR`), a triangulação das duas
câmeras e o Filtro de Kalman (predição por odometria + atualização por
câmera). Publica `/robot_pose` como `geometry_msgs/PoseWithCovarianceStamped`.
