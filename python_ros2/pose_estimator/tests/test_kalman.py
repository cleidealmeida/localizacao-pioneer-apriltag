#!/usr/bin/env python3
"""test_kalman.py — validação offline do PlanarKalmanFilter (sem ROS).

Cenário simulado (60 s @ 20 Hz):
  - robô anda em círculo (raio 1 m, como nos experimentos do laboratório);
  - odometria com viés de deslizamento -> DERIVA acumulativa (o problema
    que o projeto quer resolver);
  - 2 câmeras com ruído gaussiano e taxa de detecção < 100%, distâncias
    diferentes (ruído de medição dependente de distância);
  - período de OCLUSÃO TOTAL (nenhuma câmera vê a tag) no meio do trajeto,
    para verificar a continuidade por predição exigida no plano (Módulo 3).

Critérios de aprovação:
  1. RMSE do Kalman << RMSE da odometria pura;
  2. RMSE do Kalman <= RMSE das medições cruas de câmera (filtra ruído);
  3. durante a oclusão, o erro não explode (continuidade via odometria).
"""
import numpy as np

import sys, os
# Funciona com tests/ dentro do pacote ROS2 (python_ros2/pose_estimator/tests/,
# com o código-fonte em pose_estimator/pose_estimator/) ou um nível acima disso.
_here = os.path.dirname(os.path.abspath(__file__))
for _cand in (
    os.path.join(_here, "..", "pose_estimator"),
    os.path.join(_here, ".."),
    os.path.join(_here, "..", ".."),
):
    if os.path.isfile(os.path.join(_cand, "pose_estimator", "kalman.py")):
        sys.path.insert(0, os.path.abspath(_cand)); break
from pose_estimator.kalman import PlanarKalmanFilter, camera_measurement_noise
from pose_estimator.transforms import wrap_angle

rng = np.random.default_rng(42)

DT = 0.05          # 20 Hz
T_TOTAL = 60.0
N = int(T_TOTAL / DT)
V = 0.15           # m/s
RADIUS = 1.0
W = V / RADIUS     # rad/s

# viés de odometria: rodas escorregando (2% linear, +ruído)
SLIP = 0.98
ODOM_NOISE_XY = 0.001
ODOM_NOISE_TH = 0.002

CAMERAS = {
    "camera_1": dict(dist=2.0, p_detect=0.85),
    "camera_2": dict(dist=3.5, p_detect=0.75),
}
CAM_NOISE_XY = 0.02    # 2 cm
CAM_NOISE_TH = 0.04    # ~2.3°
OCCLUSION = (30.0, 38.0)  # nenhuma câmera detecta neste intervalo

# --- simulação ---
kf = PlanarKalmanFilter()

true_pose = np.array([RADIUS, 0.0, np.pi / 2])
odom_pose = true_pose.copy()

err_kf, err_odom, err_cam = [], [], []
err_kf_occl = []
time_to_recover = None
post_occl_errs = []

for i in range(N):
    t = i * DT

    # verdade: círculo
    d_th = W * DT
    d_x = V * DT
    c, s = np.cos(true_pose[2]), np.sin(true_pose[2])
    true_pose[0] += c * d_x
    true_pose[1] += s * d_x
    true_pose[2] = wrap_angle(true_pose[2] + d_th)

    # odometria: mesmo movimento com deslizamento + ruído
    d_x_od = d_x * SLIP + rng.normal(0, ODOM_NOISE_XY)
    d_th_od = d_th * SLIP + rng.normal(0, ODOM_NOISE_TH)
    c, s = np.cos(odom_pose[2]), np.sin(odom_pose[2])
    odom_pose[0] += c * d_x_od
    odom_pose[1] += s * d_x_od
    odom_pose[2] = wrap_angle(odom_pose[2] + d_th_od)

    # Kalman: predição com o delta da ODOMETRIA (o que o robô "acha" que andou)
    kf.predict(d_x_od, 0.0, d_th_od)

    # câmeras
    occluded = OCCLUSION[0] <= t <= OCCLUSION[1]
    for cam in CAMERAS.values():
        if occluded or rng.random() > cam["p_detect"]:
            continue
        z = np.array([
            true_pose[0] + rng.normal(0, CAM_NOISE_XY),
            true_pose[1] + rng.normal(0, CAM_NOISE_XY),
            wrap_angle(true_pose[2] + rng.normal(0, CAM_NOISE_TH)),
        ])
        Rm = camera_measurement_noise(cam["dist"])
        kf.update(z, Rm)
        err_cam.append(np.hypot(z[0] - true_pose[0], z[1] - true_pose[1]))

    if kf.initialized:
        e_kf = np.hypot(kf.x[0] - true_pose[0], kf.x[1] - true_pose[1])
        err_kf.append(e_kf)
        err_odom.append(np.hypot(odom_pose[0] - true_pose[0],
                                 odom_pose[1] - true_pose[1]))
        if occluded:
            err_kf_occl.append(e_kf)
        if t > OCCLUSION[1] and time_to_recover is None:
            post_occl_errs.append((t, e_kf))
            if e_kf < 0.05:
                time_to_recover = t - OCCLUSION[1]

rmse = lambda v: float(np.sqrt(np.mean(np.square(v))))

r_kf, r_od, r_cam = rmse(err_kf), rmse(err_odom), rmse(err_cam)
print(f"RMSE posição — Odometria pura : {r_od*100:6.2f} cm")
print(f"RMSE posição — Medição câmera : {r_cam*100:6.2f} cm")
print(f"RMSE posição — Kalman (fusão) : {r_kf*100:6.2f} cm")
print(f"Erro máx. do KF durante oclusão total de {OCCLUSION[1]-OCCLUSION[0]:.0f}s: "
      f"{max(err_kf_occl)*100:.2f} cm")
print(f"Tempo p/ reconvergir (<5 cm) após oclusão: "
      f"{time_to_recover:.2f} s" if time_to_recover else "NÃO reconvergiu")
print(f"Medições rejeitadas pelo gate: {kf.n_rejected}")

assert r_kf < 0.5 * r_od, "KF deveria ser muito melhor que odometria pura"
assert r_kf <= r_cam * 1.05, "KF deveria filtrar o ruído das câmeras"
assert max(err_kf_occl) < 0.30, "erro na oclusão não pode explodir"
assert time_to_recover is not None and time_to_recover < 2.0
print("\n✅ TODOS OS CRITÉRIOS PASSARAM")
