"""server.py — servidor local (só 127.0.0.1) pro assistente de calibração no
navegador (docs/assistente_calibracao.html). Reaproveita a mesma matemática de
core.py que calib.py/reverse_localization.py já usam por linha de comando —
esse servidor só expõe isso via HTTP pra ser acionado por botões no site.

Uso:
    python server.py
    (ou) uvicorn server:app --host 127.0.0.1 --port 8000

NUNCA rode isso com --host 0.0.0.0 numa rede compartilhada: não há
autenticação, é pensado pra rodar na mesma máquina de quem está calibrando.
"""
import glob
import sys
import threading
import time
import uuid

import cv2
from fastapi import FastAPI, HTTPException
from fastapi.middleware.cors import CORSMiddleware
from fastapi.responses import StreamingResponse
from pydantic import BaseModel

from core import (
    PATTERN_SIZE,
    calibrate_camera,
    find_chessboard_corners,
    make_object_points,
    reprojection_quality,
)

HOST, PORT = "127.0.0.1", 8000

app = FastAPI(title="Assistente de calibração — servidor local")
app.add_middleware(
    CORSMiddleware,
    allow_origins=["*"],  # seguro aqui: só escuta em 127.0.0.1 (ver aviso acima)
    allow_methods=["*"],
    allow_headers=["*"],
)


class CameraSession:
    """Uma câmera aberta. Uma thread só fica lendo o frame mais recente
    (latest_frame); o stream MJPEG e os endpoints de captura/calibração leem
    dessa cópia em vez de chamar cap.read() cada um por conta própria (evita
    disputa pelo mesmo cv2.VideoCapture)."""

    def __init__(self, device, width, height):
        self.cap = cv2.VideoCapture(device)
        if not self.cap.isOpened():
            raise RuntimeError(f"Não abriu o dispositivo de vídeo '{device}'.")
        self.cap.set(cv2.CAP_PROP_FRAME_WIDTH, width)
        self.cap.set(cv2.CAP_PROP_FRAME_HEIGHT, height)
        self.lock = threading.Lock()
        self.latest_frame = None
        self.running = True
        self.objpoints = []
        self.imgpoints = []
        self.image_size = None
        self.thread = threading.Thread(target=self._reader_loop, daemon=True)
        self.thread.start()

    def _reader_loop(self):
        while self.running:
            ok, frame = self.cap.read()
            if ok:
                with self.lock:
                    self.latest_frame = frame
            else:
                time.sleep(0.02)

    def get_frame(self):
        with self.lock:
            return None if self.latest_frame is None else self.latest_frame.copy()

    def close(self):
        self.running = False
        self.thread.join(timeout=1.0)
        self.cap.release()


SESSIONS: dict[str, CameraSession] = {}


class OpenSessionReq(BaseModel):
    device: str
    width: int = 640
    height: int = 480


def _get_session(session_id: str) -> CameraSession:
    session = SESSIONS.get(session_id)
    if session is None:
        raise HTTPException(404, "Sessão não encontrada (câmera não aberta ou já fechada).")
    return session


@app.get("/api/health")
def health():
    return {"status": "ok", "version": "1.0"}


@app.get("/api/cameras")
def list_cameras():
    """Lista dispositivos de vídeo plausíveis. No Linux, procura por
    /dev/v4l/by-id/* (caminho estável, igual ao já usado em cameras.yaml); no
    Windows (e como fallback no Linux), testa índices numéricos 0..4."""
    devices = []
    if sys.platform.startswith("linux"):
        for path in sorted(glob.glob("/dev/v4l/by-id/*")):
            devices.append({"device": path})
    if not devices:
        for idx in range(5):
            cap = cv2.VideoCapture(idx)
            ok = cap.isOpened()
            cap.release()
            if ok:
                devices.append({"device": str(idx)})
    return {"devices": devices}


@app.post("/api/sessions")
def open_session(req: OpenSessionReq):
    device = int(req.device) if req.device.isdigit() else req.device
    try:
        session = CameraSession(device, req.width, req.height)
    except RuntimeError as e:
        raise HTTPException(400, str(e))
    session_id = str(uuid.uuid4())
    SESSIONS[session_id] = session
    return {"session_id": session_id, "width": req.width, "height": req.height}


@app.get("/api/sessions/{session_id}/stream")
def stream(session_id: str, overlay: str = "chessboard"):
    session = _get_session(session_id)

    def gen():
        while session_id in SESSIONS:
            frame = session.get_frame()
            if frame is None:
                time.sleep(0.03)
                continue
            display = frame.copy()
            if overlay == "chessboard":
                gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
                found, corners = find_chessboard_corners(gray)
                if found:
                    cv2.drawChessboardCorners(display, PATTERN_SIZE, corners, found)
            ok, jpg = cv2.imencode(".jpg", display)
            if not ok:
                continue
            yield (b"--frame\r\nContent-Type: image/jpeg\r\n\r\n" + jpg.tobytes() + b"\r\n")
            time.sleep(0.04)  # ~25 fps

    return StreamingResponse(gen(), media_type="multipart/x-mixed-replace; boundary=frame")


@app.post("/api/sessions/{session_id}/captures")
def capture(session_id: str):
    session = _get_session(session_id)
    frame = session.get_frame()
    if frame is None:
        raise HTTPException(400, "Sem frame disponível ainda — aguarde a câmera inicializar.")
    gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
    found, corners = find_chessboard_corners(gray)
    if found:
        session.objpoints.append(make_object_points())
        session.imgpoints.append(corners)
        session.image_size = gray.shape[::-1]
    return {"found": found, "count": len(session.objpoints)}


@app.post("/api/sessions/{session_id}/calibrate")
def calibrate(session_id: str):
    session = _get_session(session_id)
    try:
        result = calibrate_camera(session.objpoints, session.imgpoints, session.image_size)
    except ValueError as e:
        raise HTTPException(400, str(e))
    result["images_used"] = len(session.objpoints)
    result["quality"] = reprojection_quality(result["reprojection_error_px"])
    return result


@app.post("/api/sessions/{session_id}/close")
def close_session(session_id: str):
    session = SESSIONS.pop(session_id, None)
    if session:
        session.close()
    return {"closed": True}


if __name__ == "__main__":
    import uvicorn

    uvicorn.run(app, host=HOST, port=PORT)
