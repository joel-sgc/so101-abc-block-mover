#!/usr/bin/env python
"""Stream both Pi cameras as MJPEG so you can check them from a browser.

On the Pi:      uv run python scripts/camera_view.py          (Ctrl+C to stop)
On the laptop:  open http://<pi-ip>:8080   (e.g. http://10.42.0.194:8080)

Stop anything else using the cameras (lerobot-record, the zmq host) first: a camera can only be opened once.
"""

import argparse
import threading
import time
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer

import cv2

CAMS = {
    "front (external)": "/dev/v4l/by-id/usb-046d_Brio_101_2627APAAGJX8-video-index0",
    "wrist": "/dev/v4l/by-id/usb-Generic_USB_HD_Camera_200910120001-video-index0",
}


class Grabber(threading.Thread):
    def __init__(self, path, width, height, quality):
        super().__init__(daemon=True)
        self.cap = cv2.VideoCapture(path, cv2.CAP_V4L2)
        self.cap.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc(*"MJPG"))
        self.cap.set(cv2.CAP_PROP_FRAME_WIDTH, width)
        self.cap.set(cv2.CAP_PROP_FRAME_HEIGHT, height)
        self.quality = quality
        self.jpeg = None
        if not self.cap.isOpened():
            raise RuntimeError(f"cannot open {path} (is another process using it?)")

    def run(self):
        while True:
            ok, frame = self.cap.read()
            if ok:
                ok, buf = cv2.imencode(".jpg", frame, [int(cv2.IMWRITE_JPEG_QUALITY), self.quality])
                if ok:
                    self.jpeg = buf.tobytes()


def make_handler(grabbers, fps):
    names = list(grabbers)

    class H(BaseHTTPRequestHandler):
        def log_message(self, *a):
            pass

        def do_GET(self):
            if self.path == "/":
                body = "<body style='background:#111;color:#eee;font-family:sans-serif'><h3>SO-101 cameras</h3>" + "".join(
                    f"<div style='display:inline-block;margin:6px'><div>{n}</div><img src='/s/{i}' width=640></div>"
                    for i, n in enumerate(names)
                )
                self.send_response(200)
                self.send_header("Content-Type", "text/html")
                self.end_headers()
                self.wfile.write(body.encode())
            elif self.path.startswith("/s/"):
                g = grabbers[names[int(self.path[3:])]]
                self.send_response(200)
                self.send_header("Content-Type", "multipart/x-mixed-replace; boundary=f")
                self.end_headers()
                try:
                    while True:
                        if g.jpeg:
                            self.wfile.write(b"--f\r\nContent-Type: image/jpeg\r\nContent-Length: %d\r\n\r\n" % len(g.jpeg))
                            self.wfile.write(g.jpeg + b"\r\n")
                        time.sleep(1 / fps)
                except (BrokenPipeError, ConnectionResetError):
                    pass
            else:
                self.send_error(404)

    return H


if __name__ == "__main__":
    ap = argparse.ArgumentParser()
    ap.add_argument("--port", type=int, default=8080)
    ap.add_argument("--width", type=int, default=640)
    ap.add_argument("--height", type=int, default=480)
    ap.add_argument("--fps", type=int, default=15, help="stream rate to the browser")
    ap.add_argument("--quality", type=int, default=70)
    a = ap.parse_args()
    grabbers = {n: Grabber(p, a.width, a.height, a.quality) for n, p in CAMS.items()}
    for g in grabbers.values():
        g.start()
    print(f"Streaming on http://0.0.0.0:{a.port}  (Ctrl+C to stop)")
    ThreadingHTTPServer(("0.0.0.0", a.port), make_handler(grabbers, a.fps)).serve_forever()
