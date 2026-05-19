# python
import cv2
import socketio
import base64
import time
import sys

SERVER_URLS = ["http://localhost:3000", "http://127.0.0.1:3000"]  # try both loopback variants
RETRY_INITIAL = 1.0
RETRY_MAX = 10.0

sio = socketio.Client(logger=False, engineio_logger=False)


def connect_with_retry(urls=SERVER_URLS, initial_delay=RETRY_INITIAL, max_delay=RETRY_MAX):
    delay = initial_delay
    while True:
        for url in urls:
            try:
                print(f"Attempting connect to {url} ...")
                sio.connect(url, wait=True, transports=["polling", "websocket"])
                if sio.connected:
                    print(f"Connected to {url}")
                    return
            except Exception as e:
                print(f"Connect failed ({url}): {e}")
        print(f"Retrying in {delay:.1f}s...")
        time.sleep(delay)
        delay = min(delay * 2, max_delay)


def send_loop(source=0, fps=20):
    cap = cv2.VideoCapture(source)
    delay = 1.0 / fps
    try:
        while True:
            if not sio.connected:
                print("Socket not connected — attempting reconnect...")
                connect_with_retry()

            ret, frame = cap.read()
            if not ret:
                print("No frame read (end of file or camera). Exiting send loop.")
                break

            _, buf = cv2.imencode('.jpg', frame)
            b64 = base64.b64encode(buf).decode('ascii')

            try:
                sio.emit('newFrame', b64)
            except Exception as e:
                # connection may have dropped between check and emit
                print(f"Emit failed: {e}")
                try:
                    sio.disconnect()
                except Exception:
                    pass
                time.sleep(0.5)
                continue

            time.sleep(delay)
    finally:
        cap.release()
        try:
            sio.disconnect()
        except Exception:
            pass


if __name__ == '__main__':
    # quick check: ensure Node server is running first
    print("Ensure your Node server is running (e.g. run `node server.js` in `WebServer`).")
    # connect (will retry until successful)
    connect_with_retry()
    # start sending frames; change source to a file path or camera index
    source = "WIN_20260513_18_40_03_Pro.mp4" if len(sys.argv) == 1 else sys.argv[1]
    send_loop(source=source, fps=20)
