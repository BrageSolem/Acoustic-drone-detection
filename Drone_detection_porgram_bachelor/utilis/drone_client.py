import socketio
import threading
import time
import random
from datetime import datetime

class DroneSocketClient:
    """
    Lightweight socket.io client for sending/receiving drone updates.

    Usage:
      from drone_client import DroneSocketClient
      client = DroneSocketClient('http://localhost:3000',
                                 on_init=lambda data: print('init', data),
                                 on_update=lambda d: print('update', d))
      client.connect()
      client.emit_drone({...})
      client.emit_degree({...})
      client.disconnect()
    """

    def __init__(self, server_url='http://localhost:3001', on_init=None, on_update=None):
        self.server_url = server_url
        self.on_init = on_init
        self.on_update = on_update

        self.sio = socketio.Client()
        self._stop_event = threading.Event()
        self._send_thread = None

        # register events
        self.sio.on('connect')(self._on_connect)
        self.sio.on('disconnect')(self._on_disconnect)
        self.sio.on('initData')(self._on_init_data)
        self.sio.on('updateDrones')(self._on_update_drones)

    # --- Public API ---
    def connect(self, **connect_kwargs):
        """Connect to the socket.io server. Pass additional kwargs to `socketio.Client.connect`."""
        self.sio.connect(self.server_url, **connect_kwargs)

    def disconnect(self):
        """Stop any background sender and disconnect."""
        self.stop_periodic_send()
        try:
            self.sio.disconnect()
        except Exception:
            pass

    def emit_drone(self, drone):
        """Emit a single drone object using event 'newDrone'."""
        try:
            self.sio.emit('newDrone', drone)
        except Exception:
            pass

    def emit_degree(self, degree):
        """Emit a degree update using event 'newDegree'."""
        try:
            self.sio.emit('newDegree', degree)
        except Exception:
            pass

    def _on_connect(self):
        # silent by default;
        pass

    def _on_disconnect(self):
        # silent by default;
        pass

    def _on_init_data(self, data):
        if callable(self.on_init):
            try:
                self.on_init(data)
            except Exception:
                pass

    def _on_update_drones(self, data):
        if callable(self.on_update):
            try:
                self.on_update(data)
            except Exception:
                pass

    @staticmethod
    def _default_make_drone():
        return {
            "id": f"py_{random.randint(0,9999)}",
            "type": "PythonSim",
            "time": datetime.utcnow().isoformat(),
            "lat": 59.91 + random.uniform(-0.01, 0.01),
            "lng": 10.75 + random.uniform(-0.01, 0.01)
        }