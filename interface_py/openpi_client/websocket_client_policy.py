from __future__ import annotations

import msgpack
import msgpack_numpy
import websocket

msgpack_numpy.patch()


class WebsocketClientPolicy:
    def __init__(self, host: str, port: int):
        self.url = f"ws://{host}:{port}"
        self.ws = websocket.create_connection(self.url)
        self._server_metadata = {}
        self._try_read_metadata()

    def _try_read_metadata(self):
        timeout = self.ws.gettimeout()
        try:
            self.ws.settimeout(0.2)
            payload = self.ws.recv()
        except Exception:
            return
        finally:
            self.ws.settimeout(timeout)

        try:
            msg = msgpack.unpackb(payload, raw=False)
        except Exception:
            return
        if isinstance(msg, dict):
            self._server_metadata = msg.get("metadata", msg)

    def get_server_metadata(self):
        return self._server_metadata

    def infer(self, obs):
        payload = msgpack.packb(obs, use_bin_type=True)
        self.ws.send_binary(payload)
        resp = self.ws.recv()
        return msgpack.unpackb(resp, raw=False)

    def close(self):
        self.ws.close()

