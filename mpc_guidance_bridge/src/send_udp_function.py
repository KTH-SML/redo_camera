import socket
import struct
import time
import numpy as np
from typing import List, Tuple

MAGIC = 0x4D504350  # "MPCP"
VERSION = 1

class MPCUdpSender:
    MTU_SAFE = 1200 
    XYVV_SIZE = 16
    COUNT_SIZE = 4          # uint32
    MAX_XYVV_PER_PACKET = max(1, (MTU_SAFE - COUNT_SIZE) // XYVV_SIZE)

    _pack_count = struct.Struct("<I")        # uint32 little-endian
    _pack_xyvv  = struct.Struct("<ffff")     # float32 x4 little-endian

    def __init__(self, host: str = "127.0.0.1", port_points: int = 50052, port_obstacles: int = 50051, max_bytes: int = 65000):
        self.addr_points = (host, port_points)
        self.sock_points = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.max_bytes = max_bytes
        self.seq = 0

        self.addr_obs = (host, port_obstacles)
        self.sock_obs = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)

    def send_points(self, pts_xy: np.ndarray, ok: bool, steer_rate: float = 0.0, accel: float = 0.0):
        """
        pts_xy: shape (N,2), float-like
        """
        pts = np.asarray(pts_xy, dtype=np.float32)
        if pts.ndim != 2 or pts.shape[1] != 2:
            raise ValueError(f"pts_xy must be (N,2), got {pts.shape}")

        N = pts.shape[0]
        flags = 1 if ok else 0
        t_ns = time.time_ns()

        # I: uint32, H:uint16, H:uint16, I:uint32, Q:uint64, H:uint16, H:uint16
        header = struct.pack("!IHHIQHH", MAGIC, VERSION, flags, self.seq, t_ns, N, 0)

        payload = pts.reshape(-1).tobytes(order="C")

        self.sock_points.sendto(header + payload, self.addr_points)
        self.seq = (self.seq + 1) & 0xFFFFFFFF
    
    def send_obstacles_as_xyvv(self, xyvv_list: List[Tuple[float, float, float, float]]) -> bool:
        if xyvv_list is None:
            xyvv_list = []

        total = len(xyvv_list)
        off = 0

        while off < total:
            chunk = xyvv_list[off : off + self.MAX_XYVV_PER_PACKET]
            off += len(chunk)

            buf = bytearray()
            buf += self._pack_count.pack(len(chunk))

            for (x, y, vx, vy) in chunk:
                fx = float(x) if np.isfinite(x) else 0.0
                fy = float(y) if np.isfinite(y) else 0.0
                fvx = float(vx) if np.isfinite(vx) else 0.0
                fvy = float(vy) if np.isfinite(vy) else 0.0
                buf += self._pack_xyvv.pack(fx, fy, fvx, fvy)

            n = self.sock_obs.sendto(buf, self.addr_obs)
            if n != len(buf):
                return False

        return True