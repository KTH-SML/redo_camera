#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import time
import struct
import fleetmqsdk

TOPIC = "control"
EXPECTED_LEN = 24  # bytes (<Ifffii)

def main():
    fm = fleetmqsdk.FleetMQ()
    cfg, addr = fm.getConfig(True)
    print("[INFO] got config. starting receive loop...", flush=True)

    last_pkg = None

    while True:
        data = fm.receiveBytes(TOPIC)
        if not data:
            time.sleep(0.01)
            continue

        if len(data) != EXPECTED_LEN:
            print(f"[WARN] unexpected length={len(data)} bytes raw={data!r}", flush=True)
            time.sleep(0.01)
            continue

        try:
            pkgNr, refStr, refThr, refBrk, direction, model1 = struct.unpack(
                "<Ifffii", data
            )

            # Optional sanity checks
            if not (-4.0 <= refStr <= 4.0):
                print(f"[WARN] steering out of expected range: {refStr}", flush=True)
            if not (0.0 <= refThr <= 1.2):
                print(f"[WARN] throttle out of range: {refThr}", flush=True)
            if not (0.0 <= refBrk <= 1.2):
                print(f"[WARN] brake out of range: {refBrk}", flush=True)

            # Packet continuity check
            if last_pkg is not None:
                delta = (pkgNr - last_pkg) & 0xFFFFFFFF
                if delta != 1:
                    print(f"[WARN] pkg jump: prev={last_pkg} now={pkgNr}", flush=True)
            last_pkg = pkgNr

            print(
                f"[RX] pkg={pkgNr:10d} "
                f"str={refStr:+.3f}rad "
                f"thr={refThr:.3f} "
                f"brk={refBrk:.3f} "
                f"dir={'FWD' if direction==1 else 'REV'} "
                f"model={model1}",
                flush=True,
            )

        except struct.error as e:
            print(f"[ERR] unpack failed: {e} raw={data!r}", flush=True)

        time.sleep(0.01)

if __name__ == "__main__":
    main()
