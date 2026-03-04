#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import time
import json
import fleetmqsdk

TOPIC = "control"

def main():
    fm = fleetmqsdk.FleetMQ()
    cfg, addr = fm.getConfig(True)
    print("[INFO] got config. starting receive loop...", flush=True)

    while True:
        data = fm.receiveBytes(TOPIC)
        if data:
            try:
                msg = json.loads(data.decode("utf-8"))
                print(f"[RX] t={msg.get('t')} steer={msg.get('steering')} thr={msg.get('throttle')} brk={msg.get('brake')} drt={msg.get('direction')}", flush=True)
            except Exception as e:
                print(f"[RX] parse failed: {e} bytes={data[:80]!r}", flush=True)

        time.sleep(0.01)

if __name__ == "__main__":
    main()
