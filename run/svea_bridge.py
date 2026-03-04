#!/usr/bin/env python

import fleetmqsdk
import socket
import time
import signal
import sys
import struct
import datetime
from threading import Thread

EVERY_IP = "0.0.0.0"
UDP_IP_LOCAL = "192.168.1.121"


def main():
    fleetmq = fleetmqsdk.FleetMQ()
    config, addresses = fleetmq.getConfig(True)
    threads = []
    # sendToControlTowerThread = Thread(target=sendToControlTower, args=(fleetmq, "rcve_state"))
    # sendToControlTowerThread.start()
    # threads.append(sendToControlTowerThread)
    receiveFromControlTowerThread =Thread(target=receiveFromControlTower, args=(fleetmq, "control"))
    receiveFromControlTowerThread.start()
    threads.append(receiveFromControlTowerThread)
    metric_thread = Thread(target=pullMetrics, args=(fleetmq,))
    metric_thread.start()
    threads.append(metric_thread)

def handleExit(configReqInterface, configUpdateInterface):
    configReqInterface.close()
    configUpdateInterface.close()

def receiveFromControlTower(fleetmq, topic):
    sock_tx = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    while True:
        data = fleetmq.receiveBytes(topic)
        if data != None:        
            # fdb = struct.unpack('<IfffII',data)
            # print(fdb) 	
            sock_tx.sendto(data, (UDP_IP_LOCAL, 10003))
        time.sleep(0.01)
        
def sendToControlTower(fleetmq, topic):
    sock_rx_rcve = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    sock_rx_rcve.bind((EVERY_IP, 10002))
    while True:
        data, _ = sock_rx_rcve.recvfrom(8192)
        fleetmq.publishBytes(topic, data)
        time.sleep(0.01)

def pullMetrics(fleetmq):
    sock_tx_latency = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)

    while True:
        try:
            metrics, events = fleetmq.pullMetricsAndEvents()
        except Exception as e:
            print("[pullMetrics] exception:", repr(e))
            time.sleep(0.1)
            continue

        if not metrics:
            continue

        for m in metrics:
            print(
                "Metric:",
                getattr(m, "type", None),
                "value",
                getattr(m, "value", None),
                "peer",
                getattr(m, "peer", None),
                "ts",
                getattr(m, "timestamp", None),
            )
        try:
            latency_ms = int(float(metrics[0].value) / 2)
        except Exception as e:
            print("[pullMetrics] failed to parse latency:", repr(e))
            continue

        latency_bytes = struct.pack('<I', latency_ms)
        sock_tx_latency.sendto(latency_bytes, (UDP_IP_LOCAL, 10088))
        now = datetime.datetime.now().isoformat(timespec="milliseconds")
        print(f"{now}: {latency_ms} ms ")



def signal_handler(sig, frame):
    print("Shutting down...")
    sys.exit(0)

if __name__ == '__main__':
    signal.signal(signal.SIGINT, signal_handler)
    main()
