import socket
import RPi.GPIO as GPIO
import threading
import json
import os
import time
import subprocess
import signal
from datetime import datetime

# === GPIO Setup ===
CNY70_LEFT = 20
CNY70_RIGHT = 21
GPIO.setwarnings(False)
GPIO.setmode(GPIO.BCM)
GPIO.setup([CNY70_LEFT, CNY70_RIGHT], GPIO.IN, pull_up_down=GPIO.PUD_UP)

# === Shared UDS socket path ===
UDS_PATH = "/tmp/robot.sock"
if os.path.exists(UDS_PATH):
    os.remove(UDS_PATH)

# === Line sensor read ===
def read_line_sensors():
    return {
        "left": int(GPIO.input(CNY70_LEFT)),
        "right": int(GPIO.input(CNY70_RIGHT))
    }

# === Motor controller connection holder ===
motor_conn = None
conn_lock = threading.Lock()

# Shared state
fusion_state = {
    "prev_ticks": {"left": 0, "right": 0},
    "curr_ticks": {"left": 0, "right": 0},
    "prev_time": time.time(),
    "prev_distance": None,
    "curr_distance": None
}

# === Graceful Shutdown ===
def shutdown_all():
    print("[sensor_server] Shutting down due to subprocess exit or SIGINT...")
    try:
        sr04_proc.terminate()
    except: pass
    try:
        motor_proc.terminate()
    except: pass
    GPIO.cleanup()
    os._exit(0)

def sigint_handler(sig, frame):
    print("[sensor_server] Received SIGINT")
    shutdown_all()

signal.signal(signal.SIGINT, sigint_handler)

# === UDS Server Thread (loop to reconnect if needed) ===
def uds_server():
    global motor_conn
    server = socket.socket(socket.AF_UNIX, socket.SOCK_STREAM)
    server.bind(UDS_PATH)
    server.listen(1)
    print("[sensor_server] Waiting for motor controller to connect via UDS...")

    while True:
        conn, _ = server.accept()
        print("[sensor_server] Motor controller connected.")
        with conn_lock:
            motor_conn = conn

        try:
            while True:
                data = conn.recv(1024)
                if not data:
                    break
                try:
                    ticks = json.loads(data.decode())
                    with conn_lock:
                        fusion_state["curr_ticks"] = ticks
                except:
                    pass
        except Exception as e:
            print("[sensor_server] UDS error:", e)
        finally:
            with conn_lock:
                motor_conn = None
            print("[sensor_server] Motor controller disconnected.")

# Start UDS server in background
threading.Thread(target=uds_server, daemon=True).start()

def connect_to_node_tcp():
    try:
        sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        sock.connect(('127.0.0.1', 6000))
        print("[sensor_server] Connected to Node TCP server")
        return sock
    except Exception as e:
        print("[sensor_server] Could not connect to Node:", e)
        return None

node_tcp_sock = connect_to_node_tcp()

# === Fusion computation thread ===
def fusion_loop():
    global node_tcp_sock
    while True:
        time.sleep(0.05)
        with conn_lock:
            curr_time = time.time()
            dt = curr_time - fusion_state["prev_time"]
            prev_ticks = fusion_state["prev_ticks"]
            curr_ticks = fusion_state["curr_ticks"]

            delta_left = curr_ticks["left"] - prev_ticks["left"]
            delta_right = curr_ticks["right"] - prev_ticks["right"]
            avg_ticks = (delta_left + delta_right) / 2.0

            ticks_per_rev = 20
            wheel_circ = 20.73  # cm
            speed_cm_s = (avg_ticks / ticks_per_rev) * wheel_circ / dt if dt > 0 else 0
            turning = delta_right - delta_left
            delta_dist = 0
            if fusion_state["curr_distance"] is not None and fusion_state["prev_distance"] is not None:
                delta_dist = fusion_state["curr_distance"] - fusion_state["prev_distance"]

            # print(f"[Fusion] Speed: {speed_cm_s:.2f} cm/s | Turn: {turning} ticks | Dist: {delta_dist:.2f} cm")

            if node_tcp_sock:
                try:
                    fusion_payload = {
                        "timestamp": datetime.now().strftime('%H:%M:%S.%f')[:-3],
                        "speed_cm_s": round(speed_cm_s, 2),
                        "turning_ticks": turning,
                        "delta_distance": round(delta_dist, 2)
                    }
                    node_tcp_sock.sendall((json.dumps(fusion_payload) + "\n").encode())
                    print(f"payload : {fusion_payload}")
                except Exception as e:
                    print("[sensor_server] Failed to send to Node TCP:", e)
                    node_tcp_sock = None

            fusion_state["prev_ticks"] = curr_ticks.copy()
            fusion_state["prev_time"] = curr_time
            fusion_state["prev_distance"] = fusion_state["curr_distance"]

threading.Thread(target=fusion_loop, daemon=True).start()

# === TCP Server to Receive Distance from C Client ===
HOST = '127.0.0.1'
PORT = 5001
tcp_server = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
tcp_server.bind((HOST, PORT))
tcp_server.listen()
print(f"[sensor_server] Listening for distance on {HOST}:{PORT}...")

# Start sr04_client and motor_controller after servers are ready
sr04_proc = subprocess.Popen(["./sr04_client"], stdout=subprocess.PIPE)
motor_proc = subprocess.Popen(["python3", "motor_task4.py"])

def monitor_subprocesses():
    while True:
        time.sleep(1)
        if sr04_proc.poll() is not None:
            print("[sensor_server] sr04_client has exited.")
            break
        if motor_proc.poll() is not None:
            print("[sensor_server] motor_controller.py has exited.")
            break
    shutdown_all()

threading.Thread(target=monitor_subprocesses, daemon=True).start()

# === Main Loop: Accept C Client and Send Line + Distance to Motor ===
while True:
    conn, _ = tcp_server.accept()
    with conn:
        while True:
            try:
                data = conn.recv(1024)
                if not data:
                    break

                msg = data.decode().strip()

                if "Distance:" in msg:
                    try:
                        distance = float(msg.split(":")[1].strip().split()[0])
                        print(f"[sensor_server] Parsed distance = {distance:.2f} cm")
                        with conn_lock:
                            fusion_state["curr_distance"] = distance
                            if motor_conn:
                                try:
                                    payload = json.dumps({"distance_cm": distance}) + "\n"
                                    motor_conn.sendall(payload.encode())
                                except Exception as e:
                                    print("[sensor_server] Distance send error:", e)
                                    motor_conn = None
                    except Exception as e:
                        print("[sensor_server] Distance parse error:", e)

                line = read_line_sensors()
                conn.sendall(f"Line: {line['left']},{line['right']}\n".encode())

            except Exception as e:
                print("[sensor_server] TCP error:", e)
                break
