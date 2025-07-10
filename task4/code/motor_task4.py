#!/usr/bin/env python3
import time
import math
import random
import json
import signal
import threading
import socket
from enum import Enum

import RPi.GPIO as GPIO

# ====== GPIO PIN ASSIGNMENTS ======
SENSOR_LEFT   = 16 # BCM encoder A
SENSOR_RIGHT  = 23 # BCM encoder A

# Left motor (Motor 1) pins
M1_IN1        = 17
M1_IN2        = 22
M1_PWM_PIN    = 18
# Right motor (Motor 2) pins
M2_IN1        = 24
M2_IN2        = 4
M2_PWM_PIN    = 19

PWM_FREQ      = 100

# ====== ROBOT CONSTANTS ======
WHEEL_DIAMETER    = 6.6     # cm
TICKS_PER_ROT     = 20
TRACK_WIDTH       = 10.0    # cm wheel-to-wheel
STOP_DISTANCE     = 30.0      # cm desired clear distance
STOP_BUFFER       = 0.1       # % margin
SLOWDOWN_BUFFER   = 0.3

DUTY_CYCLE        = 35.0    # % base speed
INTERVAL          = 20.0    # seconds max run time

# ====== GLOBAL STATE ======
left_sensor_tick_count  = 0
right_sensor_tick_count = 0

init_move_left_tick = 0
init_move_right_tick = 0

sr04_dist_cm            = 0.0   # updated by socket thread
PWM_1                   = None
PWM_2                   = None
sock                    = None

# ====== ROBOT STATES ======
class RobotState(Enum):
    MOVE_BACKWARD = 1
    MOVE_FORWARD  = 2
    TURN_ANGLE    = 3
    IDLE          = 4
    STOP          = 5
    TURN_LEFT     = 6       # Robot turning left
    TURN_RIGHT    = 7      # Robot turning right

et1_state = RobotState.IDLE

# ====== ENCODER CALLBACKS ======
def sensor_left(channel):
    global left_sensor_tick_count
    left_sensor_tick_count += 1

def sensor_right(channel):
    global right_sensor_tick_count
    right_sensor_tick_count += 1

# ====== MOTOR HELPERS ======
def M1_forward(): 
    GPIO.output(M1_IN2, GPIO.LOW)  
    GPIO.output(M1_IN1, GPIO.HIGH)

def M1_backward():
    GPIO.output(M1_IN1, GPIO.LOW)  
    GPIO.output(M1_IN2, GPIO.HIGH)

def M2_forward(): 
    GPIO.output(M2_IN2, GPIO.LOW)  
    GPIO.output(M2_IN1, GPIO.HIGH)
    
def M2_backward():
    GPIO.output(M2_IN1, GPIO.LOW)  
    GPIO.output(M2_IN2, GPIO.HIGH)

def M1_setup():
    GPIO.setup([M1_IN1, M1_IN2], GPIO.OUT)
    GPIO.setup(M1_PWM_PIN, GPIO.OUT)
    return GPIO.PWM(M1_PWM_PIN, PWM_FREQ)

def M2_setup():
    GPIO.setup([M2_IN1, M2_IN2], GPIO.OUT)
    GPIO.setup(M2_PWM_PIN, GPIO.OUT)
    return GPIO.PWM(M2_PWM_PIN, PWM_FREQ)

# ====== PID CONTROLLER ======
def pid_control(target_diff, actual_diff, integral, last_error, dt,
                kp=5.0, ki=0.1, kd=0.01):
    error      = target_diff - actual_diff
    integral  += error * dt
    derivative = (error - last_error)/dt if dt>0 else 0
    out        = kp*error + ki*integral + kd*derivative
    return out, integral, error

def PID_motor_drive(PWM1, PWM2, base_dutycycle, interval):
    global left_sensor_tick_count, right_sensor_tick_count, init_move_left_tick, init_move_right_tick
    t0        = time.time()
    last_time = t0
    integral, last_error = 0.0, 0.0

    while time.time() - t0 < interval:
        now = time.time() 
        dt = now - last_time
        if dt >= 0.02:
            diff_ticks = (right_sensor_tick_count-init_move_right_tick) - (left_sensor_tick_count -init_move_left_tick)
            # diff_ticks = right_sensor_tick_count - left_sensor_tick_count
            adjust, integral, last_error = pid_control(0, diff_ticks,
                                                       integral, last_error, dt)
            if diff_ticks < 0:
                PWM1.ChangeDutyCycle(max(0, min(100, base_dutycycle - adjust)))
            else:
                PWM2.ChangeDutyCycle(max(0, min(100, base_dutycycle + adjust)))
            last_time = now
        time.sleep(0.001)

    PWM1.ChangeDutyCycle(0)
    PWM2.ChangeDutyCycle(0)

# ====== TURN HELPERS ======
def angle_to_ticks(angle, wheel_diameter=WHEEL_DIAMETER,
                   ticks_per_rotation=TICKS_PER_ROT, track_width=TRACK_WIDTH):
    arc_len  = (angle/360.0) * 2*math.pi*track_width
    circ     = math.pi*wheel_diameter
    return int(round((arc_len/circ)*ticks_per_rotation))

def distance_to_ticks(distance_cm, wheel_diameter=WHEEL_DIAMETER, ticks_per_rotation=TICKS_PER_ROT):
    wheel_circumference = math.pi * wheel_diameter
    rotations = distance_cm / wheel_circumference
    ticks = int(round(rotations * ticks_per_rotation))
    return ticks

def turn(PWM1, PWM2, duty, angle, direction):
    global left_sensor_tick_count, right_sensor_tick_count
    ticks_needed = angle_to_ticks(angle)
    PWM1.ChangeDutyCycle(duty)
    PWM2.ChangeDutyCycle(0)
    if direction: 
        M1_forward()
    else:         
        M1_backward()

    start = left_sensor_tick_count
    t0    = time.time()
    # print(f"tuning left_tick: {left_sensor_tick_count} and ticks nedded  {ticks_needed}")
    while (((left_sensor_tick_count - start) < ticks_needed) and (time.time()-t0 < 5.0)):
        # print(f"tuning left_tick: {left_sensor_tick_count} and ticks nedded  {ticks_needed}, start {start}")
        time.sleep(0.001)
    PWM1.ChangeDutyCycle(0)
    PWM2.ChangeDutyCycle(0)
    left_sensor_tick_count = start
    right_sensor_tick_count =start


# ====== OBSTACLE AVOIDANCE ======
def move_backward(PWM1, PWM2, duty, interval):
    PWM1.ChangeDutyCycle(duty) 
    PWM2.ChangeDutyCycle(duty)
    M1_backward() 
    M2_backward()
    PID_motor_drive(PWM1, PWM2, duty, interval)

def move_forward(PWM1, PWM2, base_dutycycle, interval):
    PWM1.ChangeDutyCycle(base_dutycycle)
    PWM2.ChangeDutyCycle(base_dutycycle)
    M1_forward()
    M2_forward()
    PID_motor_drive(PWM1, PWM2, base_dutycycle, interval)

def move_backward_obstacle(PWM1, PWM2, duty, interval=INTERVAL):
    global et1_state, sr04_dist_cm
    t0 = time.time()
    et1_state = RobotState.MOVE_BACKWARD

    while time.time() - t0 < interval:
        dist = sr04_dist_cm 
        if dist == None:
             continue
        elif   dist <= (STOP_DISTANCE - min(20, int(STOP_BUFFER * duty))): 
            et1_state = RobotState.MOVE_BACKWARD
        elif (STOP_DISTANCE - min(10, int(STOP_BUFFER * duty))) < dist < (STOP_DISTANCE + min(10, int(STOP_BUFFER * duty))):
            et1_state = RobotState.TURN_ANGLE
        else:                                   
            et1_state = RobotState.STOP

        if et1_state == RobotState.MOVE_BACKWARD:
            move_backward(PWM1, PWM2, duty, 0.1)
        elif et1_state == RobotState.TURN_ANGLE:
            PWM1.ChangeDutyCycle(0)
            PWM2.ChangeDutyCycle(0)
            time.sleep(0.2)
            turn(PWM1, PWM2, duty,
                 random.randrange(0,360), random.choice([True, False]))
            et1_state = RobotState.IDLE
            print("end turn")
            time.sleep(0.5)
        elif et1_state == RobotState.IDLE:
            PWM1.ChangeDutyCycle(0) 
            PWM2.ChangeDutyCycle(0)
            time.sleep(0.1)
        elif et1_state == RobotState.STOP:
            PWM1.ChangeDutyCycle(0) 
            PWM2.ChangeDutyCycle(0)
            break
        time.sleep(0.01)

    PWM1.ChangeDutyCycle(0)
    PWM2.ChangeDutyCycle(0)
    et1_state = RobotState.STOP


def move_forward_cm(PWM1, PWM2, travel_dist_cm, duty=DUTY_CYCLE, interval=INTERVAL):
    global left_sensor_tick_count, right_sensor_tick_count, sr04_dist_cm, et1_state, init_move_left_tick, init_move_right_tick
    dist_ticks_needed = distance_to_ticks(travel_dist_cm)
    avg_ticks = (left_sensor_tick_count+right_sensor_tick_count)/2
    start = avg_ticks
    et1_state = RobotState.MOVE_FORWARD

    t0 = time.time()

    print(f"start: dist_ticks {dist_ticks_needed}, avg_ticks {avg_ticks}, left_tick: {left_sensor_tick_count}, right_tick: {right_sensor_tick_count}, sr04 {sr04_dist_cm}")

    init_move_right_tick = right_sensor_tick_count
    init_move_left_tick = left_sensor_tick_count

    while ((time.time() - t0 < interval) and ((avg_ticks - start) <= dist_ticks_needed)) :
        if sr04_dist_cm != 0.0:
            if sr04_dist_cm <= 20:
                et1_state = RobotState.STOP
            else:
                if et1_state != RobotState.MOVE_FORWARD:
                    et1_state = RobotState.MOVE_FORWARD

            # Handle state actions
            if et1_state == RobotState.MOVE_FORWARD:
                move_forward(PWM_1, PWM_2, duty, 0.1)
            elif et1_state == RobotState.IDLE:
                PWM_1.ChangeDutyCycle(0)
                PWM_2.ChangeDutyCycle(0)
                time.sleep(0.1)
            elif et1_state == RobotState.STOP:
                PWM_1.ChangeDutyCycle(0)
                PWM_2.ChangeDutyCycle(0)
                break

        time.sleep(0.01)
        avg_ticks = (left_sensor_tick_count+right_sensor_tick_count)/2

    PWM1.ChangeDutyCycle(0)
    PWM2.ChangeDutyCycle(0)
    et1_state = RobotState.STOP
    print(f"END: dist_ticks {dist_ticks_needed}, avg_ticks {avg_ticks}, left_tick: {left_sensor_tick_count}, right_tick: {right_sensor_tick_count}, sr04 {sr04_dist_cm}")

def move_backward_cm(PWM1, PWM2, travel_dist_cm, duty=DUTY_CYCLE, interval=INTERVAL):
    global left_sensor_tick_count, right_sensor_tick_count, sr04_dist_cm, et1_state
    dist_ticks_needed = distance_to_ticks(travel_dist_cm)
    avg_ticks = (left_sensor_tick_count+right_sensor_tick_count)/2
    start = avg_ticks
    et1_state = RobotState.MOVE_BACKWARD

    t0 = time.time()

    print(f"START: dist_ticks {dist_ticks_needed}, avg_ticks {avg_ticks}, left_tick: {left_sensor_tick_count}, right_tick: {right_sensor_tick_count}, sr04 {sr04_dist_cm}")

    while ((time.time() - t0 < interval) and ((avg_ticks - start) <= dist_ticks_needed)) :
        if sr04_dist_cm != 0.0:
            if sr04_dist_cm <= 20:
                et1_state = RobotState.STOP
            else:
                if et1_state != RobotState.MOVE_BACKWARD:
                    et1_state = RobotState.MOVE_BACKWARD

            # Handle state actions
            if et1_state == RobotState.MOVE_BACKWARD:
                move_backward(PWM_1, PWM_2, duty, 0.1)
            elif et1_state == RobotState.IDLE:
                PWM_1.ChangeDutyCycle(0)
                PWM_2.ChangeDutyCycle(0)
                time.sleep(0.1)
            elif et1_state == RobotState.STOP:
                PWM_1.ChangeDutyCycle(0)
                PWM_2.ChangeDutyCycle(0)
                break

        time.sleep(0.01)
        avg_ticks = (left_sensor_tick_count+right_sensor_tick_count)/2

    PWM1.ChangeDutyCycle(0)
    PWM2.ChangeDutyCycle(0)
    et1_state = RobotState.STOP
    print(f"END: dist_ticks {dist_ticks_needed}, avg_ticks {avg_ticks}, left_tick: {left_sensor_tick_count}, right_tick: {right_sensor_tick_count}, sr04 {sr04_dist_cm}")


# ====== SOCKET READER THREAD ======
def listen_for_sensor_updates(sock):
    global sr04_dist_cm
    while True:
        try:
            msg = sock.recv(1024)
            if not msg:
                break
            try:
                data = json.loads(msg.decode())
                if data.get("distance_cm") is not None:
                    sr04_dist_cm = data["distance_cm"]
                    print(f"[motor_controller] Received distance: {sr04_dist_cm:.2f} cm")
                    if sr04_dist_cm < STOP_DISTANCE:
                        print("[motor_controller] Obstacle close — stopping")
                        # et1_state = RobotState.MOVE_BACKWARD
            except:
                continue
        except:
            break

def send_ticks(sock):
    global left_sensor_tick_count, right_sensor_tick_count
    while True:
        try:
            payload = json.dumps({
                "left": left_sensor_tick_count,
                "right": right_sensor_tick_count
            })
            sock.sendall(payload.encode())
            time.sleep(0.05)
        except Exception as e:
            print("[motor_controller] Failed to send encoder ticks:", e)
            break  # exit loop if socket is broken


# ====== CLEANUP ======
def cleanup(signum=None, frame=None):
    global PWM_1, PWM_2, sock
    print("\nCleaning up…")
    try:
        GPIO.remove_event_detect(SENSOR_LEFT)
        GPIO.remove_event_detect(SENSOR_RIGHT)
    except: pass
    if PWM_1: PWM_1.stop()
    if PWM_2: PWM_2.stop()
    if sock: sock.close()
    GPIO.cleanup()
    print("Done.")
    exit(0)

# ====== MAIN ======
if __name__ == "__main__":
    signal.signal(signal.SIGINT, cleanup)
    GPIO.setmode(GPIO.BCM)
    GPIO.setwarnings(False)
    

    # Encoders
    GPIO.setup(SENSOR_LEFT,  GPIO.IN, pull_up_down=GPIO.PUD_UP)
    GPIO.setup(SENSOR_RIGHT, GPIO.IN, pull_up_down=GPIO.PUD_UP)
    GPIO.add_event_detect(SENSOR_LEFT,  GPIO.RISING, callback=sensor_left)
    GPIO.add_event_detect(SENSOR_RIGHT, GPIO.RISING, callback=sensor_right)

    # Motors
    PWM_1 = M1_setup() 
    PWM_1.start(0)
    PWM_2 = M2_setup()
    PWM_2.start(0)

    # Start SR04 socket reader
    UDS_PATH = "/tmp/robot.sock"
    sock = socket.socket(socket.AF_UNIX, socket.SOCK_STREAM)
    sock.connect(UDS_PATH)
    threading.Thread(target=listen_for_sensor_updates, args=(sock,), daemon=True).start()
    threading.Thread(target=send_ticks, args=(sock,), daemon=True).start()


    # print("Starting obstacle‐avoidance run…")
    # move_backward_obstacle(PWM_1, PWM_2, DUTY_CYCLE, INTERVAL)
    print("start moving 40 cm")
    move_forward_cm(PWM_1, PWM_2, 40, DUTY_CYCLE, INTERVAL)
    print("start turning")
    time.sleep(0.25)
    turn(PWM_1, PWM_2, DUTY_CYCLE, 30, True)
    time.sleep(0.25)
    print("start moving 20 cm")
    move_forward_cm(PWM_1, PWM_2, 20, DUTY_CYCLE, INTERVAL)
    time.sleep(0.25)

    print("start back 20 cm")
    move_backward_cm(PWM_1, PWM_2, 20, DUTY_CYCLE, INTERVAL)
    print("start turning")
    time.sleep(0.25)
    turn(PWM_1, PWM_2, DUTY_CYCLE, 40, False)
    time.sleep(0.5)
    print("start back 40 cm")
    move_backward_cm(PWM_1, PWM_2, 40, DUTY_CYCLE, INTERVAL)
    time.sleep(0.5)

    print("start moving 40 cm")
    move_forward_cm(PWM_1, PWM_2, 40, DUTY_CYCLE, INTERVAL)
    print("start turning")
    time.sleep(0.25)
    turn(PWM_1, PWM_2, DUTY_CYCLE, 40, False)
    time.sleep(0.25)
    print("start moving 20 cm")
    move_forward_cm(PWM_1, PWM_2, 20, DUTY_CYCLE, INTERVAL)
    time.sleep(0.25)

    print("start back 20 cm")
    move_backward_cm(PWM_1, PWM_2, 20, DUTY_CYCLE, INTERVAL)
    print("start turning")
    time.sleep(0.25)
    turn(PWM_1, PWM_2, DUTY_CYCLE, 30, True)
    time.sleep(0.5)
    print("start back 40 cm")
    move_backward_cm(PWM_1, PWM_2, 40, DUTY_CYCLE, INTERVAL)

    # Final report
    result = {
        "final_distance_cm": sr04_dist_cm,
        "left_ticks":       left_sensor_tick_count,
        "right_ticks":      right_sensor_tick_count,
        "final_state":      et1_state.name
    }
    print(json.dumps(result))
    cleanup()