import RPi.GPIO as GPIO
import time
from gpiozero import DistanceSensor
from enum import Enum
import json
import random
import math

# Disable GPIO warnings and set BCM pin numbering mode
GPIO.setwarnings(False)
GPIO.setmode(GPIO.BCM)

# Hardcoded constants
DUTYCYCLE = 50          # Base PWM duty cycle (%)
INTERVAL = 20           # Program execution time interval (seconds)
STOP_BUFFER = 0.1       # Buffer factor for stop distance threshold
SLOWDOWN_BUFFER = 0.3   # Buffer factor for slowdown distance threshold

# Pin definitions
# TASTER = 5              # Pushbutton input pin (unused)
# Left motor (Motor 1) pins
MOTOR1_PWM = 18         # PWM pin for speed control
MOTOR1_IN1 = 17         # Direction control pin 1
MOTOR1_IN2 = 22         # Direction control pin 2
# Right motor (Motor 2) pins
MOTOR2_PWM = 19         # PWM pin for speed control
MOTOR2_IN1 = 24         # Direction control pin 1
MOTOR2_IN2 = 4          # Direction control pin 2
# Wheel encoder sensor pins
SENSOR_LEFT = 16        # Left wheel encoder input
SENSOR_RIGHT = 23       # Right wheel encoder input

# Global variables for tracking encoder ticks
left_sensor_tick_count = 0
right_sensor_tick_count = 0

# Encoder ticks per wheel rotation
MOTOR_ENCODER_TICKS = 20

# Distance threshold (cm) for obstacle avoidance
STOP_DISTANCE = 30

# Global objects for PWM and distance sensor
distance_sensor = None
PWM_1 = None
PWM_2 = None

# Robot state machine states
class RobotState(Enum):
    STOP = 1            # Robot stopped
    MOVE_FORWARD = 2    # Robot moving forward
    MOVE_BACKWARD = 3   # Robot moving backward
    TURN_LEFT = 4       # Robot turning left
    TURN_RIGHT = 5      # Robot turning right
    IDLE = 6            # Robot idle, motors off
    SLOWDOWN = 7        # Robot slowing down
    TURN_ANGLE = 8      # Robot performing random angle turn

# Initial robot state
et1_state = RobotState.IDLE

# Configure GPIO pins
GPIO.setup(SENSOR_LEFT, GPIO.IN, pull_up_down=GPIO.PUD_UP)   # Left encoder with pull-up
GPIO.setup(SENSOR_RIGHT, GPIO.IN, pull_up_down=GPIO.PUD_UP)  # Right encoder with pull-up
# GPIO.setup(TASTER, GPIO.IN, pull_up_down=GPIO.PUD_UP)      # Pushbutton with pull-up (unused)

def M1_setup():
    """Set up left motor (Motor 1) pins and initialize PWM."""
    GPIO.setup(MOTOR1_IN1, GPIO.OUT)   # Direction pin 1
    GPIO.setup(MOTOR1_IN2, GPIO.OUT)   # Direction pin 2
    GPIO.setup(MOTOR1_PWM, GPIO.OUT)   # PWM pin for speed
    pwm = GPIO.PWM(MOTOR1_PWM, 90)     # Initialize PWM at 90Hz
    return pwm

def M2_setup():
    """Set up right motor (Motor 2) pins and initialize PWM."""
    GPIO.setup(MOTOR2_IN1, GPIO.OUT)   # Direction pin 1
    GPIO.setup(MOTOR2_IN2, GPIO.OUT)   # Direction pin 2
    GPIO.setup(MOTOR2_PWM, GPIO.OUT)   # PWM pin for speed
    pwm = GPIO.PWM(MOTOR2_PWM, 90)     # Initialize PWM at 90Hz
    return pwm

def M1_forward():
    """Set left motor (Motor 1) to rotate forward."""
    GPIO.output(MOTOR1_IN2, GPIO.LOW)
    GPIO.output(MOTOR1_IN1, GPIO.HIGH)

def M1_backward():
    """Set left motor (Motor 1) to rotate backward."""
    GPIO.output(MOTOR1_IN1, GPIO.LOW)
    GPIO.output(MOTOR1_IN2, GPIO.HIGH)

def M2_forward():
    """Set right motor (Motor 2) to rotate forward."""
    GPIO.output(MOTOR2_IN2, GPIO.LOW)
    GPIO.output(MOTOR2_IN1, GPIO.HIGH)

def M2_backward():
    """Set right motor (Motor 2) to rotate backward."""
    GPIO.output(MOTOR2_IN1, GPIO.LOW)
    GPIO.output(MOTOR2_IN2, GPIO.HIGH)

def pid_control(target_diff, actual_diff, integral, last_error, dt, kp, ki, kd):
    """
    Compute PID correction to balance motor speeds based on encoder tick difference.

    Args:
        target_diff (float): Desired tick difference (0 for straight movement).
        actual_diff (float): Current tick difference (right - left).
        integral (float): Accumulated error for integral term.
        last_error (float): Previous error for derivative term.
        dt (float): Time step (seconds).
        kp (float): Proportional gain.
        ki (float): Integral gain.
        kd (float): Derivative gain.

    Returns:
        tuple: (output, integral, error)
            - output: PID correction for motor speed adjustment.
            - integral: Updated integral term.
            - error: Current error for next iteration.
    """
    error = target_diff - actual_diff
    integral += error * dt
    derivative = (error - last_error) / dt if dt > 0 else 0
    output = kp * error + ki * integral + kd * derivative
    return output, integral, error

def PID_motor_drive(PWM1, PWM2, base_dutycycle, interval):
    """
    Drive motors with PID control to maintain straight movement using encoder ticks.

    Args:
        PWM1: PWM object for left motor.
        PWM2: PWM object for right motor.
        base_dutycycle (float): Base PWM duty cycle (0-100%).
        interval (float): Duration to drive (seconds).

    Modifies:
        left_sensor_tick_count, right_sensor_tick_count: Global encoder tick counts.
    """
    global left_sensor_tick_count, right_sensor_tick_count
    start_time = time.time()
    last_time = start_time
    KP, KI, KD = 5.0, 0.1, 0.01  # PID constants
    integral = 0.0
    last_error = 0.0

    while time.time() - start_time < interval:
        current_time = time.time()
        dt = current_time - last_time
        if dt >= 0.02:  # Update every 20ms
            tick_diff = right_sensor_tick_count - left_sensor_tick_count
            adjust, integral, last_error = pid_control(0, tick_diff, integral, last_error, dt, KP, KI, KD)
            if tick_diff < 0:  # Right wheel lagging
                left_dutycycle = max(0, min(100, base_dutycycle - adjust))
                PWM1.ChangeDutyCycle(left_dutycycle)
            else:  # Left wheel lagging or equal
                right_dutycycle = max(0, min(100, base_dutycycle + adjust))
                PWM2.ChangeDutyCycle(right_dutycycle)
            last_time = current_time
        time.sleep(0.001)  # Prevent CPU overload

    PWM1.ChangeDutyCycle(0)  # Stop left motor
    PWM2.ChangeDutyCycle(0)  # Stop right motor

def move_forward(PWM1, PWM2, base_dutycycle, interval):
    """
    Move robot forward with PID control for straight movement.

    Args:
        PWM1: PWM object for left motor.
        PWM2: PWM object for right motor.
        base_dutycycle (float): Base PWM duty cycle (0-100%).
        interval (float): Duration to move (seconds).
    """
    PWM1.ChangeDutyCycle(base_dutycycle)
    PWM2.ChangeDutyCycle(base_dutycycle)
    M1_forward()
    M2_forward()
    PID_motor_drive(PWM1, PWM2, base_dutycycle, interval)

def move_backward(PWM1, PWM2, base_dutycycle, interval):
    """
    Move robot backward with PID control for straight movement.

    Args:
        PWM1: PWM object for left motor.
        PWM2: PWM object for right motor.
        base_dutycycle (float): Base PWM duty cycle (0-100%).
        interval (float): Duration to move (seconds).
    """
    PWM1.ChangeDutyCycle(base_dutycycle)
    PWM2.ChangeDutyCycle(base_dutycycle)
    M1_backward()
    M2_backward()
    PID_motor_drive(PWM1, PWM2, base_dutycycle, interval)

def angle_to_ticks(angle, wheel_diameter=6.5, ticks_per_rotation=20, track_width=10.0):
    """
    Calculate encoder ticks required for a pivot turn.

    Args:
        angle (float): Turn angle in degrees.
        wheel_diameter (float): Wheel diameter in cm (default: 6.5).
        ticks_per_rotation (int): Encoder ticks per wheel rotation (default: 20).
        track_width (float): Distance between wheels in cm (default: 10.0).

    Returns:
        int: Number of ticks required for the active wheel.
    """
    turn_radius = track_width
    arc_length = (angle / 360) * 2 * math.pi * turn_radius  # Arc length in cm
    wheel_circumference = math.pi * wheel_diameter  # Wheel circumference in cm
    rotations = arc_length / wheel_circumference
    ticks = int(round(rotations * ticks_per_rotation))
    return ticks

def turn(PWM1, PWM2, dutycycle, angle, direction):
    """
    Turn the robot by a specified angle in the given direction using encoder ticks.

    Args:
        PWM1: PWM object for left motor.
        PWM2: PWM object for right motor.
        dutycycle (float): PWM duty cycle (0-100%) for the active motor.
        angle (int): Desired turn angle in degrees.
        direction (bool): True for right turn, False for left turn.

    Modifies:
        left_sensor_tick_count, right_sensor_tick_count: Global encoder tick counts.
        et1_state: Robot state (set to IDLE after turn).
    """
    global left_sensor_tick_count, right_sensor_tick_count, et1_state
    ticks = angle_to_ticks(angle, wheel_diameter=6.6, ticks_per_rotation=20, track_width=10.0)
    timeout = 5.0  # Maximum turn time (seconds)

    PWM1.ChangeDutyCycle(dutycycle)
    PWM2.ChangeDutyCycle(0)

    if direction:  # Right turn: left motor forward, right motor stopped
        print(f"Turning right {angle} degrees ({ticks} ticks)")
        M1_forward()
    else:  # Left turn: left motor backward, right motor stopped
        print(f"Turning left {angle} degrees ({ticks} ticks)")
        M1_backward()

    left_count_init = left_sensor_tick_count
    start_time = time.time()
    while (left_sensor_tick_count - left_count_init) < ticks and time.time() - start_time < timeout:
        time.sleep(0.001)  # Non-blocking loop for encoder updates

    PWM1.ChangeDutyCycle(0)  # Stop motors
    PWM2.ChangeDutyCycle(0)

def move_backward_obstacle(PWM_1, PWM_2, duty, interval=20):
    """
    Move robot backward with obstacle avoidance using a state machine.

    Args:
        PWM_1: PWM object for left motor.
        PWM_2: PWM object for right motor.
        duty (float): PWM duty cycle (0-100%) for motor speed.
        interval (float): Maximum runtime (seconds, default: 20).

    Modifies:
        et1_state, left_sensor_tick_count, right_sensor_tick_count: Global state and ticks.

    Notes:
        Transitions to MOVE_BACKWARD, TURN_ANGLE, or STOP based on distance sensor.
        Performs random angle turns (0-360°) when near obstacles.
        Uses STOP_BUFFER and SLOWDOWN_BUFFER for distance thresholds.
    """
    global et1_state, left_sensor_tick_count, right_sensor_tick_count, distance_sensor
    runtime_start = time.time()
    et1_state = RobotState.MOVE_BACKWARD

    while time.time() - runtime_start < interval:
        sensor_distance = distance_sensor.distance * 100  # Convert to cm
        # State transitions based on distance
        if sensor_distance <= (STOP_DISTANCE - max(20, int(STOP_BUFFER * duty))):
            et1_state = RobotState.MOVE_BACKWARD
        elif (STOP_DISTANCE - max(10, int(STOP_BUFFER * duty))) < sensor_distance < (STOP_DISTANCE + max(10, int(STOP_BUFFER * duty))):
            et1_state = RobotState.TURN_ANGLE
        else:
            et1_state = RobotState.STOP

        # Handle state actions
        if et1_state == RobotState.MOVE_BACKWARD:
            move_backward(PWM_1, PWM_2, duty, 0.1)
        elif et1_state == RobotState.TURN_ANGLE:
            PWM_1.ChangeDutyCycle(0)
            PWM_2.ChangeDutyCycle(0)
            time.sleep(0.2)
            print(f"diatance : {distance_sensor.distance * 100}")
            turn(PWM_1, PWM_2, duty, random.randrange(0, 360), random.choice([True, False]))
            et1_state = RobotState.IDLE
            time.sleep(0.5)
        elif et1_state == RobotState.IDLE:
            PWM_1.ChangeDutyCycle(0)
            PWM_2.ChangeDutyCycle(0)
            time.sleep(0.1)
        elif et1_state == RobotState.STOP:
            PWM_1.ChangeDutyCycle(0)
            PWM_2.ChangeDutyCycle(0)
            break
        time.sleep(0.01)

    PWM_1.ChangeDutyCycle(0)
    PWM_2.ChangeDutyCycle(0)
    et1_state = RobotState.STOP

def sensor_left(channel):
    """Increment left wheel encoder tick count on rising edge."""
    global left_sensor_tick_count
    left_sensor_tick_count += 1

def sensor_right(channel):
    """Increment right wheel encoder tick count on rising edge."""
    global right_sensor_tick_count
    right_sensor_tick_count += 1

def cleanup():
    """Clean up GPIO resources, stop PWM, and close sensor connections."""
    global PWM_1, PWM_2, distance_sensor
    print("Cleaning up GPIO and resources...")
    try:
        if distance_sensor:
            distance_sensor.close()
            distance_sensor = None
        if PWM_1:
            PWM_1.stop()
        if PWM_2:
            PWM_2.stop()
        try:
            GPIO.remove_event_detect(SENSOR_LEFT)
        except:
            pass
        try:
            GPIO.remove_event_detect(SENSOR_RIGHT)
        except:
            pass
        print("Cleanup complete.")
    except Exception as e:
        print(f"Error during cleanup: {e}")

try:
    # Initialize PWM for motors
    PWM_1 = M1_setup()
    PWM_1.start(0)
    PWM_2 = M2_setup()
    PWM_2.start(0)

    # Set up encoder interrupts
    GPIO.add_event_detect(SENSOR_LEFT, GPIO.RISING, callback=sensor_left)
    GPIO.add_event_detect(SENSOR_RIGHT, GPIO.RISING, callback=sensor_right)

    # Initialize distance sensor (HC-SR04)
    distance_sensor = DistanceSensor(echo=27, trigger=25)

    # Set movement parameters
    duty = DUTYCYCLE  # PWM duty cycle
    interval = INTERVAL  # Movement duration (seconds)

    # Move backward with obstacle avoidance
    move_backward_obstacle(PWM_1, PWM_2, duty, interval)

    # Stop motors
    PWM_1.ChangeDutyCycle(0)
    PWM_2.ChangeDutyCycle(0)
    time.sleep(0.5)  # Brief pause

    # Read final distance and output data
    sensor_dist = distance_sensor.distance * 100
    data = {
        "distance": sensor_dist,
        "left_ticks": left_sensor_tick_count,
        "right_ticks": right_sensor_tick_count
    }
    print(json.dumps(data))

except KeyboardInterrupt:
    print("\nProgram interrupted by user.")
finally:
    cleanup()  # Ensure resources are released