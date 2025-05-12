import RPi.GPIO as GPIO
import time
from gpiozero import DistanceSensor
from enum import Enum
import json

# Disable GPIO warnings and set BCM pin numbering mode
GPIO.setwarnings(False)
GPIO.setmode(GPIO.BCM)

# Pin definitions
TASTER = 5              # Pushbutton input pin
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

# Distance threshold (cm) to stop for obstacle avoidance
STOP_DISTANCE = 10

# Dutycycle Hardcoded
DUTYCYCLE = 50 

# The program exection time interval
INTERVAL = 20

# The buffer to Stop distance 
STOP_BUFFER = 0.15

# The slow-down buffer
SLOWDOWN_BUFFER = 0.3

# Global objects for PWM and distance sensor
distance_sensor = None
PWM_1 = None
PWM_2 = None

# Robot state machine states
class RobotState(Enum):
    STOP = 1            # Robot stopped
    MOVE_FORWARD = 2    # Robot moving forward
    IDLE = 3            # Robot idle, motors off
    SLOWDOWN = 4        # Robot slowing down near obstacle

# Initial robot state
et1_state = RobotState.IDLE

# Configure GPIO pins
GPIO.setup(SENSOR_LEFT, GPIO.IN, pull_up_down=GPIO.PUD_UP)   # Left encoder with pull-up
GPIO.setup(SENSOR_RIGHT, GPIO.IN, pull_up_down=GPIO.PUD_UP)  # Right encoder with pull-up
GPIO.setup(TASTER, GPIO.IN, pull_up_down=GPIO.PUD_UP)        # Pushbutton with pull-up

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
    GPIO.output(MOTOR1_IN2, GPIO.LOW)   # Set direction for forward
    GPIO.output(MOTOR1_IN1, GPIO.HIGH)

def M2_forward():
    """Set right motor (Motor 2) to rotate forward."""
    GPIO.output(MOTOR2_IN2, GPIO.LOW)   # Set direction for forward
    GPIO.output(MOTOR2_IN1, GPIO.HIGH)

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

def move_forward_obstacle(PWM_1, PWM_2, duty, interval=20):
    """
    Move robot forward with obstacle avoidance using a state machine.

    Args:
        PWM_1: PWM object for left motor.
        PWM_2: PWM object for right motor.
        duty (float): PWM duty cycle (0-100%) for motor speed.
        interval (float): Maximum runtime (seconds, default: 20).

    Modifies:
        et1_state, left_sensor_tick_count, right_sensor_tick_count: Global state and ticks.
    """
    global et1_state, left_sensor_tick_count, right_sensor_tick_count, distance_sensor
    runtime_start = time.time()
    et1_state = RobotState.MOVE_FORWARD

    while time.time() - runtime_start < interval:
        sensor_distance = distance_sensor.distance * 100  # Convert to cm
        # State transitions based on distance
        if sensor_distance <= (STOP_DISTANCE + max(10, int(STOP_BUFFER * duty))):
            et1_state = RobotState.STOP
        elif STOP_DISTANCE < sensor_distance < (STOP_DISTANCE + max(20, int(SLOWDOWN_BUFFER * duty))):
            et1_state = RobotState.SLOWDOWN
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
        elif et1_state == RobotState.SLOWDOWN:
            slow_duty = max(20,int((sensor_distance / 100) * duty))
            move_forward(PWM_1, PWM_2, slow_duty, 0.1)
            print(f"slowing down {slow_duty}, distance {sensor_distance}, left: {left_sensor_tick_count}, right : {right_sensor_tick_count}")
        elif et1_state == RobotState.STOP:
            PWM_1.ChangeDutyCycle(0)
            PWM_2.ChangeDutyCycle(0)
            GPIO.output(MOTOR1_IN2, GPIO.LOW)   # Set direction for forward
            GPIO.output(MOTOR1_IN1, GPIO.LOW)
            GPIO.output(MOTOR2_IN2, GPIO.LOW)   # Set direction for forward
            GPIO.output(MOTOR2_IN1, GPIO.LOW)
            # time.sleep(0.1)

            print(f"stop dist: {sensor_distance}, left: {left_sensor_tick_count}, right : {right_sensor_tick_count}")
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
    duty = DUTYCYCLE  # PWM duty cycle (50%)
    interval = INTERVAL  # Movement duration (seconds)

    # Move forward with obstacle avoidance
    move_forward_obstacle(PWM_1, PWM_2, duty, interval)

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