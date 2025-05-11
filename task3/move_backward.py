import RPi.GPIO as GPIO
import time
from gpiozero import DistanceSensor
from enum import Enum

import json
import random
import math

GPIO.setwarnings(False)
GPIO.setmode(GPIO.BCM)

Taster = 5             # Pin for pushbutton

# Motor 1 (Left) pin definitions
Motor1_PWM = 18        # PWM pin for Motor 1
Motor1_IN1 = 17        # Input pin 1 for Motor 1
Motor1_IN2 = 22        # Input pin 2 for Motor 1

# Motor 2 (Right) pin definitions
Motor2_PWM = 19        # PWM pin for Motor 2
Motor2_IN1 = 24        # Input pin 1 for Motor 2
Motor2_IN2 = 4         # Input pin 2 for Motor 2

# Sensor pin definitions
SENSOR_LEFT = 16       # Pin for left sensor
SENSOR_RIGHT = 23      # Pin for right sensor

# Variables to keep track of sensor tick counts
left_sensor_tick_count = 0
right_sensor_tick_count = 0

MOTOR_ENCODER_TICKS = 20

STOP_DISTANCE = 30

distance_sensor =None
PWM_1 = None
PWM_2 = None

# State machine states
class RobotState(Enum):
    STOP = 1
    MOVE_FORWARD = 2
    MOVE_BACKWARD = 3
    TURN_LEFT = 4
    TURN_RIGHT = 5
    IDLE = 6
    SLOWDOWN = 7
    TURN_ANGLE = 8

et1_state = RobotState.IDLE

# GPIO setup for LED and pushbutton
GPIO.setup(SENSOR_LEFT, GPIO.IN, pull_up_down=GPIO.PUD_UP)
GPIO.setup(SENSOR_RIGHT, GPIO.IN, pull_up_down=GPIO.PUD_UP)
GPIO.setup(Taster, GPIO.IN, pull_up_down=GPIO.PUD_UP)   # Set pushbutton pin as input with pull-up resistor


# Function to setup Motor 1 (Left) control
def M1_setup():
    GPIO.setup(Motor1_IN1, GPIO.OUT)   # Set Motor1_IN1 as output for direction control
    GPIO.setup(Motor1_IN2, GPIO.OUT)   # Set Motor1_IN2 as output for direction control
    GPIO.setup(Motor1_PWM, GPIO.OUT)   # Set Motor1_PWM pin as output for speed control
    PWM_1 = GPIO.PWM(Motor1_PWM, 90)   # Initialize PWM with 90Hz frequency
    return PWM_1                       

# Function to setup Motor 2 (Right) control
def M2_setup():
    GPIO.setup(Motor2_IN1, GPIO.OUT)   # Set Motor2_IN1 as output for direction control
    GPIO.setup(Motor2_IN2, GPIO.OUT)   # Set Motor2_IN2 as output for direction control
    GPIO.setup(Motor2_PWM, GPIO.OUT)   # Set Motor2_PWM pin as output for speed control
    PWM_2 = GPIO.PWM(Motor2_PWM, 90)   # Initialize PWM with 90Hz frequency
    return PWM_2                       

# Function to rotate Motor 1 forward
def M1_forward():
    GPIO.output(Motor1_IN2, GPIO.LOW)  # Set Motor1_IN2 LOW for forward rotation
    GPIO.output(Motor1_IN1, GPIO.HIGH) # Set Motor1_IN1 HIGH for forward rotation

# Function to rotate Motor 1 backward
def M1_backward():
    GPIO.output(Motor1_IN1, GPIO.LOW)  # Set Motor1_IN1 LOW for backward rotation
    GPIO.output(Motor1_IN2, GPIO.HIGH) # Set Motor1_IN2 HIGH for backward rotation

# Function to rotate Motor 2 forward
def M2_forward():
    GPIO.output(Motor2_IN2, GPIO.LOW)  # Set Motor2_IN2 LOW for forward rotation
    GPIO.output(Motor2_IN1, GPIO.HIGH) # Set Motor2_IN1 HIGH for forward rotation

# Function to rotate Motor 2 backward
def M2_backward():
    GPIO.output(Motor2_IN1, GPIO.LOW)  # Set Motor2_IN1 LOW for backward rotation
    GPIO.output(Motor2_IN2, GPIO.HIGH) # Set Motor2_IN2 HIGH for backward rotation


# PID control function to compute correction for motor speed based on tick difference
def pid_control(target_diff, actual_diff, integral, last_error, dt, kp, ki, kd):
    """
    Computes PID control output to minimize the difference between target and actual tick counts.
    
    Args:
        target_diff (float): Desired tick difference (setpoint, typically 0 for straight movement).
        actual_diff (float): Current tick difference (right - left sensor ticks).
        integral (float): Accumulated error for integral term.
        last_error (float): Previous error for derivative term.
        dt (float): Time step (seconds) since last update.
        kp (float): Proportional gain.
        ki (float): Integral gain.
        kd (float): Derivative gain.
    
    Returns:
        tuple: (output, integral, error)
            - output (float): PID correction to adjust motor speed.
            - integral (float): Updated integral term.
            - error (float): Current error for use as last_error in next call.
    """
    # Calculate error as the difference between target and actual tick difference
    error = target_diff - actual_diff
    # Update integral term by accumulating error over time
    integral += error * dt
    # Calculate derivative term as rate of change of error
    derivative = (error - last_error) / dt if dt > 0 else 0
    # Compute PID output: proportional + integral + derivative
    output = kp * error + ki * integral + kd * derivative
    return output, integral, error

# Core function to drive motors with PID control for straight movement
def PID_motor_drive(PWM1, PWM2, base_dutycycle, interval):
    """
    Drives motors using PID control to maintain straight movement by balancing wheel ticks.
    
    Args:
        PWM1: PWM object for left motor.
        PWM2: PWM object for right motor.
        base_dutycycle (float): Base PWM duty cycle (0-100%) for motor speed.
        interval (float): Duration (seconds) to drive the motors.
    
    Uses global variables:
        left_sensor_tick_count, right_sensor_tick_count: Encoder tick counts for left and right wheels.
    """
    # Access global tick counts for wheel encoders
    global left_sensor_tick_count, right_sensor_tick_count
    # Record start time to track duration
    start_time = time.time()
    last_time = start_time
    # PID constants for proportional, integral, and derivative terms
    KP = 5.0
    KI = 0.1
    KD = 0.01
    # Initialize PID variables
    integral = 0.0
    last_error = 0.0

    # Run until the specified interval has elapsed
    while (time.time() - start_time < interval):
        current_time = time.time()
        dt = current_time - last_time

        # Update every 0.02 seconds to avoid excessive computation
        if dt >= 0.02:
            # Calculate tick difference (right - left) to detect deviation
            tick_diff = right_sensor_tick_count - left_sensor_tick_count
            # Compute PID correction
            adjust, integral, last_error = pid_control(0, tick_diff, integral, last_error, dt, KP, KI, KD)
            # Adjust motor speeds based on tick difference
            if tick_diff < 0:  # Right wheel lagging, slow left motor
                left_dutycycle = max(0, min(100, base_dutycycle - adjust))
                PWM1.ChangeDutyCycle(left_dutycycle)
            else:  # Left wheel lagging or equal, adjust right motor
                right_dutycycle = max(0, min(100, base_dutycycle + adjust))
                PWM2.ChangeDutyCycle(right_dutycycle)

            # print(f"Backward - Left ticks: {left_sensor_tick_count}, Right ticks: {right_sensor_tick_count}, pid_adjust: {adjust:.4f}, "
            #       f"Diff: {tick_diff}, Left PWM: {left_dutycycle:.1f}, Right PWM: {right_dutycycle:.1f}")
            # Update last_time for next iteration
            last_time = current_time
        # Small delay to prevent excessive CPU usage
        time.sleep(0.001)
    
    # Stop motors by setting PWM duty cycles to 0
    PWM1.ChangeDutyCycle(0)
    PWM2.ChangeDutyCycle(0)

# Function to move the robot forward for a given interval
def move_forward(PWM1, PWM2, base_dutycycle, interval):
    """
    Moves the robot forward using PID control to maintain straight movement.
    
    Args:
        PWM1: PWM object for left motor.
        PWM2: PWM object for right motor.
        base_dutycycle (float): Base PWM duty cycle (0-100%) for motor speed.
        interval (float): Duration (seconds) to move forward.
    """
    # Set initial PWM duty cycles for both motors
    PWM1.ChangeDutyCycle(base_dutycycle)
    PWM2.ChangeDutyCycle(base_dutycycle)
    
    # Set motor directions to forward (assumes M1/M2_forward are defined externally)
    M2_forward()
    M1_forward()
    # Drive motors with PID control
    PID_motor_drive(PWM1, PWM2, base_dutycycle, interval)

# Function to move the robot backward for a given interval
def move_backward(PWM1, PWM2, base_dutycycle, interval):
    """
    Moves the robot backward using PID control to maintain straight movement.
    
    Args:
        PWM1: PWM object for left motor.
        PWM2: PWM object for right motor.
        base_dutycycle (float): Base PWM duty cycle (0-100%) for motor speed.
        interval (float): Duration (seconds) to move backward.
    """
    # Set initial PWM duty cycles for both motors
    PWM1.ChangeDutyCycle(base_dutycycle)
    PWM2.ChangeDutyCycle(base_dutycycle)
    
    # Set motor directions to backward (assumes M1/M2_backward are defined externally)
    M2_backward()
    M1_backward()

    # Drive motors with PID control
    PID_motor_drive(PWM1, PWM2, base_dutycycle, interval)


# Function to calculate ticks for a turn based on wheel diameter and track width
def angle_to_ticks(angle, wheel_diameter=6.5, ticks_per_rotation=20, track_width = 10.0):
    """
    Calculate encoder ticks required for a pivot turn.

    Args:
        angle (float): Turn angle in degrees (e.g., 90, 180, 360).
        wheel_diameter (float): Wheel diameter in cm (default: 6.5).
        track_width (float): Distance between wheels in cm (default: 10.0).
        ticks_per_rotation (int): Encoder ticks per wheel rotation (default: 20).

    Returns:
        int: Number of ticks required for the active wheel.
    """
    # turn_radius = track_width / 2
    turn_radius = track_width
    # Calculate arc length for the turn
    arc_length = (angle / 360) * 2 * math.pi * turn_radius  # Arc length in cm
    # Calculate wheel rotations
    wheel_circumference = math.pi * wheel_diameter  # ~20.42 cm
    rotations = arc_length / wheel_circumference
    # Convert to encoder ticks
    ticks = int(round(rotations * ticks_per_rotation))
    return ticks

# Updated turn function
def turn(PWM1, PWM2, dutycycle, angle, direction):
    """
    Turns the robot by a specified angle in the given direction using encoder ticks.

    Args:
        PWM1: PWM object for left motor.
        PWM2: PWM object for right motor.
        dutycycle (float): PWM duty cycle (0-100%) for the active motor.
        angle (int): Desired turn angle (90, 180, or 360 degrees).
        direction (bool): True for right turn, False for left turn.

    Uses global variables:
        left_sensor_tick_count, right_sensor_tick_count: Encoder tick counts.
        et1_state: Robot state (updated to IDLE after turn).
    """
    global left_sensor_tick_count, right_sensor_tick_count
    # Validate angle
    # if angle not in [45, 90, 135, 180,225, 270, 315, 360]:
    #     print(f"Error: Invalid angle {angle}. Supported angles: 90, 180, 360")
    #     return

    # Calculate ticks based on robot parameters
    ticks = angle_to_ticks(angle, wheel_diameter=6.6, ticks_per_rotation=20, track_width= 10.0)
    timeout = 5.0  # Maximum turn time (seconds) to prevent infinite loops

    PWM1.ChangeDutyCycle(dutycycle)
    PWM2.ChangeDutyCycle(0)

    if direction:  # Right turn
        # Left motor active, right motor stopped for pivot turn
        print(f"Turning right {angle} degrees ({ticks} ticks)")
        M1_forward()  
    else:  # Left turn
        print(f"Turning left {angle} degrees ({ticks} ticks)")
        M1_backward()

    
    left_count_init = left_sensor_tick_count
    start_time = time.time()
    while (left_sensor_tick_count - left_count_init) < ticks and time.time() - start_time < timeout:
        time.sleep(0.001)  # Non-blocking loop for encoder updates

    # Stop motors
    PWM1.ChangeDutyCycle(0)
    PWM2.ChangeDutyCycle(0)
    # Reset tick counts
    # left_sensor_tick_count = 0
    # right_sensor_tick_count = 0

# Function to move forward with obstacle avoidance using a state machine
def move_forward_obstracle(PWM_1, PWM_2, duty, interval=20):
    """
    Moves the robot forward with obstacle avoidance, using a state machine to handle
    forward movement, backward movement, right turns, and stopping based on distance sensor.
    
    Args:
        PWM_1: PWM object for left motor.
        PWM_2: PWM object for right motor.
        duty (float): PWM duty cycle (0-100%) for motor speed.
        interval (float): Maximum runtime (seconds) for the function (default: 20).
    
    Uses global variables:
        et1_state: Robot state (e.g., MOVE_FORWARD, TURN_RIGHT, STOP).
        left_sensor_tick_count, right_sensor_tick_count: Encoder tick counts for wheels.
    
    Dependencies:
        DistanceSensor: Object for reading distance (assumed to be from a library like gpiozero).
        move_forward, move_backward: External functions for PID-controlled movement.
        RobotState: Enum defining robot states.
    """
    global et1_state, left_sensor_tick_count, right_sensor_tick_count, distance_sensor
    
    # Record start time to track runtime
    runtime_start = time.time()
    # Set initial state to MOVE_FORWARD
    et1_state = RobotState.MOVE_FORWARD
    # Run until the specified interval is reached
    while time.time() - runtime_start < interval:
        # Read distance in centimeters (convert from meters)
        sensor_distance = distance_sensor.distance * 100
        # State transitions based on distance
        if sensor_distance <= (STOP_DISTANCE-(max(10,int(.1*duty)))):
            # Obstacle detected within 5-35 cm, turn right
            et1_state = RobotState.MOVE_BACKWARD
            # print("setting to stopped")
            # print(f"setting state Turn right : {et1_state}")
        elif  (STOP_DISTANCE-(max(10,int(.1*duty)))) < sensor_distance < (STOP_DISTANCE+(max(10,int(.1*duty)))):
            et1_state = RobotState.TURN_ANGLE
            # print("SLOWING DOWN")
        else:
            # No obstacle (distance > 35 cm), move forward if not already doing so
            et1_state = RobotState.STOP

        # Handle actions for each state
        if et1_state == RobotState.MOVE_FORWARD:
            # Move forward with PID control for 0.1 seconds
            move_forward(PWM_1, PWM_2, duty, 0.1)
            # print(f"FWD: dist : {sensor_distance:.2f}, lft_tick : {left_sensor_tick_count}, rgt_tick : {right_sensor_tick_count}")
        elif et1_state == RobotState.MOVE_BACKWARD:
            # Move backward with PID control for 0.1 seconds
            move_backward(PWM_1, PWM_2, duty, 0.1)
        elif et1_state == RobotState.TURN_ANGLE:
            # Stop motors briefly before turning
            PWM_1.ChangeDutyCycle(0)
            PWM_2.ChangeDutyCycle(0)
            time.sleep(0.2)
            # Perform a 90-degree right turn
            turn(PWM_1, PWM_2, duty, random.randrange(0,360), random.choice([True,False]))
            et1_state = RobotState.IDLE
            time.sleep(0.5)  # Pause after turn to stabilize
        elif et1_state == RobotState.IDLE:
            # Stop motors in IDLE state
            PWM_1.ChangeDutyCycle(0)
            PWM_2.ChangeDutyCycle(0)
            time.sleep(0.1)
        elif et1_state == RobotState.SLOWDOWN:
            # Slow down motors in SLOWDOWN state
            slowduty = int(((sensor_distance)/100)*duty)
            PWM_1.ChangeDutyCycle(slowduty)
            PWM_2.ChangeDutyCycle(slowduty)
            # print(f"FWD: dist : {sensor_distance:.2f}, lft_tick : {left_sensor_tick_count}, rgt_tick : {right_sensor_tick_count}, slow duty : {slowduty}")
            time.sleep(0.1)
        elif et1_state == RobotState.STOP:
            # Stop motors and exit loop
            # print("Stopped")
            PWM_1.ChangeDutyCycle(0)
            PWM_2.ChangeDutyCycle(0)
            # print(f"FWD: dist : {sensor_distance:.2f}, lft_tick : {left_sensor_tick_count}, rgt_tick : {right_sensor_tick_count}")
            break
        # Small delay for state transitions and sensor readings
        time.sleep(0.01)

    # Stop motors when exiting
    PWM_1.ChangeDutyCycle(0)
    PWM_2.ChangeDutyCycle(0)
    # Set final state to STOP
    et1_state = RobotState.STOP

# Callback function for left sensor detection to count ticks
def sensor_left(Channel):
    global left_sensor_tick_count
    left_sensor_tick_count += 1

# Callback function for right sensor detection to count ticks    
def sensor_right(Channel):
    global right_sensor_tick_count
    right_sensor_tick_count += 1

# Cleanup function
def cleanup():
    """
    Cleans up GPIO resources, stops PWM, and closes sensor connections.
    """
    global PWM_1, PWM_2, distance_sensor
    print("Cleaning up GPIO and resources...")
    try:
        # Close gpiozero devices first
        if distance_sensor:
            distance_sensor.close()
            distance_sensor = None
        # Stop PWM signals
        if PWM_1:
            PWM_1.stop()
        if PWM_2:
            PWM_2.stop()
        # Remove event detection for encoders
        try:
            GPIO.remove_event_detect(SENSOR_LEFT)
        except:
            pass  # Ignore if already removed
        try:
            GPIO.remove_event_detect(SENSOR_RIGHT)
        except:
            pass  # Ignore if already removed

        print("Cleanup complete.")
    except Exception as e:
        print(f"Error during cleanup: {e}")


try:
    # Initialize PWM for both motors and set initial duty cycle to 0
    PWM_1 = M1_setup()
    PWM_1.start(0)                         # Start PWM for Motor 1 with 0% duty cycle
    PWM_2 = M2_setup()
    PWM_2.start(0)                         # Start PWM for Motor 2 with 0% duty cycle

    GPIO.add_event_detect(SENSOR_LEFT, GPIO.RISING, callback = sensor_left)
    GPIO.add_event_detect(SENSOR_RIGHT, GPIO.RISING, callback = sensor_right)

    # Initialize distance sensor (e.g., HC-SR04) with specified GPIO pins
    distance_sensor = DistanceSensor(echo=27, trigger=25)
 
    # duty = int(input("enter the duty cycle : "))
    # interval = int(input("enter the intervel : "))

    duty = 50
    interval = 5

    move_forward_obstracle(PWM_1, PWM_2,duty, interval)
    # print(f"Left ticks: {left_sensor_tick_count}, Right ticks: {right_sensor_tick_count} ")
    # print("forward done")

    PWM_1.ChangeDutyCycle(0)
    PWM_2.ChangeDutyCycle(0)
    time.sleep(0.5)  # Debounce pushbutton

    sensor_dist = distance_sensor.distance * 100

    data = {
    "distance": sensor_dist,
    "left_ticks" : left_sensor_tick_count,
    "right_ticks" : right_sensor_tick_count
    }
    
    print(json.dumps(data))

except KeyboardInterrupt:
    print("\nProgram interrupted by user.")
finally:
    cleanup()  # Ensure cleanup on exit

            