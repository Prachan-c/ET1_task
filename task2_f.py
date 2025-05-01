import RPi.GPIO as GPIO
import time
from gpiozero import DistanceSensor

GPIO.setwarnings(False)
GPIO.setmode(GPIO.BCM)

# Pin definitions
LED = 7                # Pin for LED
Taster = 5             # Pin for pushbutton

# Motor 1 (Left) pin definitions
Motor1_PWM = 18        # PWM pin for Motor 1
Motor1_IN1 = 17        # Input pin 1 for Motor 1
Motor1_IN2 = 22        # Input pin 2 for Motor 1

# Motor 2 (Right) pin definitions
Motor2_PWM = 19        # PWM pin for Motor 2
Motor2_IN1 = 24        # Input pin 1 for Motor 2
Motor2_IN2 = 4         # Input pin 2 for Motor 2

duty_cycle = 0         # Initialize PWM duty cycle

# Sensor pin definitions
sensor_left = 16       # Pin for left sensor
sensor_right = 23      # Pin for right sensor

# Variables to keep track of sensor tick counts
left_sensor_tick_count = 0
right_sensor_tick_count = 0

MOTOR_ENCODER_TICKS = 20

# GPIO setup for LED and pushbutton
GPIO.setup(LED, GPIO.OUT)                               # Set LED pin as output
GPIO.setup(Taster, GPIO.IN, pull_up_down=GPIO.PUD_UP)   # Set pushbutton pin as input with pull-up resistor
GPIO.setup(sensor_left, GPIO.IN, pull_up_down=GPIO.PUD_UP)
GPIO.setup(sensor_right, GPIO.IN, pull_up_down=GPIO.PUD_UP)



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

# Initialize PWM for both motors and set initial duty cycle to 0
PWM_1 = M1_setup()
PWM_1.start(0)                         # Start PWM for Motor 1 with 0% duty cycle
PWM_2 = M2_setup()
PWM_2.start(0)                         # Start PWM for Motor 2 with 0% duty cycle

def pid_control(target_diff, actual_diff, integral, last_error, dt, kp, ki, kd):
    error = target_diff - actual_diff
    integral += error * dt
    derivative = (error - last_error) / dt if dt > 0 else 0
    output = kp * error + ki * integral + kd * derivative
    # print(f"PID: {(kp * error):.2f}, {(ki * integral):.2f}, {(kd * derivative):.2f}, integ : {integral:.2f}, error: {error:.2f}")
    return output, integral, error

# Function to move forward for given interval
def move_forward(PWM1, PWM2, base_dutycycle, interval):
    global left_sensor_tick_count, right_sensor_tick_count
    start_time = time.time()
    last_time = start_time
    KP = 5.0
    KI = 0.1
    KD = 0.01
    integral = 0.0
    last_error = 0.0
    left_dutycycle = base_dutycycle
    
    # right_offset = int(4.5 + (.1*base_dutycycle))
    right_offset = 0
    right_dutycycle = min(100, (base_dutycycle + right_offset))
    print(f"intial right duty : {right_dutycycle}, left duty: {left_dutycycle}")
    PWM1.ChangeDutyCycle(left_dutycycle)
    PWM2.ChangeDutyCycle(right_dutycycle)
    
    M2_forward()
    M1_forward()


    while(time.time()-start_time < interval):
        current_time = time.time()
        dt = current_time - last_time

        if dt >= 0.02:
            tick_diff = right_sensor_tick_count - left_sensor_tick_count
            adjust, integral, last_error = pid_control(0, tick_diff, integral, last_error, dt, KP, KI, KD )
            if tick_diff < 0:
                left_dutycycle = max(0, min(100, base_dutycycle - adjust))
                # right_dutycycle = max(0, min(100, base_dutycycle + adjust))
                PWM1.ChangeDutyCycle(left_dutycycle)
            else:
                right_dutycycle = max(0, min(100, base_dutycycle + right_offset + adjust))
                # left_dutycycle = max(0, min(100, base_dutycycle + adjust))
                PWM2.ChangeDutyCycle(right_dutycycle)

            # print(f"Backward - Left ticks: {left_sensor_tick_count}, Right ticks: {right_sensor_tick_count}, pid_adjust: {adjust:.4f}, "
            #       f"Diff: {tick_diff}, Left PWM: {left_dutycycle:.1f}, Right PWM: {right_dutycycle:.1f}")
            last_time = current_time
        time.sleep(0.001)
    
    PWM1.ChangeDutyCycle(0)
    PWM2.ChangeDutyCycle(0)

def move_backward(PWM1, PWM2, base_dutycycle, interval):
    global left_sensor_tick_count, right_sensor_tick_count
    start_time = time.time()
    last_time = start_time
    KP = 5.0
    KI = 0.1
    KD = 0.01
    integral = 0.0
    last_error = 0.0
    left_dutycycle = base_dutycycle
    
    # right_offset = int(4.5 + (.1*base_dutycycle))
    right_offset = 0
    right_dutycycle = min(100, (base_dutycycle + right_offset))
    print(f"intial right duty : {right_dutycycle}, left duty: {left_dutycycle}")
    PWM1.ChangeDutyCycle(left_dutycycle)
    PWM2.ChangeDutyCycle(right_dutycycle)
    
    M2_backward()
    M1_backward()


    while(time.time()-start_time < interval):
        current_time = time.time()
        dt = current_time - last_time

        if dt >= 0.02:
            tick_diff = right_sensor_tick_count - left_sensor_tick_count
            adjust, integral, last_error = pid_control(0, tick_diff, integral, last_error, dt, KP, KI, KD )
            if tick_diff < 0:
                left_dutycycle = max(0, min(100, base_dutycycle - adjust))
                # right_dutycycle = max(0, min(100, base_dutycycle + adjust))
                PWM1.ChangeDutyCycle(left_dutycycle)
            else:
                right_dutycycle = max(0, min(100, base_dutycycle + right_offset + adjust))
                # left_dutycycle = max(0, min(100, base_dutycycle + adjust))
                PWM2.ChangeDutyCycle(right_dutycycle)

            # print(f"Backward - Left ticks: {left_sensor_tick_count}, Right ticks: {right_sensor_tick_count}, pid_adjust: {adjust:.4f}, "
            #       f"Diff: {tick_diff}, Left PWM: {left_dutycycle:.1f}, Right PWM: {right_dutycycle:.1f}")
            last_time = current_time
        time.sleep(0.001)
    
    PWM1.ChangeDutyCycle(0)
    PWM2.ChangeDutyCycle(0)


# Function to move backward for given interval
def move_backward_direct(PWM1, PWM2, dutycycle, interval):
    PWM1.ChangeDutyCycle(dutycycle)
    PWM2.ChangeDutyCycle(dutycycle)
    M2_backward()
    M1_backward()
    time.sleep(interval)



# Callback function for left sensor detection to count ticks
def sensor_left(Channel):
    global left_sensor_tick_count
    left_sensor_tick_count += 1

# Callback function for right sensor detection to count ticks    
def sensor_right(Channel):
    global right_sensor_tick_count
    right_sensor_tick_count += 1

# Function to print sensor tick data
def print_ticks(duty, direction):
    global left_sensor_tick_count, right_sensor_tick_count
    print(f"{direction} : Dutycycle: {duty} --> Left ticks: {left_sensor_tick_count}, Right ticks: {right_sensor_tick_count} ")

    # left_sensor_tick_count = 0
    # right_sensor_tick_count = 0

def move_forward_9offset(PWM1, PWM2, dutycycle, interval):
    PWM1.ChangeDutyCycle(dutycycle)
    offset = int(4.5 + (.1*dutycycle))
    right_duty = max(0, min(100, dutycycle + offset ))
    PWM2.ChangeDutyCycle(right_duty)
    print(f"intial right duty : {right_duty}, left duty: {dutycycle}")
    M2_forward()
    M1_forward()
    time.sleep(interval)

GPIO.add_event_detect(16, GPIO.RISING, callback = sensor_left)
GPIO.add_event_detect(23, GPIO.RISING, callback = sensor_right)

def turn(PWM1, PWM2, dutycycle, angle, direction):
    global left_sensor_tick_count, right_sensor_tick_count
    ticks = 0
    if angle == 90:
        ticks = 15
    elif angle == 180:
        ticks = 30
    elif angle == 360:
        ticks = 60
    else:
        ticks = 0

    if direction: #turn left
        PWM1.ChangeDutyCycle(dutycycle)
        PWM2.ChangeDutyCycle(0)
        print("turn left")
        left_sensor_tick_count = 0
        while((left_sensor_tick_count <= ticks) ):
            time.sleep(0.01)
    else: #turn right
        PWM2.ChangeDutyCycle(dutycycle)
        PWM1.ChangeDutyCycle(0)
        right_sensor_tick_count = 0
        while((right_sensor_tick_count <= ticks) ):
            time.sleep(0.01)

    PWM1.ChangeDutyCycle(0)
    PWM2.ChangeDutyCycle(0)


def move_forward_obstracle(PWM1, PWM2, base_dutycycle, interval):
    global left_sensor_tick_count, right_sensor_tick_count
    distance_sensor = DistanceSensor(echo=27, trigger=25)
    start_time = time.time()
    last_time = start_time
    KP = 5.0
    KI = 0.1
    KD = 0.01
    integral = 0.0
    last_error = 0.0
    left_dutycycle = base_dutycycle
    right_dutycycle = base_dutycycle
    print(f"intial right duty : {right_dutycycle}, left duty: {left_dutycycle}")
    PWM1.ChangeDutyCycle(left_dutycycle)
    PWM2.ChangeDutyCycle(right_dutycycle)
    
    M2_forward()
    M1_forward()


    while(time.time()-start_time < interval):
        current_time = time.time()
        dt = current_time - last_time
        sensor_distance = distance_sensor.distance * 100

        if sensor_distance in range(10,30) :
            print(f"distance = {sensor_distance:.2f}")
            PWM1.ChangeDutyCycle(0)
            PWM2.ChangeDutyCycle(0)

            # Logic to turn 90 degree 
            print("start turn")
            turn(PWM1, PWM2, base_dutycycle, 90, True)
            time.sleep(1)
            integral = 0.0
            last_error = 0.0
            left_sensor_tick_count = 0
            right_sensor_tick_count = 0
            print(f"After turn -> Left ticks: {left_sensor_tick_count}, Right ticks: {right_sensor_tick_count} ")
            PWM1.ChangeDutyCycle(base_dutycycle)
            PWM2.ChangeDutyCycle(base_dutycycle)
            last_time = current_time
        elif sensor_distance <=10:
            move_backward(PWM1, PWM2, base_dutycycle, 1)

            

        if dt >= 0.02:
            tick_diff = right_sensor_tick_count - left_sensor_tick_count
            adjust, integral, last_error = pid_control(0, tick_diff, integral, last_error, dt, KP, KI, KD )
            if tick_diff < 0:
                left_dutycycle = max(0, min(100, base_dutycycle - adjust))
                # right_dutycycle = max(0, min(100, base_dutycycle + adjust))
                PWM1.ChangeDutyCycle(left_dutycycle)
            else:
                right_dutycycle = max(0, min(100, base_dutycycle + adjust))
                # left_dutycycle = max(0, min(100, base_dutycycle + adjust))
                PWM2.ChangeDutyCycle(right_dutycycle)

            # print(f"Backward - Left ticks: {left_sensor_tick_count}, Right ticks: {right_sensor_tick_count}, pid_adjust: {adjust:.4f}, "
            #       f"Diff: {tick_diff}, Left PWM: {left_dutycycle:.1f}, Right PWM: {right_dutycycle:.1f}")
            last_time = current_time
        time.sleep(0.001)
    
    PWM1.ChangeDutyCycle(0)
    PWM2.ChangeDutyCycle(0)

def wheel_rotation(PWM1, PWM2, num, dutycycle, direction):
    global left_sensor_tick_count, right_sensor_tick_count
    total_ticks = MOTOR_ENCODER_TICKS * num
    PWM1.ChangeDutyCycle(dutycycle)
    PWM2.ChangeDutyCycle(dutycycle)
    if direction == True:
        M1_forward()
        M2_forward()
    else:
        M1_backward()
        M2_backward()
        
    while((left_sensor_tick_count <= total_ticks) or (right_sensor_tick_count <= total_ticks)):
        #print(f"Left ticks: {left_sensor_tick_count}, Right ticks: {right_sensor_tick_count} ")
        if right_sensor_tick_count == total_ticks:
            PWM2.ChangeDutyCycle(0)
            print("Right motor set to 0 duty")

        if left_sensor_tick_count == total_ticks:
            PWM1.ChangeDutyCycle(0)
            print("Left motor set to 0 duty")

        time.sleep(0.01)

    print_ticks(dutycycle, "Forward" if direction else "Backward")
    left_sensor_tick_count = 0
    right_sensor_tick_count = 0


while (1):
        
        if GPIO.input(Taster) == GPIO.LOW:
            duty = int(input("enter the duty cycle : "))
            interval = int(input("enter the intervel : "))
            # wheel_rotation(PWM_1, PWM_2, 5, 60, True)
            # print("forward done")
            # time.sleep(1)
            # wheel_rotation(PWM_1, PWM_2, 5, 60, False)
            # print("Backword done")

            # move_forward(PWM_1, PWM_2,duty, 5)
            # print(f"Left ticks: {left_sensor_tick_count}, Right ticks: {right_sensor_tick_count} ")
            # print("forward done")
            
            # PWM_1.ChangeDutyCycle(0)
            # PWM_2.ChangeDutyCycle(0)
            # left_sensor_tick_count = 0
            # right_sensor_tick_count = 0

            # move_backward(PWM_1, PWM_2,duty, 5)
            # print(f"Left ticks: {left_sensor_tick_count}, Right ticks: {right_sensor_tick_count} ")
            # print("Backward done")

            # PWM_1.ChangeDutyCycle(0)
            # PWM_2.ChangeDutyCycle(0)
            # left_sensor_tick_count = 0
            # right_sensor_tick_count = 0
            
            move_forward_obstracle(PWM_1, PWM_2,duty, interval)
            print(f"Left ticks: {left_sensor_tick_count}, Right ticks: {right_sensor_tick_count} ")
            print("forward done")
            
            PWM_1.ChangeDutyCycle(0)
            PWM_2.ChangeDutyCycle(0)
            left_sensor_tick_count = 0
            right_sensor_tick_count = 0