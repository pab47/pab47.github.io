import math
import pygame
import hiwonder.ActionGroupControl as AGC
import hiwonder.Board as Board
import hiwonder.Mpu6050 as Mpu6050
import cv2
import threading
import time

# Initialize Pygame and the joystick subsystem
pygame.init()
pygame.joystick.init()

if pygame.joystick.get_count() < 1:
    print("No joystick detected!")
    pygame.quit()
    exit()

joystick = pygame.joystick.Joystick(0)
joystick.init()

# Robot control variables
is_crouched = False
last_action_time = 0
l1_pressed = False
r1_pressed = False
l3_pressed = False
cross_pressed = False
triangle_pressed = False
circle_pressed = False
square_pressed = False

# Serializes access to the servo bus - both the joystick loop and the
# background fall-watch thread can trigger action groups, and they must
# never issue AGC.runActionGroup calls at the same time.
action_lock = threading.Lock()

# Head control variables
camera_on = False
cap = None
SERVO_1_MIN = 800
SERVO_1_MAX = 2200
SERVO_2_MIN = 500
SERVO_2_MAX = 2500
SERVO_1_POS = 1500
SERVO_2_POS = 1500

# ---- IMU-based fall detection (runs continuously in the background) ------
# MPU-6050 on the expansion board (I2C address 0x68)
mpu = Mpu6050.mpu6050(0x68)
mpu.set_accel_range(mpu.ACCEL_RANGE_2G)
mpu.set_gyro_range(mpu.GYRO_RANGE_2000DEG)

# Calibration reference (from read_sensors.py), pitch about +Y, + = nose down:
#   standing        : pitch  -6.1 deg
#   fallen forward  : pitch +80.7 deg  (face down)
#   fallen backward : pitch -85.0 deg  (on back)
FALL_PITCH_THRESHOLD_DEG = 45.0
FALL_CHECK_INTERVAL = 0.2       # seconds between background IMU reads
FALL_CONFIRM_COUNT = 2          # consecutive over-threshold reads required before recovering
RECOVERY_COOLDOWN = 1.0         # seconds to pause checking right after a recovery move

def chip_to_body(v):
    """MPU-6050 chip axes -> body axes (X forward, Y left, Z up).
    Found experimentally: upright -> +y_chip up; on face -> +z_chip up; on right side -> -x_chip up."""
    return {'x': -v['z'], 'y': -v['x'], 'z': v['y']}

def get_pitch_deg():
    """Pitch about +Y, positive = leaning forward (nose down). Valid only when ~static."""
    a = chip_to_body(mpu.get_accel_data(g=True))
    return math.degrees(math.atan2(-a['x'], math.hypot(a['y'], a['z'])))

def fall_watch_thread():
    """Runs continuously in the background - no button press needed. Reads the
    IMU every FALL_CHECK_INTERVAL seconds and, once a fall reads consistently
    for FALL_CONFIRM_COUNT samples in a row, plays the matching stand-up move."""
    global is_crouched
    forward_count = 0
    backward_count = 0
    while True:
        pitch = get_pitch_deg()

        if pitch > FALL_PITCH_THRESHOLD_DEG:
            forward_count += 1
            backward_count = 0
        elif pitch < -FALL_PITCH_THRESHOLD_DEG:
            backward_count += 1
            forward_count = 0
        else:
            forward_count = 0
            backward_count = 0

        if forward_count >= FALL_CONFIRM_COUNT:
            print("Fell forward (pitch {:+.1f} deg) - running stand_up_front".format(pitch))
            with action_lock:
                AGC.runActionGroup('stand_up_front')
            is_crouched = False
            forward_count = 0
            time.sleep(RECOVERY_COOLDOWN)
        elif backward_count >= FALL_CONFIRM_COUNT:
            print("Fell backward (pitch {:+.1f} deg) - running stand_up_back".format(pitch))
            with action_lock:
                AGC.runActionGroup('stand_up_back')
            is_crouched = False
            backward_count = 0
            time.sleep(RECOVERY_COOLDOWN)

        time.sleep(FALL_CHECK_INTERVAL)

# Function to adjust the robot's head position
def adjust_head(x_axis, y_axis):
    global SERVO_1_POS, SERVO_2_POS
    SERVO_1_POS += (y_axis * -1/10)
    SERVO_2_POS += (x_axis * -1/10)
    SERVO_1_POS = max(min(SERVO_1_POS, SERVO_1_MAX), SERVO_1_MIN)
    SERVO_2_POS = max(min(SERVO_2_POS, SERVO_2_MAX), SERVO_2_MIN)
    Board.setPWMServoPulse(1, SERVO_1_POS, 500)
    Board.setPWMServoPulse(2, SERVO_2_POS, 500)

# Camera thread function
def camera_thread():
    global cap, camera_on
    while True:
        if camera_on and (cap is None or not cap.isOpened()):
            cap = cv2.VideoCapture(0)
        if not camera_on and cap is not None:
            cap.release()
            cap = None
            cv2.destroyAllWindows()
        if camera_on and cap is not None and cap.isOpened():
            ret, frame = cap.read()
            if ret:
                cv2.imshow("Camera Feed", frame)
                cv2.waitKey(1)
        time.sleep(0.05)  # Slight delay to reduce CPU usage

# Start the camera thread
threading.Thread(target=camera_thread, daemon=True).start()

# Set initial servo positions for the robot's head
Board.setPWMServoPulse(1, SERVO_1_POS, 500)
Board.setPWMServoPulse(2, SERVO_2_POS, 500)

# Make the robot stand at the start
with action_lock:
    AGC.runActionGroup('stand')

# Start the fall watchdog once the robot is standing - runs for the rest of
# the program automatically, with no button press required.
threading.Thread(target=fall_watch_thread, daemon=True).start()

try:
    while True:
        pygame.event.pump()  # Process event queue
        current_time = time.time()

        x_axis_left = joystick.get_axis(0)  # Left joystick left-right
        y_axis_left = joystick.get_axis(1)  # Left joystick up-down
        x_axis_right = joystick.get_axis(3)  # Right joystick left-right
        y_axis_right = joystick.get_axis(4)  # Right joystick up-down

        # Movement and turning control
        if not is_crouched and current_time - last_action_time > 1.1:
            if y_axis_left < -0.5:
                with action_lock:
                    AGC.runActionGroup('go_forward')
                last_action_time = current_time
            elif y_axis_left > 0.5:
                with action_lock:
                    AGC.runActionGroup('go_backward')
                last_action_time = current_time

            # Sideways movement (strafe) with left joystick left-right
            if x_axis_left < -0.5:
                with action_lock:
                    AGC.runActionGroup('left_move')
                last_action_time = current_time
            elif x_axis_left > 0.5:
                with action_lock:
                    AGC.runActionGroup('right_move')
                last_action_time = current_time

            # Turning with left and right bumpers
            current_l1_state = joystick.get_button(4)
            current_r1_state = joystick.get_button(5)
            if current_l1_state and not l1_pressed:
                with action_lock:
                    AGC.runActionGroup('turn_left_fast')
                last_action_time = current_time
                l1_pressed = True
            elif not current_l1_state:
                l1_pressed = False

            if current_r1_state and not r1_pressed:
                with action_lock:
                    AGC.runActionGroup('turn_right_fast')
                last_action_time = current_time
                r1_pressed = True
            elif not current_r1_state:
                r1_pressed = False

        # Crouch/Uncrouch with L3
        current_l3_state = joystick.get_button(11)
        if current_l3_state and not l3_pressed:
            if not is_crouched:
                with action_lock:
                    AGC.runActionGroup('squat')
                is_crouched = True
                print("Robot crouched")
            else:
                with action_lock:
                    AGC.runActionGroup('stand')
                is_crouched = False
                print("Robot standing")
            l3_pressed = True
        elif not current_l3_state and l3_pressed:
            l3_pressed = False


        # Head movement control with right joystick
        if abs(x_axis_right) > 0.1 or abs(y_axis_right) > 0.1:
            adjust_head(x_axis_right, y_axis_right)

        # Sit-ups with Square button
        if joystick.get_button(3) and not square_pressed:  # Square button
            with action_lock:
                AGC.runActionGroup('sit_ups')
            square_pressed = True
        elif not joystick.get_button(3):
            square_pressed = False

        # Toggle Dance with Triangle button          
        if joystick.get_button(2) and not triangle_pressed:  # Triangle button
            with action_lock:
                AGC.runActionGroup('twist')
            triangle_pressed = True
        elif not joystick.get_button(2):
            triangle_pressed = False

        # Toggle Chest Pound with Cross button
        if joystick.get_button(0) and not cross_pressed:  # Cross button
            with action_lock:
                AGC.runActionGroup('chest')
            cross_pressed = True
        elif not joystick.get_button(0):
            cross_pressed = False

        # Toggle Wave with Circle button
        if joystick.get_button(1) and not circle_pressed:  # Circle button
            with action_lock:
                AGC.runActionGroup('wave')
            circle_pressed = True
        elif not joystick.get_button(1):
            circle_pressed = False

except KeyboardInterrupt:
    if cap is not None:
        cap.release()
    cv2.destroyAllWindows()
    pygame.quit()
