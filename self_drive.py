import time
import RPi.GPIO as GPIO
from gpiozero import Servo
import cv2
import numpy as np
import math

# Set the GPIO mode to BCM
GPIO.setmode(GPIO.BCM)

# # Set up GPIO pin 14 as the throttle pin
throttle_pin = 14
throttle_on = False
GPIO.setup(throttle_pin, GPIO.OUT)
GPIO.output(throttle_pin, throttle_on)

# # Set up GPIO pin 13 as the servo pin
servo = Servo(13)

def set_servo_turn_amt(value):
    servo.value = value

def toggle_throttle(on=False, off=False):
    global throttle_on
    if on:
        GPIO.output(throttle_pin, on)
        throttle_on = True
    elif off:
        GPIO.output(throttle_pin, False)
        throttle_on = False
    else:
        GPIO.output(throttle_pin, not throttle_on)
        throttle_on = not throttle_on


def detect_edges(frame):
    hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
    lower_blue = np.array([90, 120, 0], dtype = "uint8")
    upper_blue = np.array([120, 255, 255], dtype="uint8")
    mask = cv2.inRange(hsv,lower_blue,upper_blue)
    # detect edges
    edges = cv2.Canny(mask, 50, 100) 
    cv2.imshow("edges",edges)
    return edges


def detect_stop_sign(frame):
    hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
    lower_blue = np.array([0, 40, 60], dtype = "uint8")
    upper_blue = np.array([20, 80, 100], dtype="uint8")
    mask = cv2.inRange(hsv,lower_blue,upper_blue)
    # detect stop sign
    num_red_px = cv2.countNonZero(mask)
    print("num_red_px: ", num_red_px)
    return num_red_px >= 30

def region_of_interest(edges):
    height, width = edges.shape # extract the height and width of the edges frame
    mask = np.zeros_like(edges) # make an empty matrix with same dimensions of the edges frame
    # only focus lower half of the screen
    # specify the coordinates of 4 points (lower left, upper left, upper right, lower right)
    polygon = np.array([[
        (0, height), 
        (0,  height/2),
        (width , height/2),
        (width , height),
    ]], np.int32)
    cv2.fillPoly(mask, polygon, 255) # fill the polygon with blue color 
    cropped_edges = cv2.bitwise_and(edges, mask) 
    cv2.imshow("roi",cropped_edges)
    return cropped_edges

def detect_line_segments(cropped_edges):
    rho = 1  
    theta = np.pi / 180  
    min_threshold = 10 
    line_segments = cv2.HoughLinesP(cropped_edges, rho, theta, min_threshold, 
                                    np.array([]), minLineLength=5, maxLineGap=0)
    return line_segments

def average_slope_intercept(frame, line_segments):
    lane_lines = []
    if line_segments is None:
        print("no line segment detected")
        return lane_lines
    height, width,_ = frame.shape
    left_fit = []
    right_fit = []
    boundary = 1/3
    left_region_boundary = width * (1 - boundary) 
    right_region_boundary = width * boundary 
    for line_segment in line_segments:
        for x1, y1, x2, y2 in line_segment:
            if x1 == x2:
                print("skipping vertical lines (slope = infinity)")
                continue
            fit = np.polyfit((x1, x2), (y1, y2), 1)
            slope = (y2 - y1) / (x2 - x1)
            intercept = y1 - (slope * x1)
            if slope < 0:
                if x1 < left_region_boundary and x2 < left_region_boundary:
                    left_fit.append((slope, intercept))
            else:
                if x1 > right_region_boundary and x2 > right_region_boundary:
                    right_fit.append((slope, intercept))
    left_fit_average = np.average(left_fit, axis=0)
    if len(left_fit) > 0:
        lane_lines.append(make_points(frame, left_fit_average))
    right_fit_average = np.average(right_fit, axis=0)
    if len(right_fit) > 0:
        lane_lines.append(make_points(frame, right_fit_average))
    # lane_lines is a 2-D array consisting the coordinates of the right and left lane lines
    # for example: lane_lines = [[x1,y1,x2,y2],[x1,y1,x2,y2]]
    # where the left array is for left lane and the right array is for right lane 
    # all coordinate points are in pixels
    return lane_lines

def make_points(frame, line):
    height, width, _ = frame.shape
    slope, intercept = line
    y1 = height  # bottom of the frame
    y2 = int(y1 / 2)  # make points from middle of the frame down
    if slope == 0: 
        slope = 0.1    
    x1 = int((y1 - intercept) / slope)
    x2 = int((y2 - intercept) / slope)
    return [[x1, y1, x2, y2]]

def display_lines(frame, lines, line_color=(0, 255, 0), line_width=6): # line color (B,G,R)
    line_image = np.zeros_like(frame)
    if lines is not None:
        for line in lines:
            for x1, y1, x2, y2 in line:
                cv2.line(line_image, (x1, y1), (x2, y2), line_color, line_width)
    line_image = cv2.addWeighted(frame, 0.8, line_image, 1, 1)  
    return line_image


def get_steering_angle(frame, lane_lines):
    height, width, _ = frame.shape
    if len(lane_lines) == 2: # if two lane lines are detected
        _, _, left_x2, _ = lane_lines[0][0] # extract left x2 from lane_lines array
        _, _, right_x2, _ = lane_lines[1][0] # extract right x2 from lane_lines array
        mid = int(width / 2)
        x_offset = (left_x2 + right_x2) / 2 - mid
        y_offset = int(height / 2)  
    elif len(lane_lines) == 1: # if only one line is detected
        x1, _, x2, _ = lane_lines[0][0]
        x_offset = x2 - x1
        y_offset = int(height / 2)
    elif len(lane_lines) == 0: # if no line is detected
        x_offset = 0
        y_offset = int(height / 2)
    angle_to_mid_radian = math.atan(x_offset / y_offset)
    angle_to_mid_deg = int(angle_to_mid_radian * 180.0 / math.pi)  
    steering_angle = angle_to_mid_deg + 90 
    return steering_angle

def display_heading_line(frame, steering_angle, line_color=(0, 0, 255), line_width=5):
    heading_image = np.zeros_like(frame)
    height, width, _ = frame.shape
    steering_angle_radian = steering_angle / 180.0 * math.pi
    x1 = int(width / 2)
    y1 = height
    x2 = int(x1 - height / 2 / math.tan(steering_angle_radian))
    y2 = int(height / 2)
    cv2.line(heading_image, (x1, y1), (x2, y2), line_color, line_width)
    heading_image = cv2.addWeighted(frame, 0.8, heading_image, 1, 1)
    return heading_image

video = cv2.VideoCapture(0)
video.set(cv2.CAP_PROP_FRAME_WIDTH,320)
video.set(cv2.CAP_PROP_FRAME_HEIGHT,240)

''' SETTING P AND D VALUES '''

kp = 0.2    # change for testing purposes
kd =  0.5

stop_timing = 0
stop_state = 0

''' PLOTTING DATA '''

error_data = []
steering_pwm_data = []
throttle_pwm_data = []

proportional_resp_data = []
derivative_resp_data = []

p_vals = []
d_vals = []
err_vals = []

''' MAIN LOOP '''

last_update = time.time()
last_cycle = last_update
update_time = 0.1# seconds
stop_time = update_time * 2
last_error = 0
stops = 0
last_stopped = -1
while True:
    ret,frame = video.read()
    frame = cv2.flip(frame,1)
    #Calling the functions
    edges = detect_edges(frame)
    roi = region_of_interest(edges)
    line_segments = detect_line_segments(roi)
    lane_lines = average_slope_intercept(frame,line_segments)
    lane_lines_image = display_lines(frame,lane_lines)
    steering_angle = get_steering_angle(frame, lane_lines)
    heading_image = display_heading_line(lane_lines_image,steering_angle)
    cv2.imshow("heading", heading_image)
    # Check if we should toggle the throttle
    now = time.time()
    if (throttle_on and abs(now - last_update) >= update_time) or (not throttle_on and (now -last_update) >= stop_time):
        last_update = now
        toggle_throttle()
    # OLD STEERING: Adjust the wheel angle
    # if steering_angle < 80:
    #     set_servo_turn_amt(-0.5)
    # elif steering_angle > 100:
    #     set_servo_turn_amt(0.5)
    # else:
    #     set_servo_turn_amt(0)

    ''' CALCULATE DERIVATIVE FROM PD ALGORITHM '''
    now = time.time()
    dt = now - last_cycle
    deviation = steering_angle - 90
    error = -deviation
    base_turn = 0
    proportional = kp * error
    derivative = kd * (error - last_error) / dt
    error_data.append(error)
    proportional_resp_data.append(proportional)
    derivative_resp_data.append(derivative)
    #stop sign check
    if detect_stop_sign(frame):
        if (last_stopped == -1):
            toggle_throttle(False, True)
            #time.sleep(2)
            print("STOP DETECTED STOP DETECTED STOP DETECTED STOP DETECTED ")
            last_stopped = time.time()
            while (time.time() - last_stopped < 3):
                print("waiting") 
        elif (time.time() - last_stopped >= 20):
            toggle_throttle(False, True)
            break
        else:
            print("not waiting")
    ''' FOR PLOTTING PURPOSES '''
    p_vals.append(proportional)
    d_vals.append(derivative)
    err_vals.append(error)
    ''' DETERMINE TURN AMOUNT (WITH PWM CONTROL) '''
    turn_amt = base_turn + proportional + derivative
    if turn_amt < -1:
        turn_amt = -1
    elif turn_amt > 0.5:
        turn_amt = 0.5
    print(f"Turn amt: {turn_amt}")
    set_servo_turn_amt(turn_amt)
    last_error = error
    last_cycle = now
    key = cv2.waitKey(1)
    if key == 27:
        break

# Reset the throttle and servo to neutral
toggle_throttle(True, False)
set_servo_turn_amt(0)

video.release()
cv2.destroyAllWindows()
