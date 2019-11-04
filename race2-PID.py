from picamera.array import PiYUVArray, PiRGBArray
from picamera import PiCamera

from scipy.signal import find_peaks, butter, filtfilt

import time
import matplotlib.pyplot as plt
import skimage as ski
import numpy as np

# Camera resolution

res = (640, 480)

CAMERA_CENTER = res[0] // 2
       
from pwm import PWM

# Enable servo
SERVO_MIDDLE = 1500000

servo = PWM(1)
servo.period = 20000000
servo.duty_cycle = SERVO_MIDDLE
servo.enable = True

# Enable motor
MOTOR_BRAKE = 1000000

motor = PWM(0)
motor.period = 20000000
motor.duty_cycle = MOTOR_BRAKE
motor.enable = True

motor.duty_cycle = MOTOR_BRAKE 

# Run a track detection algorithm on a single horizontal line.
# Uses YUV420 image format as the Y component corresponds to image intensity (gray image)
# and thus there is no need to convert from RGB to BW

RUN_TIMER = 30 # seconds
history=[]

camera = PiCamera()
        
# Check the link below for the combinations between mode and resolution
# https://picamera.readthedocs.io/en/release-1.13/fov.html#sensor-modes
camera.sensor_mode = 7
camera.resolution = res
camera.framerate = 10

# Initialize the buffer and start capturing
rawCapture = PiYUVArray(camera, size=res)
stream = camera.capture_continuous(rawCapture, format="yuv", use_video_port=True)

# Measure the time needed to process 300 images to estimate the FPS
t = time.time()

# To filter the noise in the image we use a 3rd order Butterworth filter

# Wn = 0.02, the cut-off frequency, acceptable values are from 0 to 1
b, a = butter(3, 0.1)

line_pos    = CAMERA_CENTER
first_frame = True

# start car
motor.duty_cycle = MOTOR_BRAKE + 120000
track_width = 0

# PID constants for sensitivity
_p = 1.0
i = 0.09
d = 0.08

# array to keep track of pid data
error_history = np.array([],dtype=int)
history_length = 5

for f in stream:
    if first_frame:
        first_frame = False
        # Reset the buffer for the next image
        rawCapture.truncate(0)
        continue
    
    # Stop after RUN_TIMER seconds
    if (time.time() - t) > RUN_TIMER:
        break
        
    # Get the intensity component of the image (a trick to get black and white images)
    I = f.array[:, :, 0]
    
    # Reset the buffer for the next image
    rawCapture.truncate(0)
    
    # Select a horizontal line in the middle of the image
    L = I[195, :]

    # Smooth the transitions so we can detect the peaks 
    Lf = filtfilt(b, a, L)
    history.append(Lf)
    
    # Find peaks which are higher than 0.5
    p = find_peaks(Lf, height=160)
      
    peaks = p[0]
    
    
    line_left   = None
    line_right  = None
    peaks_left  = peaks[peaks < CAMERA_CENTER]
    peaks_right = peaks[peaks > CAMERA_CENTER]
    
    # state machine
    
    if peaks_left.size:
        line_left = peaks_left.max()

    if peaks_right.size:
        line_right = peaks_right.min()
        
    if line_left and line_right:
        line_pos    = (line_left + line_right ) // 2
        track_width = line_right - line_left
        motor.duty_cycle = MOTOR_BRAKE + 200000
        
    elif line_left and not line_right:
        line_pos    = line_left + int(track_width / 2)
        motor.duty_cycle = MOTOR_BRAKE + 150000
        
    elif not line_left and line_right:
        line_pos    = line_right - int(track_width / 2)
        motor.duty_cycle = MOTOR_BRAKE + 150000
        
    else:
        track_width = 400
        print("no line")
        
    print(line_pos, peaks)

    # get error
    current_err = CAMERA_CENTER - line_pos
    
    # collect/trim error history
    error_history = np.append(error_history, current_err)
    if len(error_history)>history_length:
        error_history = error_history[-history_length:]
        
    #get pid output
    if(len(error_history) > 1):
        integral_array = np.cumsum(error_history)
        integral_result = float(integral_array[-1:])

        derivative_array = np.gradient(error_history)
        derivative_result = float(derivative_array[-1:])
    else:
        integral_result = 0
        derivative_result = 0
        
    
    output = _p*float(current_err) + i*integral_result + d*derivative_result
    print("output = ", output)
    DUTY_CYCLE = SERVO_MIDDLE + 5000*int(output)
    if DUTY_CYCLE > 2000000:
        DUTY_CYCLE = 2000000
    if DUTY_CYCLE < 1000000:
        DUTY_CYCLE = 1000000
        
    servo.duty_cycle =  DUTY_CYCLE

    
#Initialize lines position
#Check which lines are closer them in the next frame

        
        #print(line_pos)
        
motor.duty_cycle = MOTOR_BRAKE    
    

# Release resources
stream.close()
rawCapture.close()
camera.close()