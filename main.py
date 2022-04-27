 #import simulation modules (must be removed when integrated with hardware)
import dynamic_model
dynamic_model.enable_wind = False

#import functional modules (must be modified to match correct integration of hardware imports)
import board
import pulseio
import time
import busio
import adafruit_bno055
import pwmio

#setup communication protocol (must be modified to match correct integration of communication)
pulses = pulseio.PulseIn(board.D5,maxlen=1,idle_state=(False))
i2c = busio.I2C(board.SCL,board.SDA)
sensor = adafruit_bno055.BNO055_I2C(i2c)

# PID CONTROLLER CONFIGURATION
# --------------------------------------------
Kp = 0.01  # proportional constant
Kd = 0.006   # derivative constant
Ki = 0.0000005 # integral constant
# --------------------------------------------

t_prev = 0 # previous time sample initialization
I = 0 # integral storage variable initialization
rate_input = 0 # rate input variable initialization

#BEGIN FLY-BY-WIRE OPERATIONS
##############################################
while True:
    #initialize loop start time
    loop_start = time.monotonic()

    # Wait until pulses have been received on pin D5
    while len(pulses)==0:
        pass

    # Recieve tailrotor rate command from the RC controller
    current_pulse = pulses.popleft()
    if current_pulse > 2001:
        continue
    #print (current_pulse)
    # Reset pulse signal acquisition
    pulses.pause()
    pulses.clear()
    pulses.resume()
    
    # Check calibration state of the BNO055
    if sensor.calibrated==True:
        board.D13=True

    # Acquire angular rates from BNO055
    xrate,yrate,zrate=sensor.gyro

    #print(zrate) # yaw rotation rate

    # Convert RC command in ms to a forward loop input in degrees per second
    rate_input = ((current_pulse/1000)-1.5)*20
    print (rate_input)

    ##### Rate Error summing junction between tailrotor input and sampled yaw rotation rate in degrees per second
    rate_error = (rate_input) - zrate
    #print (rate_error)
    #####

    # timing setup for the integral control computation
    t_now = time.monotonic()
    delta_t = t_now - t_prev
    I = I + rate_error*delta_t

    # The "rate output" is the desired rate setting as a result of passing through the controller
    rate_output = Kp*rate_error + Kd*zrate + Ki*I
    #print (rate_output)

    # The following loop limits the maximum yaw output to +/- 0.5 ms, and then commands the yaw motors by using
    # yaw output converted into duty cycle
    if rate_output <= 0.6 and rate_output >= -0.6:
        tailrotor_command = 3932 + rate_output*1310
        tailrotor = pwmio.PWMOut(board.D9, frequency=40, duty_cycle=tailrotor_command)
    elif rate_output > 0.6:
        tailrotor_command = 3932 + 0.5 * 1310
        tailrotor = pwmio.PWMOut(board.D9, frequency=40, duty_cycle=tailrotor_command)
    else:
        tailrotor_command = 3932 + -0.6 * 1310
        tailrotor = pwmio.PWMOut(board.D9, frequency=40, duty_cycle=tailrotor_command)
    t_prev = time.monotonic()
    pass