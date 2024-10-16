import math
import time

class PIDController:
    def __init__(self, kp, ki, kd, dt, integral_limit=None, output_limit=None, derivative_filter=0.1):
        self.kp = kp
        self.ki = ki
        self.kd = kd
        self.dt = dt
        self.integral = 0
        self.prev_err = 0
        self.integral_limit = integral_limit
        self.output_limit = output_limit
        self.derivative_filter = derivative_filter
        self.prev_derivative = 0

    def compute(self, err):
        # Integral with anti-windup
        self.integral += err * self.dt
        # if self.integral_limit is not None:
        #     self.integral = max(min(self.integral, self.integral_limit), -self.integral_limit)

        # Derivative with filtering
        derivative = (err - self.prev_err) / self.dt
        # derivative = self.derivative_filter * derivative + (1 - self.derivative_filter) * self.prev_derivative

        # PID response
        response = self.kp * err + self.ki * self.integral + self.kd * derivative

        # # Limit output
        # if self.output_limit is not None:
        #     response = max(min(response, self.output_limit), -self.output_limit)

        # Update previous values
        self.prev_err = err
        self.prev_derivative = derivative

        return response

def sysCall_init():
    sim = require('sim')

    self.body = sim.getObject('/body')
    self.left_motor = sim.getObject('/left_joint')
    self.right_motor = sim.getObject('/right_joint')

    # Initialize PID controller for pitch (balancing)
    self.pid_pitch = PIDController(
        kp=15,  # Start with a reasonable value
        ki=2,  # Start with a reasonable value
        kd=0,  # Start with a reasonable value
        dt=0.5,
        integral_limit=5,
        output_limit=20,  # Limit the output to prevent excessive motor commands
        derivative_filter=0.1
    )

    # Initialize PID controller for position
    self.pid_pos = PIDController(
        kp=120,  # Start with a reasonable value
        ki=2,  # Start with a reasonable value
        kd=0,  # Start with a reasonable value
        dt=0.5,
        integral_limit=5,
        output_limit=20,  # Limit the output to prevent excessive motor commands
        derivative_filter=0.1
    )

    self.stationary = True

    # curr state
    self.pitch = 0
    self.pos = 0
    self.pos_error = 0
    
    self.setpoint_pitch = 0
    # PID gains
    # gains
    self.pos_gain = 0

    # integral PID controllers
    self.p_integral = 0
    self.pos_integral = 0

    self.target_pos = 0
    # Variable for keyboard control (rotation only)
    self.manual_rotate = 0
    self.manual_rotate_speed = 10  # Adjust this to control rotation speed

    self.pitch_error = 0

def sysCall_actuation():
    pos_gain = 0
    pitch_gain = self.pid_pitch.compute(self.pitch_error)

    if self.stationary:
        pos_gain = self.pid_pos.compute(self.pos_error)

    vel = pitch_gain - pos_gain
    left_motor_vel = vel * 10 + self.manual_rotate * self.manual_rotate_speed
    right_motor_vel = vel * 10 - self.manual_rotate * self.manual_rotate_speed

    sim.setJointTargetVelocity(self.left_motor, left_motor_vel)
    sim.setJointTargetVelocity(self.right_motor, right_motor_vel)

def sysCall_sensing():
    eulerAngles = sim.getObjectOrientation(self.body)
    roll, yaw, self.pitch = sim.alphaBetaGammaToYawPitchRoll(eulerAngles[0], eulerAngles[1], eulerAngles[2])

    pitch = eulerAngles[2]
    deg = pitch * (180 / math.pi)
    self.pitch_error = self.setpoint_pitch - self.pitch

    if -45 <= deg <= 45:
        self.pos = sim.getObjectPosition(self.body, -1)[1]
        self.pos_error = self.target_pos - self.pos
    elif -135 < deg < -45:
        self.pos = sim.getObjectPosition(self.body, -1)[0]
        self.pos_error = self.target_pos - self.pos
    elif -180 < deg <= -135 or 135 < deg <= 180:
        self.pos = sim.getObjectPosition(self.body, -1)[1]
        self.pos_error = self.target_pos + self.pos
    elif 45 < deg < 135:
        self.pos = sim.getObjectPosition(self.body, -1)[0]
        self.pos_error = self.target_pos + self.pos

    message, data, data2 = sim.getSimulatorMessage()
    if message == sim.message_keypress:
        if data[0] == 2007:  # forward up arrow
            # self.target_pos += 0.007
            self.stationary = False
            self.setpoint_pitch = 0.02
        elif data[0] == 2008:  # backward down arrow
            self.stationary = False
            self.setpoint_pitch = -0.02
        elif (data[0] == 2009):  # left arrow key (rotate left)
            self.manual_rotate = 1  # Positive for left rotation
        elif (data[0] == 2010):  # right arrow key (rotate right)
            self.manual_rotate = -1  # Negative for right rotation
        elif data[0] == 113:  # 'q' key
            self.stationary = True
            self.setpoint_pitch = 0
            self.manual_rotate = 0

            # Reset position error
            if -45 <= deg <= 45:
                self.target_pos = sim.getObjectPosition(self.body, -1)[1]
            elif -135 < deg < -45:
                self.target_pos = sim.getObjectPosition(self.body, -1)[0]
            elif -180 < deg <= -135 or 135 < deg <= 180:
                self.target_pos = sim.getObjectPosition(self.body, -1)[1]
            elif 45 < deg < 135:
                self.target_pos = sim.getObjectPosition(self.body, -1)[0]

            self.pos_error = 0
    else:
        self.manual_rotate = 0



def PID(kp, ki, kd, err, prev_err, integral):
    derivative = (err - prev_err)/0.5
    integral += err*0.5
    response = kp * err + ki * integral + kd * derivative
    return response, integral

def sysCall_cleanup():
    # do some clean-up here
    # This function will be executed when the simulation ends
    
    ####### ADD YOUR CODE HERE ######
    # Any cleanup (if required) to take the scene back to it's original state after simulation
    # It helps in case simulation fails in an unwanted state.
    #################################
    pass

# See the user manual or the available code snippets for additional callback functions and details