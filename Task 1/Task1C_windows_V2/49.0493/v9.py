import math
import time
import numpy as np

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

def calculate_signed_distance(body_pos, cone_pos, setpoint_pos):
    delta_x = setpoint_pos[0] - body_pos[0]
    delta_y = setpoint_pos[1] - body_pos[1]
    distance = np.sqrt(delta_x**2 + delta_y**2)
    vec_body_to_setpoint = np.array([delta_x, delta_y])
    vec_body_to_cone = np.array([cone_pos[0] - body_pos[0], cone_pos[1] - body_pos[1]])
    dot_product = np.dot(vec_body_to_setpoint, vec_body_to_cone)
    if dot_product > 0:
        return distance
    else:
        return -distance

def sysCall_init():
    sim = require('sim')

    self.body = sim.getObject('/body')
    self.cone = sim.getObject('/body/Cone')
    self.left_motor = sim.getObject('/left_joint')
    self.right_motor = sim.getObject('/right_joint')

    self.pid_pitch = PIDController(kp=25, ki=3, kd=0, dt=0.5, integral_limit=5, output_limit=20, derivative_filter=0.1)
    self.pid_pos = PIDController(kp=120, ki=2, kd=0, dt=0.5, integral_limit=5, output_limit=20, derivative_filter=0.1)

    self.stationary = True
    self.decelerating = False

    self.setpoint_pitch = 0
    self.setpoint_pos = sim.getObjectPosition(self.body, -1)

    self.pos_error = 0
    self.pitch_error = 0
  
    self.manual_rotate = 0
    self.manual_rotate_speed = 5

    self.pos_gain = 0
    self.pitch_gain = 0

def sysCall_actuation():
    self.pitch_gain = self.pid_pitch.compute(self.pitch_error)

    if self.stationary:
        self.pos_gain = self.pid_pos.compute(self.pos_error)
        
    print(f"pitch_gain: {self.pitch_gain}")
    print(f"pos_gain: {self.pos_gain}")

    vel = self.pitch_gain - self.pos_gain
    print(f"Vel: {vel}\n")  # Debugging statement added

    left_motor_vel = vel * 10 + self.manual_rotate * self.manual_rotate_speed
    right_motor_vel = vel * 10 - self.manual_rotate * self.manual_rotate_speed

    sim.setJointTargetVelocity(self.left_motor, left_motor_vel)
    sim.setJointTargetVelocity(self.right_motor, right_motor_vel)

def sysCall_sensing():
    eulerAngles = sim.getObjectOrientation(self.body)
    roll, yaw, pitch = sim.alphaBetaGammaToYawPitchRoll(eulerAngles[0], eulerAngles[1], eulerAngles[2])
    self.pitch_error = self.setpoint_pitch - pitch

    body_pos = sim.getObjectPosition(self.body, -1)
    cone_pos = sim.getObjectPosition(self.cone, -1)
    setpoint_pos = self.setpoint_pos

    self.pos_error = calculate_signed_distance(body_pos, cone_pos, setpoint_pos)

    message, data, data2 = sim.getSimulatorMessage()
    vel_offset = 0.15
    deceleration_rate = 0.001  # Rate at which to decelerate

    if message == sim.message_keypress:
        if data[0] == 2007:
            self.manual_rotate = 0
            self.stationary = False
            self.setpoint_pitch = -vel_offset
            self.decelerating = False
            print("forward key pressed")
        elif data[0] == 2008:
            self.manual_rotate = 0
            self.stationary = False
            self.setpoint_pitch = vel_offset
            self.decelerating = False
            print("Backward key pressed")
        elif data[0] == 2009:
            self.manual_rotate = 1
            if not self.stationary:
                self.manual_rotate = 0.5
            print("left key pressed")
        elif data[0] == 2010:
            self.manual_rotate = -1
            if not self.stationary:
                self.manual_rotate = -0.5
            print("right key pressed")
        elif data[0] == 113:
            print("bot stopped")
            self.stationary = True
            self.manual_rotate = 0
            self.decelerating = True  # Start decelerating

            self.setpoint_pos = sim.getObjectPosition(self.body, -1)

    # Gradually decelerate to stop
    if self.decelerating:
        if abs(self.setpoint_pitch) > deceleration_rate:
            self.setpoint_pitch -= np.sign(self.setpoint_pitch) * deceleration_rate
        else:
            self.setpoint_pitch = 0
            self.decelerating = False  # Stop decelerating
            # self.stationary = True

        self.pitch_error = self.setpoint_pitch - pitch

def sysCall_cleanup():
    pass