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
        if self.integral_limit is not None:
            self.integral = max(min(self.integral, self.integral_limit), -self.integral_limit)

        # Derivative with filtering
        derivative = (err - self.prev_err) / self.dt
        derivative = self.derivative_filter * derivative + (1 - self.derivative_filter) * self.prev_derivative

        # PID response
        response = self.kp * err + self.ki * self.integral + self.kd * derivative

        # Limit output
        if self.output_limit is not None:
            response = max(min(response, self.output_limit), -self.output_limit)

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
    # print(f"Euclid dot product: {dot_product}")
    if dot_product > 0:
        return distance
    else:
        return -distance

def velocity_direction(body_pos, cone_pos, linear_velocity):
    vec_body_to_cone = np.array([cone_pos[0] - body_pos[0], cone_pos[1] - body_pos[1], cone_pos[2] - body_pos[2]])
    vec_body_to_cone_norm = vec_body_to_cone / np.linalg.norm(vec_body_to_cone)
    linear_velocity_norm = linear_velocity / np.linalg.norm(linear_velocity)
    dot_product = np.dot(vec_body_to_cone_norm, linear_velocity_norm)
    # print(f"Vel dir: {dot_product}")
    return 1 if dot_product > 0 else -1

def sysCall_init():
    sim = require('sim')

    self.body = sim.getObject('/body')
    self.cone = sim.getObject('/body/Cone')
    self.left_motor = sim.getObject('/left_joint')
    self.right_motor = sim.getObject('/right_joint')
    # self.prismatic_joint = sim.getObject('/Prismatic_joint')
    # self.arm_joint = sim.getObject('/arm_joint')

    self.pid_pitch = PIDController(kp=48, ki=3, kd=7, dt=0.5, integral_limit=5, output_limit=20, derivative_filter=0.1)
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

    self.prev_vel = 0

def sysCall_actuation():
    self.pitch_gain = self.pid_pitch.compute(self.pitch_error)

    if self.stationary:
        self.pos_gain = self.pid_pos.compute(self.pos_error)

    vel = self.pitch_gain - self.pos_gain

    print(f"Vel: {vel}")

    print(f"pos error: {self.pos_error}")
    print(f"pitch error: {self.pitch_error}")

    print(f"pos gain: {self.pos_gain}")
    print(f"pitch gain: {self.pitch_gain}")

    print(f"setpoint: {self.setpoint_pitch}\n")

    body_pos = sim.getObjectPosition(self.body, -1)
    cone_pos = sim.getObjectPosition(self.cone, -1)
    linear_velocity = sim.getObjectVelocity(self.body, -1)[0]

    dir = velocity_direction(body_pos, cone_pos, linear_velocity)

    # Handle deceleration
    if self.decelerating:
        if np.sign(self.prev_vel) != np.sign(dir):
            # Stop movement and stabilize at the current position
            self.setpoint_pos = body_pos
            self.setpoint_pitch = 0 # Set pitch to 0 for balancing
            self.decelerating = False
            print("Deceleration complete.")
            
            return

    self.prev_vel = dir

    left_motor_vel = vel * 12 + self.manual_rotate * self.manual_rotate_speed
    right_motor_vel = vel * 12 - self.manual_rotate * self.manual_rotate_speed

    sim.setJointTargetVelocity(self.left_motor, left_motor_vel)
    sim.setJointTargetVelocity(self.right_motor, right_motor_vel)

def sysCall_sensing():
    eulerAngles = sim.getObjectOrientation(self.body)
    roll, yaw, pitch = sim.alphaBetaGammaToYawPitchRoll(eulerAngles[0], eulerAngles[1], eulerAngles[2])
    self.pitch_error = self.setpoint_pitch - pitch

    body_pos = sim.getObjectPosition(self.body, -1)
    cone_pos = sim.getObjectPosition(self.cone, -1)
    setpoint_pos = self.setpoint_pos

    current_pos_error = calculate_signed_distance(body_pos, cone_pos, setpoint_pos)

    self.pos_error = current_pos_error

    self.vel_offset = 0.26
    message, data, data2 = sim.getSimulatorMessage()

    if message == sim.message_keypress:
        if data[0] == 2007:
            self.manual_rotate = 0
            self.stationary = False
            self.setpoint_pitch = -self.vel_offset
        elif data[0] == 2008:
            self.manual_rotate = 0
            self.stationary = False
            self.setpoint_pitch = self.vel_offset
        elif data[0] == 2009:
            self.manual_rotate = 1
            if not self.stationary:
                self.manual_rotate = 0.6
        elif data[0] == 2010:
            self.manual_rotate = -1
            if not self.stationary:
                self.manual_rotate = -0.6
        # elif (data[0] == 113): # q
        #     if sim.getJointTargetVelocity(self.prismatic_joint) == 0:
        #         sim.setJointTargetVelocity(self.prismatic_joint, -0.1)
        #     else:
        #         sim.setJointTargetVelocity(self.prismatic_joint, 0) 
        # elif (data[0] == 101): # e
        #     if sim.getJointTargetVelocity(self.prismatic_joint) == 0:
        #         sim.setJointTargetVelocity(self.prismatic_joint, 0.1)
        #     else:
        #         sim.setJointTargetVelocity(self.prismatic_joint, 0)  
        # elif (data[0] == 119): # w
        #     if sim.getJointTargetVelocity(self.arm_joint) == 0:
        #         sim.setJointTargetVelocity(self.arm_joint, 1)
        #     else:
        #         sim.setJointTargetVelocity(self.arm_joint, 0) 
        # elif (data[0] == 115): # s
        #     if sim.getJointTargetVelocity(self.arm_joint) == 0:
        #         sim.setJointTargetVelocity(self.arm_joint, -1)
        #     else:
        #         sim.setJointTargetVelocity(self.arm_joint, 0)
        elif data[0] == 122:  # 'z' key for stop
            self.stationary = True
            self.manual_rotate = 0
            self.decelerating = True  # Initiate deceleration
            self.setpoint_pitch = 0  # Set pitch to 0 for balance
            self.setpoint_pos = sim.getObjectPosition(self.body, -1)
            self.pos_error = 0
            self.pitch_error = 0
            self.pos_gain = 0
            self.integral = 0
            self.prev_err = 0

def sysCall_cleanup():
    pass