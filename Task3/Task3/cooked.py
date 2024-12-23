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
        # derivative = self.derivative_filter * derivative + (1 - self.derivative_filter) * self.prev_derivative

        # PID response
        response = self.kp * err + self.ki * self.integral + self.kd * derivative

        # # Limit output
        if self.output_limit is not None:
            response = max(min(response, self.output_limit), -self.output_limit)

        # Update previous values
        self.prev_err = err
        self.prev_derivative = derivative

        return response
    
    def reset(self):
        self.integral = 0
        self.prev_err = 0
        self.prev_derivative = 0

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
    
def calculate_signed_velocity(body_pos, cone_pos, linear_velocity):
    vec_body_to_cone = np.array([cone_pos[0] - body_pos[0], cone_pos[1] - body_pos[1]])
    velocity_xy = np.array([linear_velocity[0], linear_velocity[1]])
    velocity_magnitude = np.linalg.norm(velocity_xy)
    dot_product = np.dot(velocity_xy, vec_body_to_cone)
    return velocity_magnitude if dot_product > 0 else -velocity_magnitude

def new_pos():
    eulerAngles = sim.getObjectOrientation(self.body)
    rot = eulerAngles[2]
    pos_x = self.pos_change * math.sin(rot)
    pos_y = self.pos_change * math.cos(rot)
    pos = sim.getObjectPosition(self.body, -1)
    pos[0] += pos_x
    pos[1] += pos_y
    return pos

def sysCall_init():
    sim = require('sim')

    self.body = sim.getObject('/body')
    self.cone = sim.getObject('/body/Cone')
    self.left_motor = sim.getObject('/left_joint')
    self.right_motor = sim.getObject('/right_joint')
    self.prismatic_joint = sim.getObject('/Prismatic_joint')
    self.arm_joint = sim.getObject('/arm_joint')

    self.pid_pitch = PIDController(kp=40, ki=3, kd=5, dt=0.5, integral_limit=None, output_limit=None, derivative_filter=0.1)
    self.pid_pos = PIDController(kp=80, ki=2, kd=5, dt=0.5, integral_limit=None, output_limit=40, derivative_filter=0.1)
    self.pid_vel = PIDController(kp=2, ki=1, kd=0, dt=0.5, integral_limit=None, output_limit=None, derivative_filter=0.1)


    self.stationary = True
    self.decelerating = False
    self.stopping = False

    self.setpoint_pitch = 0
    self.setpoint_pos = sim.getObjectPosition(self.body, -1)
    self.setpoint_vel = 0

    self.pos_error = 0
    self.pitch_error = 0
    self.vel_error = 0
    self.pos_change = 0.2

    self.manual_rotate = 0
    self.manual_rotate_speed = 5
    self.deceleration_rate = 0.05

    self.pos_gain = 0
    self.pitch_gain = 0
    self.vel_gain = 0

def sysCall_actuation():
    self.pitch_gain = self.pid_pitch.compute(self.pitch_error)

    if self.stationary:
        self.vel_gain = 0
        self.pos_gain = self.pid_pos.compute(self.pos_error)
    else:
        self.vel_gain = self.pid_vel.compute(self.vel_error)
        self.pos_gain = 0

    vel = 0

    # Adjust the combination of gains based on the stopping state
    if self.stopping:
        print("STOPPING")
        print(abs(self.vel_error))
        # self.pitch_gain = 0
        # self.pos_gain = 0
        # self.vel_gain = 0
        if abs(self.vel_error) > 0.1:
            print("Pitch Error: ", self.pitch_error)
            body_pos = sim.getObjectPosition(self.body, -1)
            cone_pos = sim.getObjectPosition(self.cone, -1)
            linear_velocity, _ = sim.getObjectVelocity(self.body)

            signed_velocity_xy = calculate_signed_velocity(body_pos, cone_pos, linear_velocity)

            if abs(self.pitch_error) > 0.14 and np.sign(self.pitch_error) == np.sign(signed_velocity_xy):
                vel = self.pitch_gain - self.pos_gain - self.vel_gain
            else:
                vel = self.pitch_gain + self.pos_gain - self.vel_gain
        else:
            print("BTE")
            self.stopping = False
            self.stationary = True
            # self.manual_rotate = 0
            self.setpoint_pitch = 0
            self.setpoint_pos = sim.getObjectPosition(self.body, -1)
            self.setpoint_vel = 0

            # Reset PID controllers
            self.pid_pitch.reset()
            self.pid_pos.reset()
            self.pid_vel.reset()

            # Reset errors
            self.pos_error = 0
            self.pitch_error = 0
            self.vel_error = 0
    else:
        vel = self.pitch_gain - self.pos_gain - self.vel_gain

    if self.decelerating:
        vel *= (1 - self.deceleration_rate)
        if abs(self.vel_error) < 0.2:
            print("DONE DEC")
            self.setpoint_pos = sim.getObjectPosition(self.body)
            self.pid_pos.reset()
            self.pid_vel.reset()
            self.pid_pitch.reset()
            self.stopping = False
            self.decelerating = False
            # vel = 0


    left_motor_vel = vel * 10 + self.manual_rotate * self.manual_rotate_speed
    right_motor_vel = vel * 10 - self.manual_rotate * self.manual_rotate_speed

    sim.setJointTargetVelocity(self.left_motor, left_motor_vel)
    sim.setJointTargetVelocity(self.right_motor, right_motor_vel)

    print("Vel gain: ", self.vel_gain)
    print("Pos gain: ", self.pos_gain)
    print("Pitch gain: ", self.pitch_gain)

    print("Vel: ", vel)

def sysCall_sensing():
    pitch_offset = 0.05
    vel_offset = 0.8
    deceleration_rate = 0.001  # Rate at which to decelerate

    message, data, data2 = sim.getSimulatorMessage()
    if message == sim.message_keypress:
        if data[0] == 2007:
            self.manual_rotate = 0
            self.stationary = False
            # self.setpoint_pitch = -pitch_offset
            self.setpoint_vel = vel_offset
            self.decelerating = False
        elif data[0] == 2008:
            self.manual_rotate = 0
            self.stationary = False
            # self.setpoint_pitch = pitch_offset
            self.setpoint_vel = -vel_offset
            self.decelerating = False
        elif data[0] == 2009:
            self.manual_rotate = -1
            if not self.stationary:
                self.manual_rotate = -0.5
        elif data[0] == 2010:
            self.manual_rotate = 1
            if not self.stationary:
                self.manual_rotate = 0.5
        elif data[0] == 122:
            print("bot stopped")
            self.stationary = True
            self.stopping = True
            self.decelerating = True
            self.manual_rotate = 0
            self.setpoint_pitch = 0
            self.setpoint_pos = sim.getObjectPosition(self.body, -1)
            self.setpoint_vel = 0

            # Reset PID controllers
            self.pid_pitch.reset()
            self.pid_pos.reset()
            self.pid_vel.reset()

            # Reset errors
            self.pos_error = 0
            self.pitch_error = 0
            self.vel_error = 0
        elif (data[0] == 113): # q
            if sim.getJointTargetVelocity(self.prismatic_joint) == 0:
                sim.setJointTargetVelocity(self.prismatic_joint, -0.1)
            else:
                sim.setJointTargetVelocity(self.prismatic_joint, 0) 
        elif (data[0] == 101): # e
            if sim.getJointTargetVelocity(self.prismatic_joint) == 0:
                sim.setJointTargetVelocity(self.prismatic_joint, 0.1)
            else:
                sim.setJointTargetVelocity(self.prismatic_joint, 0)  
        elif (data[0] == 119): # w
            if sim.getJointTargetVelocity(self.arm_joint) == 0:
                sim.setJointTargetVelocity(self.arm_joint, 1)
            else:
                sim.setJointTargetVelocity(self.arm_joint, 0) 
        elif (data[0] == 115): # s
            if sim.getJointTargetVelocity(self.arm_joint) == 0:
                sim.setJointTargetVelocity(self.arm_joint, -1)
            else:
                sim.setJointTargetVelocity(self.arm_joint, 0)

            # # Create a dummy object at the current position
            # dummy_handle = sim.createDummy(0.05, None)
            # sim.setObjectPosition(dummy_handle, -1, self.setpoint_pos)
            # sim.setObjectAlias(dummy_handle, "StopPositionMarker")


    eulerAngles = sim.getObjectOrientation(self.body)
    roll, yaw, pitch = sim.alphaBetaGammaToYawPitchRoll(eulerAngles[0], eulerAngles[1], eulerAngles[2])
    self.pitch_error = self.setpoint_pitch - pitch

    body_pos = sim.getObjectPosition(self.body, -1)
    cone_pos = sim.getObjectPosition(self.cone, -1)
    setpoint_pos = self.setpoint_pos

    self.pos_error = calculate_signed_distance(body_pos, cone_pos, setpoint_pos)

    # Get the linear velocity of the robot's body
    linear_velocity, _ = sim.getObjectVelocity(self.body)
  

    # Calculate the signed velocity in the xy-plane
    signed_velocity_xy = calculate_signed_velocity(body_pos, cone_pos, linear_velocity)
    # print("LineVel: ", signed_velocity_xy)
    self.vel_error = self.setpoint_vel - signed_velocity_xy

def sysCall_cleanup():
    pass