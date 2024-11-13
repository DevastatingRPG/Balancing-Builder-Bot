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

        # # Limit output
        if self.output_limit is not None:
            response = max(min(response, self.output_limit), -self.output_limit)

        # Update previous values
        self.prev_err = err
        self.prev_derivative = derivative

        return response

def calculate_distance_with_sign(body_pos, cone_pos, setpoint_pos):
    """
    Calculate the Euclidean distance between body and setpoint positions and determine the sign.
    
    Args:
    body_pos (tuple): The body position as (x, y).
    cone_pos (tuple): The cone position as (x, y).
    setpoint_pos (tuple): The setpoint position as (x, y).
    
    Returns:
    float: The signed distance from body to setpoint.
    """
    delta_x = setpoint_pos[0] - body_pos[0]
    delta_y = setpoint_pos[1] - body_pos[1]
    distance = np.sqrt(delta_x**2 + delta_y**2)
    
    # Vector from body_pos to setpoint_pos
    vec_body_to_setpoint = np.array([delta_x, delta_y])
    
    # Vector from body_pos to cone_pos
    vec_body_to_cone = np.array([cone_pos[0] - body_pos[0], cone_pos[1] - body_pos[1]])
    
    # Normalize the vector from body_pos to setpoint_pos
    vec_body_to_setpoint_normalized = vec_body_to_setpoint / np.linalg.norm(vec_body_to_setpoint)
    
    # Project the vector from body_pos to cone_pos onto the normalized vector from body_pos to setpoint_pos
    projection_length = np.dot(vec_body_to_cone, vec_body_to_setpoint_normalized)
    
    # Calculate the perpendicular distance from cone_pos to the line defined by body_pos and setpoint_pos
    perpendicular_distance = np.linalg.norm(vec_body_to_cone - projection_length * vec_body_to_setpoint_normalized)
    
    # Determine the sign based on the cross product
    cross_product = np.cross(vec_body_to_setpoint, vec_body_to_cone)
    if cross_product > 0:
        return distance
    else:
        return -distance

def sysCall_init():
    sim = require('sim')

    self.body = sim.getObject('/body')
    self.cone = sim.getObject('/body/Cone')
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
        kp=20,  # Start with a reasonable value
        ki=0,  # Start with a reasonable value
        kd=0.5,  # Start with a reasonable value
        dt=0.5,
        integral_limit=5,
        output_limit=20,  # Limit the output to prevent excessive motor commands
        derivative_filter=0.1
    )

    # Initialize additional PID controller for motor velocity adjustment
    self.pid_motor = PIDController(
        kp=3,  # Start with a reasonable value
        ki=0,  # Start with a reasonable value
        kd=0,  # Start with a reasonable value
        dt=0.5,
        integral_limit=5,
        output_limit=100,  # Limit the output to prevent excessive motor commands
        derivative_filter=0.1
    )


    # Flags
    self.stationary = True

    # Setpoints
    self.setpoint_pitch = 0
    self.setpoint_pos = sim.getObjectPosition(self.body, -1)

    # Errors
    self.pos_error = 0
    self.pitch_error = 0
    
    # Variable for keyboard control (rotation only)
    self.manual_rotate = 0
    self.manual_rotate_speed = 5  # Adjust this to control rotation speed

def sysCall_actuation():
    pos_gain = 0
    pitch_gain = self.pid_pitch.compute(self.pitch_error)

    if self.stationary:
        pos_gain = self.pid_pos.compute(self.pos_error)

    vel = pitch_gain - pos_gain
    vel = self.pid_motor.compute(vel)
    try:
        sign = pos_gain / abs(pos_gain)
        vel *= sign
    except ZeroDivisionError:
        pass


    left_motor_vel = vel * 10 + self.manual_rotate * self.manual_rotate_speed
    right_motor_vel = vel * 10 - self.manual_rotate * self.manual_rotate_speed

    sim.setJointTargetVelocity(self.left_motor, left_motor_vel)
    sim.setJointTargetVelocity(self.right_motor, right_motor_vel)



def sysCall_sensing():
    eulerAngles = sim.getObjectOrientation(self.body)
    roll, yaw, pitch = sim.alphaBetaGammaToYawPitchRoll(eulerAngles[0], eulerAngles[1], eulerAngles[2])

    pitch_ang = eulerAngles[2]
    deg = pitch_ang * (180 / math.pi)

    self.pitch_error = self.setpoint_pitch - pitch

    body_pos = sim.getObjectPosition(self.body, -1)
    cone_pos = sim.getObjectPosition(self.cone, -1)
    self.pos_error = calculate_distance_with_sign(body_pos, cone_pos, self.setpoint_pos)
    print(self.pos_error)

    message, data, data2 = sim.getSimulatorMessage()
    vel_offset = 0.01
    if message == sim.message_keypress:
        if data[0] == 2007:  # forward up arrow
            self.stationary = False
            self.setpoint_pitch = vel_offset
        elif data[0] == 2008:  # backward down arrow
            self.stationary = False
            self.setpoint_pitch = -vel_offset
        elif (data[0] == 2009):  # left arrow key (rotate left)
            self.manual_rotate = 1  # Positive for left rotation
        elif (data[0] == 2010):  # right arrow key (rotate right)
            self.manual_rotate = -1  # Negative for right rotation
        elif data[0] == 113:  # 'q' key
            self.stationary = True
            self.manual_rotate = 0

            self.setpoint_pitch = 0
            self.setpoint_pos = sim.getObjectPosition(self.body, -1)

            self.pos_error = 0
            self.pitch_error = self.setpoint_pitch - pitch
    else:
        self.manual_rotate = 0


def sysCall_cleanup():
    pass

