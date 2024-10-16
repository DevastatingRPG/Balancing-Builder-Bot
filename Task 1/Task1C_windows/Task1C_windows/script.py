import math
import time
import matplotlib.pyplot as plt

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

def sysCall_init():
    sim = require('sim')

    self.body = sim.getObject('/body')
    self.left_motor = sim.getObject('/left_joint')
    self.right_motor = sim.getObject('/right_joint')

    # Initialize PID controller for pitch (balancing)
    self.pid_pitch = PIDController(
        kp=1,  # Start with a reasonable value
        ki=2,  # Start with a reasonable value
        kd=0,  # Start with a reasonable value
        dt=0.5,
        integral_limit=5,
        output_limit=20,  # Limit the output to prevent excessive motor commands
        derivative_filter=0.1
    )

    # Initialize PID controller for position
    self.pid_pos = PIDController(
        kp=0.2,  # Start with a reasonable value
        ki=0,  # Start with a reasonable value
        kd=0.3,  # Start with a reasonable value
        dt=0.5,
        integral_limit=5,
        output_limit=20,  # Limit the output to prevent excessive motor commands
        derivative_filter=0.1
    )

    # Current state
    self.pitch = 0
    self.pos = sim.getObjectPosition(self.body)[0:2]

    # Variables for keyboard control
    self.manual_rotate = 0
    self.manual_rotate_speed = 30  # Adjust this to control rotation speed
    self.setpoint_pitch = 0.001  # Setpoint for forward/backward movement

    # Time step
    self.dt = sim.getSimulationTimeStep()

    # Data streams for pitch angle
    self.pitch_data = []
    self.time_data = []

    # Physical properties
    self.mass_robot = 0.05  # kg
    self.mass_wheel = 0.018  # kg
    self.wheel_radius = 0.1  # m
    self.max_torque = 2.5  # Nm

    # Calculate total mass
    self.total_mass = self.mass_robot + 2 * self.mass_wheel

    # Calculate maximum force
    self.max_force = self.max_torque / self.wheel_radius

    # Calculate maximum acceleration
    self.max_acceleration = self.max_force / self.total_mass

    # Estimate maximum jerk (assuming response time of 0.1 seconds)
    self.max_jerk = self.max_acceleration / 0.1

    # Should Bot be stationary or not
    self.stationary = True
    self.setpoint_pos = sim.getObjectPosition(self.body)[0:2]

def sysCall_actuation():
    pos_gain = 0

    if self.stationary:
        # Position control for stationary
        pos_error = get_dist(self.pos, self.setpoint_pos)
        pos_gain = self.pid_pos.compute(pos_error)
        self.setpoint_pitch = pos_gain


    # Pitch control for balancing
    pitch_error = self.setpoint_pitch - self.pitch
    pitch_gain = self.pid_pitch.compute(pitch_error)

    vel = pitch_gain

    left_motor_vel = vel * 5 + self.manual_rotate * self.manual_rotate_speed
    right_motor_vel = vel * 5 - self.manual_rotate * self.manual_rotate_speed

    sim.setJointTargetVelocity(self.left_motor, left_motor_vel, [self.max_acceleration, self.max_jerk])
    sim.setJointTargetVelocity(self.right_motor, right_motor_vel, [self.max_acceleration, self.max_jerk])

    # Update the graph with the current pitch angle
    self.pitch_data.append(self.pitch)
    self.time_data.append(sim.getSimulationTime())

def sysCall_sensing():
    self.pitch = get_roll(self.body)
    if self.stationary:
        self.pos = sim.getObjectPosition(self.body)[0:2]
    vel = 0.01
    message, data, data2 = sim.getSimulatorMessage()
    if message == sim.message_keypress:
        if data[0] == 2007:  # forward up arrow
            self.stationary = False
            self.setpoint_pitch = vel
            self.manual_rotate = 0
        elif data[0] == 2008:  # backward down arrow
            self.stationary = False
            self.setpoint_pitch = -vel
            self.manual_rotate = 0
        elif data[0] == 2009:  # left arrow key
            self.manual_rotate = 1
        elif data[0] == 2010:  # right arrow key
            self.manual_rotate = -1
        elif data[0] == 113:
            self.stationary = True
            self.setpoint_pos = sim.getObjectPosition(self.body)[0:2]
            self.setpoint_pitch = 0
    else:
        # pass
        self.manual_rotate = 0

        # self.setpoint_pitch = 0  # Stop forward/backward movement when no key is pressed

def get_dist(current, desired):
    dx = desired[0] - current[0]
    dy = desired[1] - current[1]
    distance = math.sqrt(dx**2 + dy**2)
    
    # Determine the sign based on the direction
    sign = 1 if dx >= 0 else -1
    return sign * distance

def get_roll(body):
    eulerAngles = sim.getObjectOrientation(body)
    yaw, pitch, roll = sim.alphaBetaGammaToYawPitchRoll(eulerAngles[0], eulerAngles[1], eulerAngles[2])
    return roll

def sysCall_cleanup():
    # Cleanup function executed when the simulation ends
    plt.figure()
    plt.plot(self.time_data, self.pitch_data, label='Pitch Angle (degrees)')
    plt.xlabel('Time Step')
    plt.ylabel('Pitch Angle (degrees)')
    plt.title('Pitch Angle Over Time')
    plt.legend()
    plt.savefig(r'C:\Users\devas\OneDrive\Desktop\pitch_angle_plot.png')
    plt.close()

    # Find the oscillation period (Pu)
    if len(self.pitch_data) > 1:
        peaks = []
        for i in range(1, len(self.pitch_data) - 1):
            if self.pitch_data[i-1] < self.pitch_data[i] > self.pitch_data[i+1]:
                peaks.append(self.time_data[i])
        if len(peaks) > 1:
            Pu = peaks[1] - peaks[0]
            print(f"Ultimate Period (Pu): {Pu} seconds")
        else:
            print("Not enough peaks to determine Pu")
    else:
        print("Not enough data to determine Pu")