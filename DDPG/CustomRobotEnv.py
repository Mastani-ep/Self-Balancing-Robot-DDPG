import gym
import numpy as np


class CustomRobotEnv(gym.Env):
    def __init__(self):
        super(CustomRobotEnv, self).__init__()
        self.observation_space = gym.spaces.Box(low=np.array([-90]), high=np.array([90]), dtype=np.float32)
        self.action_space = gym.spaces.Box(low=-1, high=1, shape=(1,), dtype=np.float32)

        # Robot state variables
        self.angle = 0.0
        self.moving = False

        # Threshold for falling (in degrees)
        self.fall_threshold = 70.0

        # Mass and dimensions of components
        self.motor_width = 0.04  # 4cm in meters
        self.motor_height = 0.04  # 4cm in meters
        self.wheel_radius = 0.063 / 2  # Wheels diameter (6.3cm) in meters
        self.metal_plate_width = 0.082  # 8.2cm in meters
        self.metal_plate_height = 0.003  # 3mm in meters
        self.plastic_plate_weight = 0.045  # 45g in kg
        self.plastic_plate_width = 0.082  # 8.2cm in meters
        self.plastic_plate_height = 0.004  # 4mm in meters
        self.arduino_board_weight = 0.055  # 55g in kg
        self.arduino_board_width = 0.052  # 5.2cm in meters
        self.arduino_board_height = 0.025  # 2.5cm in meters
        self.distance_between_plates = 0.013  # 1.3cm in meters
        self.distance_between_plate_and_arduino = 0.047  # 4.7cm in meters
        self.center_of_mass_X = 0.0405  # 4cm in meters
        self.center_of_mass_Y = 0.0564  # 5.64cm in meters

        self.total_mass = 0.680
        self.center_of_mass = [self.center_of_mass_X, self.center_of_mass_Y]

        self.torque_to_acceleration_slope = 0.0004542741699916177
        self.torque_to_acceleration_intercept = 0.0
        self.carpet_friction_coefficient = 0.003

        self.gravity = 9.81

        # Calculate the moment of inertia
        self.moment_of_inertia = (1.814 * 10 ** (-4) +
                                  1.355 * 10 ** (-4) +
                                  6.929 * 10 ** (-4) +
                                  1.24 * 10 ** (-5) +
                                  5.412 * 10 ** (-5)
                                  )
        self.distance_mass_to_axis = self.center_of_mass_Y - self.wheel_radius

        # Set the initial state of the environment
        self.reset()

    def _calculate_friction(self, moving):
        if moving:
            return 12.0  # Dynamic friction threshold
        else:
            return 30.0  # Static friction threshold

    def _calculate_torque(self, torque_value):
        # Calculate actual torque based on torque value using the conversion formula
        torque = self.torque_to_acceleration_slope * torque_value + self.torque_to_acceleration_intercept
        return torque

    def step(self, action):
        motor_speeds = np.round(action * 255, decimals=0)

        friction_threshold = self._calculate_friction(self.moving)
        if np.abs(motor_speeds) < friction_threshold:
            motor_speeds = 0.0
            self.moving = False
        else:
            self.moving = True

        torque = np.sum(self._calculate_torque(motor_speeds))

        vector_direction = -1 if torque > 0 else 1
        inclination_side_offset = 1.10 if self.angle > 0 else 1

        angular_acceleration = (
                vector_direction * self.carpet_friction_coefficient * self.total_mass * self.gravity * self.wheel_radius /
                self.moment_of_inertia +
                torque * self.total_mass * self.wheel_radius * self.distance_mass_to_axis * np.cos(np.radians(self.angle)) /
                self.moment_of_inertia ** 2 +
                inclination_side_offset * self.total_mass * self.gravity * self.distance_mass_to_axis * np.sin(np.radians(self.angle)) /
                self.moment_of_inertia * 10)

        dt = 0.02  # Time step

        self.angle += (np.degrees(angular_acceleration) * dt ** 2) / 2
        self.angle = np.round(self.angle, decimals=2)

        # Check if the robot has fallen
        done = np.abs(self.angle) > self.fall_threshold or np.abs(action) > 1

        if done:
            reward = 0.0
        else:
            if np.abs(self.angle) < 1:
                reward = 1.0
            elif np.abs(self.angle) < 5:
                reward = 0.8 * (1 - (np.abs(self.angle) / 50))
            elif np.abs(self.angle) < 10:
                reward = 0.6 * (1.025 - (np.abs(self.angle) / 40))
            elif np.abs(self.angle) < 50:
                reward = 0.2
            elif np.abs(self.angle) < 70:
                reward = 0.1
            else:
                reward = 0.0

            reward -= 0.2 * np.sum((min(np.abs(motor_speeds), 255) / 255))
        reward = max(reward, 0)

        # Update observation
        observation = np.array([self.angle])

        return observation, reward, done, {}

    def reset(self):
        # Reset robot state
        self.angle = 0.0
        self.moving = False

        observation = np.array([self.angle])
        return observation
