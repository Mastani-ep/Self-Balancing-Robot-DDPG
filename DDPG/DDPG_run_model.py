import numpy as np
import serial

from CustomRobotEnv import CustomRobotEnv
from utils import build_agent

env = CustomRobotEnv()

agent = build_agent(env)
agent.load_weights('ddpg_agent.h5')

# connectivity setup
uart_port = 'COM5'  # in case of UART connectivity
uart_speed = 115200  # serial port speed
ser = serial.Serial(
    port=uart_port,
    baudrate=uart_speed,
    parity=serial.PARITY_NONE,
    stopbits=serial.STOPBITS_ONE,
    bytesize=serial.EIGHTBITS,
    timeout=0.01
)

msg = b''

while msg == b'':
    msg = ser.readline()

observation = msg.decode('utf-8').split()

observation = np.array([float(observation[0])])

while True:
    try:
        action = agent.forward(observation)

        ser.write((str((action * 255)[0]) + '\n').encode())
        print("sent: " + str((action * 255)[0]))
        msg = ser.readline()

        observation = msg.decode('utf-8').split()
        observation = np.array([float(observation[0])])
        print(observation[0])
    except:
        pass
