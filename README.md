# Self-Balancing Robot
This repository contains implementation of reinforcement learning model for balancing real life robot.\
All necessary files for training and running one of example models are inluded:
- Custom Virtual Environment based on OpenAI Gym - [CustomRobotEnv.py](DDPG/CustomRobotEnv.py)
- setup of actor and critic for DDPG algorithm - [utils.py](DDPG/utils.py)
- model training - [DDPG.py](DDPG/DDPG.py)
- running model - [DDDPG_run_model.py](DDPG/DDPG_run_model.py)
- arduino firmware for model and robot communication (option for PID controller) - [firmware.ino](DDPG/firmware/firmware.ino)
- example models ready for use - [models](DDPG/models)


### Model training
To train a model follow steps below:
1. Adjust virtual environment physics/robot simulation parameters in CustomRobotEnv.py (optional)
2. Adjust training parameters e.g. number of steps (nb_steps) in DDPG.py (optional)
3. If you would like to continue training of previous model uncomment line below in DDPG.py (optional)

````
agent.load_weights('ddpg_agent.h5')
````

Make sure that model you would like to continue training is placed in DDPG directory or adjust path to ddpg_agent file.
4. Run DDPG.py to start training. Trained model will be saved in DDPG directory.

### Model running
To run a trained model follow steps below:
1. Make sure that model is in DDPG directory or modify the path in DDPG_run_model.py
2. Flash firmware.ino to your Arduino module using Arduino IDE (make sure that manual_control in firmware is set to true)
3. When firmware is uploaded run DDPG_run_model.py (make sure that port in that file matches yours)


###  Using PID controller to control the robot
To use PID controller follow steps below:
1. Set manual_control to false in firmware.ino
2. Flash firmware.ino to your Arduino module using Arduino IDE  
  



