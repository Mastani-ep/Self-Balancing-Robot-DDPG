from CustomRobotEnv import CustomRobotEnv
from utils import build_agent


def train_model():
    env = CustomRobotEnv()

    agent = build_agent(env)

    # agent.load_weights('ddpg_agent.h5')

    agent.fit(env, nb_steps=200000, visualize=False, verbose=1, nb_max_episode_steps=5000)

    agent.save_weights('ddpg_agent.h5', overwrite=True)

    agent.test(env, nb_episodes=5, visualize=False, nb_max_episode_steps=5000)


train_model()
