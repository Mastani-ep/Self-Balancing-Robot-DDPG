from keras.layers import Activation, Dense, Flatten, Input, Concatenate
from keras.models import Sequential, Model
from keras.optimizers import Adam
from rl.agents import DDPGAgent
from rl.memory import SequentialMemory
from rl.random import OrnsteinUhlenbeckProcess


def build_agent(env):
    def build_actor():
        actor = Sequential()
        actor.add(Flatten(input_shape=(1,) + env.observation_space.shape))
        actor.add(Dense(64))
        actor.add(Activation('relu'))
        actor.add(Dense(32))
        actor.add(Activation('relu'))
        actor.add(Dense(nb_actions))
        actor.add(Activation('tanh'))
        return actor

    def build_critic():
        observation_input = Input(shape=(1,) + env.observation_space.shape, name='observation_input')
        flattened_observation = Flatten()(observation_input)

        x = Concatenate()([action_input, flattened_observation])
        x = Dense(64, activation='relu')(x)
        x = Dense(32, activation='relu')(x)
        x = Dense(1)(x)

        critic = Model(inputs=[action_input, observation_input], outputs=x)
        return critic

    nb_actions = 1

    action_input = Input(shape=(nb_actions,), name='action_input')

    memory = SequentialMemory(limit=100000, window_length=1)
    random_process = OrnsteinUhlenbeckProcess(size=nb_actions, theta=.15, mu=0., sigma=.3)

    agent = DDPGAgent(
        nb_actions=nb_actions,
        actor=build_actor(),
        critic=build_critic(),
        critic_action_input=action_input,
        memory=memory,
        random_process=random_process,
        batch_size=100
    )
    agent.compile(Adam(lr=0.001, clipnorm=1.), metrics=['mae'])
    return agent
