import sys
import pylab
import environment
import numpy as np
import random
import test_engine
import plotter
from collections import deque
from keras.layers import Dense
from keras.optimizers import Adam
from keras.models import Sequential
from keras import backend as K

EPISODES = 10000


class A2CAgent:

    def __init__(self, state_size, action_size):
        self.render = False
        self.load_model = False
        # 상태와 행동의 크기 정의
        self.state_size = state_size
        self.action_size = action_size
        # DQN hyperparameter 여기서 epsilon은 state의 stdeva의 크기로 사용될 것.
        self.epsilon = 1.0
        self.epsilon_decay = 0.999
        self.epsilon_min = 0.000001

        # 액터-크리틱 하이퍼파라미터
        self.discount_factor = 0.99
        self.actor_lr = 0.001
        # critic를 Q함수로 한다. 가즈아.
        self.critic_lr = 0.005

        self.batch_size = 100
        self.train_start = 1000  ########앙 기모딱

        # 리플레이 메모리, 최대 크기 2000
        self.memory = deque(maxlen=2000)

        # 정책신경망과 가치신경망 생성
        self.actor = self.build_actor()
        self.critic = self.build_critic()
        self.actor.compile(loss=self.custom_loss_wrapper(self.actor.input), optimizer=Adam(lr=self.critic_lr))
        self.target_critic = self.build_critic()
        # self.actor_updater = self.actor_optimizer()
        # self.critic_updater = self.critic_optimizer()

        # 타깃 모델 초기화
        self.update_target_model()

        if self.load_model:
            self.actor.load_weights("./save_model/robot_actor_trained.h5")
            self.critic.load_weights("./save_model/robot_critic_trained.h5")
            self.target_critic.load_weights("./save_model/robot_target_critic_trained.h5")

    # critic: 상태와 행동을 받아서 Q함수를 계산
    def build_critic(self):
        critic = Sequential()
        critic.add(Dense(100, input_dim=self.state_size + self.action_size, activation='relu',
                         kernel_initializer='he_uniform'))
        critic.add(Dense(100, activation='relu',
                         kernel_initializer='he_uniform'))
        critic.add(Dense(1, activation='linear',
                         kernel_initializer='he_uniform'))
        critic.summary()
        critic.compile(loss='mse', optimizer=Adam(lr=self.critic_lr))
        return critic

    # actor: 상태를 받아 각 행동의 확률을 계산
    def build_actor(self):
        actor = Sequential()
        actor.add(Dense(50, input_dim=self.state_size, activation='relu',
                        kernel_initializer='he_uniform'))
        actor.add(Dense(50, activation='relu',
                        kernel_initializer='he_uniform'))
        actor.add(Dense(self.action_size, activation='tanh',
                        kernel_initializer='he_uniform'))
        actor.summary()
        return actor

    # 타깃 모델을 모델의 가중치로 업데이트
    def update_target_model(self):
        self.target_critic.set_weights(self.critic.get_weights())

    # 정책신경망의 출력을 받아 확률적으로 행동을 선택
    def get_action(self, state):
        policy = self.actor.predict(state, batch_size=1).flatten()
        if np.random.rand() <= self.epsilon:
            sigma = np.sqrt(np.mean(np.square(policy)))
            policy += 2. * self.epsilon * np.random.normal(0, sigma, [self.action_size])
        return policy

    def append_sample(self, state, action, reward, next_state, done):
        self.memory.append((state, action, reward, next_state, done))

    def custom_loss_wrapper(self, input_tensor):
        def custom_loss(y_true, y_pred):
            return -K.mean(self.critic(K.concatenate([input_tensor, self.actor(input_tensor)], axis=1)))

        return custom_loss  ##왜 되는거지??

    # 리플레이 메모리에서 무작위로 추출한 배치로 모델 학습
    def train_model(self):
        if self.epsilon > self.epsilon_min:
            self.epsilon *= self.epsilon_decay
        # 메모리에서 배치 크기만큼 무작위로 샘플 추출
        mini_batch = random.sample(self.memory, self.batch_size)

        states = np.zeros((self.batch_size, self.state_size))
        next_states = np.zeros((self.batch_size, self.state_size))
        sas = np.zeros((self.batch_size, self.state_size + self.action_size))
        actions, rewards, dones = [], [], []

        for i in range(self.batch_size):
            states[i] = mini_batch[i][0].flatten()
            actions.append(mini_batch[i][1].flatten())
            rewards.append(mini_batch[i][2])
            next_states[i] = mini_batch[i][3].flatten()
            dones.append(mini_batch[i][4])
            sas[i] = (np.concatenate([mini_batch[i][0].flatten(), mini_batch[i][1].flatten()], axis=0))
            #  주의: sas부분을 고쳤는데 원본대조 바람

        # 현재 상태에 대한 모델의 큐함수
        # 다음 상태에 대한 타깃 모델의 큐함수
        target = self.critic.predict(sas)
        target_sas = np.concatenate([next_states, self.actor.predict(next_states)], axis=1)
        target_val = self.target_critic.predict(target_sas)  # target_critic으로 바꿔야된다
        # 벨만 최적 방정식을 이용한 업데이트 타깃
        for i in range(self.batch_size):
            if (dones[i] == 1):
                target[i] = rewards[i]
            elif(dones[i] == 2):
                target[i] = rewards[i]
            else:
                target[i] = rewards[i] + self.discount_factor * target_val[i]
        self.critic.fit(sas, target, batch_size=self.batch_size, epochs=1, verbose=0)
        for i in range(10):
            self.actor.fit(x=states, y=np.zeros((self.batch_size, self.action_size)), batch_size=self.batch_size,
                           epochs=1, verbose=0)


if __name__ == "__main__":
    # 환경과 에이전트의 생성
    env = environment.Env()
    state_size = env.state_size
    action_size = env.action_size

    agent = A2CAgent(state_size, action_size)

    global_step = 0
    scores, episodes = [], []

    for e in range(EPISODES):
        done = 0
        score = 0
        # env 초기화
        env.reset()
        env.get_state()
        state = env.state
        state = np.reshape(state, [1, state_size])
        life = 0
        if( e%100 ==0):
            test_engine.test(agent.actor.predict)
        while (done == 0):
            life +=1
            action = agent.get_action(state)
            next_state, reward, done = env.step(action)
            next_state = np.reshape(next_state, [1, state_size])
            # 에피소드가 중간에 끝나면 -100 보상 --> 없앰

            # 리플레이 메모리에 샘플 <s, a, r, s'> 저장
            agent.append_sample(state, action, reward, next_state, done)
            # 매 타임스텝마다 학습
            if len(agent.memory) >= agent.train_start:
                agent.train_model()

            score += reward
            state = next_state
            #  print('rs = ', env.R.body.rs)
            #  print('Rnow = ', env.R.body.Rnow)
            if (done != 0):
                # 각 에피소드마다 타깃 모델을 모델의 가중치로 업데이트
                agent.update_target_model()
                # 에피소드마다 학습 결과 출력
                scores.append(score)
                episodes.append(e)
                pylab.plot(episodes, scores, 'b')
                pylab.savefig("./save_graph/robot_a2c.png")
                print("episode:", e, "  score:", score, 'life: ', life)

                # 이전 10개 에피소드의 점수 평균이 490보다 크면 학습 중단
                if np.mean(scores[-min(10, len(scores)):]) > 10000:
                    print('앙 끝났띠')
                    agent.actor.save_weights("./save_model/robot_actor.h5")
                    agent.critic.save_weights(
                        "./save_model/robot_critic.h5")
                    sys.exit()
            #주의:  에피 10번 끝날때마다 test_py 이용해서 돌리고 결과확인하는 코드 넣기 바람. 즉, 여기서 얻은 weight로 시각화