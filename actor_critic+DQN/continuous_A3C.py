from keras.layers import Dense, Flatten, Input
from keras.layers.convolutional import Conv2D
from keras.optimizers import RMSprop
from keras import backend as K
from keras.models import Model
from keras.optimizers import Adam
from keras.models import Sequential
from keras.layers import Dense
import tensorflow as tf
import numpy as np
import threading
import random
import time
import environment
K.clear_session()
# 멀티쓰레딩을 위한 글로벌 변수
global episode
episode = 0
EPISODES = 8000000
# 환경 생성
env_name = "BreakoutDeterministic-v4"

# 브레이크아웃에서의 A3CAgent 클래스(글로벌신경망)
class A3CAgent:
    def __init__(self):
        # 상태크기와 행동크기를 갖고옴
        self.state_size = environment.state_size
        self.action_size = environment.action_size
        # A3C 하이퍼파라미터
        self.discount_factor = 0.999
        self.no_op_steps = 30
        self.actor_lr = 2.5e-4
        self.critic_lr = 2.5e-4
        # 쓰레드의 갯수
        self.threads = 2

        # 정책신경망과 가치신경망을 생성
        self.actor, self.critic = self.build_model()
        # 정책신경망과 가치신경망을 업데이트하는 함수 생성
        #self.optimizer = [self.actor_optimizer(), self.critic_optimizer()]

        # 텐서보드 설정
        self.sess = tf.InteractiveSession()
        K.set_session(self.sess)
        self.sess.run(tf.global_variables_initializer())

        self.summary_placeholders, self.update_ops, self.summary_op = \
            self.setup_summary()
        self.summary_writer = \
            tf.summary.FileWriter('summary/breakout_a3c', self.sess.graph)

    # 쓰레드를 만들어 학습을 하는 함수
    def train(self):
        # 쓰레드 수만큼 Agent 클래스 생성
        agents = [Agent(self.action_size, self.state_size,
                        [self.actor, self.critic], self.sess, self.discount_factor,
                        [self.summary_op, self.summary_placeholders,
                         self.update_ops, self.summary_writer])
                  for _ in range(self.threads)]

        # 각 쓰레드 시작
        for agent in agents:
            time.sleep(1)
            agent.start()

        # 10분(600초)에 한번씩 모델을 저장
        while True:
            time.sleep(60 * 10)
            self.save_model("./save_model/breakout_a3c")

    # 정책신경망과 가치신경망을 생성
    def build_model(self):
        '''
        input = Input(shape=self.state_size)
        conv = Conv2D(16, (8, 8), strides=(4, 4), activation='relu')(input)
        conv = Conv2D(32, (4, 4), strides=(2, 2), activation='relu')(conv)
        conv = Flatten()(conv)
        fc = Dense(256, activation='relu')(conv)

        policy = Dense(self.action_size, activation='softmax')(fc)
        value = Dense(1, activation='linear')(fc)

        actor = Model(inputs=input, outputs=policy)
        critic = Model(inputs=input, outputs=value)
        '''
        critic = Sequential()
        critic.add(Dense(100, input_dim=self.state_size+self.action_size, activation='relu',
                        kernel_initializer='he_uniform'))
        critic.add(Dense(100, activation='relu',
                        kernel_initializer='he_uniform'))
        critic.add(Dense(1, activation='relu',
                        kernel_initializer='he_uniform'))
        critic.compile(loss='mse', optimizer=Adam(lr=self.critic_lr))
        critic._make_predict_function()

        actor = Sequential()
        actor.add(Dense(50, input_dim=self.state_size, activation='relu',
                        kernel_initializer='he_uniform'))
        actor.add(Dense(50, activation='relu',
                        kernel_initializer='he_uniform'))
        actor.add(Dense(self.action_size, activation='tanh',
                        kernel_initializer='he_uniform'))
        actor._make_predict_function()
        # 가치와 정책을 예측하는 함수를 만들어냄

        actor.summary()
        critic.summary()

        return actor, critic

    def custom_loss_wrapper(self, input_tensor):
        def custom_loss(y_true, y_pred):
            return -K.mean(self.critic(K.concatenate([input_tensor, self.actor(input_tensor)], axis=1)))
        return custom_loss
    # 정책신경망을 업데이트하는 함수

    def setup_summary(self):
        episode_total_reward = tf.Variable(0.)
        episode_duration = tf.Variable(0.)

        tf.summary.scalar('Total Reward/Episode', episode_total_reward)
        tf.summary.scalar('Duration/Episode', episode_duration)

        summary_vars = [episode_total_reward,
                        episode_duration]

        summary_placeholders = [tf.placeholder(tf.float32)
                                for _ in range(len(summary_vars))]
        update_ops = [summary_vars[i].assign(summary_placeholders[i])
                      for i in range(len(summary_vars))]
        summary_op = tf.summary.merge_all()
        return summary_placeholders, update_ops, summary_op

    def load_model(self, name):
        self.actor.load_weights(name + "_actor.h5")
        self.critic.load_weights(name + "_critic.h5")

    def save_model(self, name):
        self.actor.save_weights(name + "_actor.h5")
        self.critic.save_weights(name + "_critic.h5")

'''
    def actor_optimizer(self):
        states = K.placeholder(shape=[None, self.state_size])
        policy = self.actor.output
        # 정책 크로스 엔트로피 오류함수

        action_prob = K.sum(action * policy, axis=1)
        cross_entropy = K.log(action_prob + 1e-10) * advantages
        cross_entropy = -K.sum(cross_entropy)

        # 탐색을 지속적으로 하기 위한 엔트로피 오류
        entropy = K.sum(policy * K.log(policy + 1e-10), axis=1)
        entropy = K.sum(entropy)

        # 두 오류함수를 더해 최종 오류함수를 만듬
        loss = cross_entropy + 0.01 * entropy
        #  loss = self.custom_loss_wrapper(states)

        loss = -K.mean(self.critic(K.concatenate([states, policy], axis=1)))
        optimizer = RMSprop(lr=self.actor_lr, rho=0.99, epsilon=0.01)
        updates = optimizer.get_updates(self.actor.trainable_weights, [], loss)
        train = K.function([self.actor.input, states],
                           [loss], updates=updates)
        return train
        
    # 가치신경망을 업데이트하는 함수
    def critic_optimizer(self):
        prediction = K.placeholder(shape=(None,))
        value = self.critic.get_output_at(-1)
        print(value)
        # [반환값 - 가치]의 제곱을 오류함수로 함
        loss = K.mean(K.square(prediction - value))

        optimizer = RMSprop(lr=self.critic_lr, rho=0.99, epsilon=0.01)
        updates = optimizer.get_updates(self.critic.trainable_weights, [], loss)
        train = K.function([self.critic.get_input_at(0), prediction],
                           [loss], updates=updates)
        return train
'''



# 액터러너 클래스(쓰레드)
class Agent(threading.Thread):
    def __init__(self, action_size, state_size, model, sess,
                discount_factor, summary_ops):
        threading.Thread.__init__(self)
        self.batch_size = 10
        # A3CAgent 클래스에서 상속
        self.action_size = action_size
        self.state_size = state_size
        self.actor, self.critic = model
        self.sess = sess
        self.epsilon = 1.0
        self.epsilon_decay = 0.999
        self.epsilon_min = 0.000001
       # self.optimizer = optimizer
        self.discount_factor = discount_factor
        [self.summary_op, self.summary_placeholders,
         self.update_ops, self.summary_writer] = summary_ops

        # 지정된 타임스텝동안 샘플을 저장할 리스트
        self.states, self.actions, self.rewards = [], [], []

        # 로컬 모델 생성
        self.local_actor, self.local_critic = self.build_local_model()

        self.avg_p_max = 0
        self.avg_loss = 0

        # 모델 업데이트 주기
        self.t_max = 20
        self.t = 0


    def run(self):
        global episode
        env = environment.Env()

        step = 0

        while episode < EPISODES:
            done = False

            score = 0, 5
            state = env.reset()

            #  history = np.stack((state, state, state, state), axis=2)
            #  history = np.reshape([history], (1, 84, 84, 4))

            while (done==0):
                step += 1
                self.t += 1
                action = self.get_action(state)

                # 선택한 행동으로 한 스텝을 실행
                next_state, reward, done = env.step(action)


                # 정책의 최대값

                self.avg_p_max += self.actor.predict(
                    np.reshape(next_state, [-1, self.state_size]))

                score += reward

                # 샘플을 저장
                self.append_sample(state, action, reward)

                # 에피소드가 끝나거나 최대 타임스텝 수에 도달하면 학습을 진행
                if self.t >= self.t_max or not (done == 0):
                    self.train_model(done)
                    self.update_local_model()
                    self.t = 0

                if not (done == 0):
                    # 각 에피소드 당 학습 정보를 기록
                    episode += 1
                    print("episode:", episode, "  score:", score, "  step:",
                          step)

                    stats = [score, self.avg_p_max / float(step),
                             step]
                    for i in range(len(stats)):
                        self.sess.run(self.update_ops[i], feed_dict={
                            self.summary_placeholders[i]: float(stats[i])
                        })
                    summary_str = self.sess.run(self.summary_op)
                    self.summary_writer.add_summary(summary_str, episode + 1)
                    self.avg_p_max = 0
                    self.avg_loss = 0
                    step = 0

    # prediction 계산
    def prediction(self, rewards, done):
        prediction = np.zeros_like(rewards)
        running_add = 0

        if not(done == 0):
            running_add = self.critic.predict(np.float32(
                self.states[-1]))[0]

        for t in reversed(range(0, len(rewards))):
            running_add = running_add * self.discount_factor + rewards[t]
            prediction[t] = running_add
        return prediction

    # 정책신경망과 가치신경망을 업데이트
    def train_model(self, done):##################action이 필요하다.
        if self.epsilon > self.epsilon_min:
            self.epsilon *= self.epsilon_decay
        prediction = self.prediction(self.rewards, done)

        states = np.zeros((len(self.states), environment.state_size))
        sas = np.zeros((len(self.states), environment.state_size+self.action_size))
        for i in range(len(self.states)):
            states[i] = self.states[i].flatten()
            sas[i] = np.concatenate([self.states[i], self.actions[i]], axis=0).flatten()
        '''values = self.critic.predict(states)
        values = np.reshape(values, len(values))

        advantages = prediction-values'''
        self.local_critic.fit(sas, prediction, batch_size=self.batch_size, epochs=1, verbose=0)
        for i in range(10):
            self.local_actor.fit(x=states, y=np.zeros((self.batch_size, self.action_size)), batch_size=self.batch_size,
                           epochs=1, verbose=0)
        '''self.optimizer[0]([states])
        self.optimizer[1]([sas, prediction])'''
        self.states, self.actions, self.rewards = [], [], []

    # 로컬신경망을 생성하는 함수
    def build_local_model(self):

        local_critic = Sequential()
        local_critic.add(Dense(100, input_dim=self.state_size + self.action_size, activation='relu',
                         kernel_initializer='he_uniform'))
        local_critic.add(Dense(100, activation='relu',
                         kernel_initializer='he_uniform'))
        local_critic.add(Dense(1, activation='linear',
                         kernel_initializer='he_uniform'))
        local_critic.compile(loss='mse', optimizer=Adam(lr=0.005))
        local_critic._make_predict_function()

        local_actor = Sequential()
        local_actor.add(Dense(50, input_dim=self.state_size, activation='relu',
                        kernel_initializer='he_uniform'))
        local_actor.add(Dense(50, activation='relu',
                        kernel_initializer='he_uniform'))
        local_actor.add(Dense(self.action_size, activation='tanh',
                        kernel_initializer='he_uniform'))
        local_actor._make_predict_function()


        # 가치와 정책을 예측하는 함수를 만들어냄

        local_actor.summary()
        local_critic.summary()

        return local_actor, local_critic

    # 로컬신경망을 글로벌신경망으로 업데이트
    def update_local_model(self):
        self.local_actor.set_weights(self.actor.get_weights())
        self.local_critic.set_weights(self.critic.get_weights())

    # 정책신경망의 출력을 받아서 확률적으로 행동을 선택
    def get_action(self, state):
        print(state)
        state = np.reshape(state, (-1, 42))
        print(state)
        policy = self.local_actor.predict(np.reshape(state, (-1, self.state_size)), batch_size=1).flatten()
        sigma = np.sqrt(np.mean(np.square(policy)))
        policy += 2. * self.epsilon * np.random.normal(0, sigma, [self.action_size])
        return policy
    '''def get_action(self, history):
        history = np.float32(history / 255.)
        policy = self.local_actor.predict(history)[0]
        action_index = np.random.choice(self.action_size, 1, p=policy)[0]
        return action_index, policy'''

    # 샘플을 저장
    def append_sample(self, state, action, reward):
        self.states.append(state)
        self.actions.append(action)
        self.rewards.append(reward)

if __name__ == "__main__":
    global_agent = A3CAgent()
    global_agent.train()
