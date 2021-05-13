#!/usr/bin/env python
import numpy as np
import sys, os, warnings
import time
import matplotlib.pyplot as plt
from arena_environment import Environment
import utilities as util
import dill as pickle
import json
import rospy
# from matplotlib import interactive

class eps_bandit:
    """
    epsilon-greedy & epsilon-decay k-bandit problem

    Inputs
    =====================================================
    env: environment class. Must inlcude an 'update' method to
    change parameters and a 'act' method to collect a reward
    eps: probability of random action 0 < eps < 1 (float).
    If 'eps' is NaN than decay bandit strategy will be used
    iters: number of steps (int)
    param_map: parameters

    """

    def __init__(self, env, iters, param_map, eps=None, action_time=None, rest_time=None):
        # Get ROS parameters
        self.name = 'learning'
        self.save_format = '.pickle'
        self.save_fields = ['action_data','time_data','action_time','rest_time',
                            'param_map','n_param','k','iters','n','k_n','reward_iter','k_iter',
                            'mean_reward','reward','k_reward','k_sort','k_reward_sort',
                            'param_sort','decay', 'params']
        self.env_save_fields = ['params','wing_gain','head_gain']
        self.params = rospy.get_param(self.name, {})

        # Get learning hyperparameters
        if eps is None:
            eps = self.params['epsilon']

        if action_time is None:
            action_time = self.params['action_time']

        if rest_time is None:
            rest_time = self.params['rest_time']

        # Intitialize the environment, bandit arms, parameter map, and learning vairables
        self.env = env # enviromnent
        self.eps = eps # search probability
        self.action_time = action_time # length of one action [s]
        self.rest_time = rest_time  # length of rest period [s]
        self.param_map = param_map # parameter corresponding to bandit arms
        self.k = self.param_map.shape[0] # # of arms

        # # of paramters to learn
        if self.param_map.ndim == 1:
            self.n_param = 1
        else:
            self.n_param = self.param_map.shape[1]  # # of parameters

        self.iters = iters # # of iterations
        self.n = 0 # step count
        self.k_n = np.zeros(self.k) # step count for each arm
        self.reward_iter = np.zeros(iters) # reward at each iteration
        self.k_iter = np.zeros(iters) # arm choice at each iteration
        self.mean_reward = 0 # mean reward
        self.reward = np.zeros(iters) # total reward

        self.k_reward = np.zeros((self.k,1)) # mean reward for each arm
        self.k_sort = np.zeros((self.k,1)) # actions sorted by reward
        self.k_reward_sort = np.zeros((self.k,1))  # sorted rewards for each action
        self.param_sort = np.zeros((self.k,self.n_param)) # parameters sorted by reward

        self.action_data = []
        self.time_data = []

        # If 'eps' is set to NaN than use an eps-decay strategy
        if np.isnan(self.eps):
            self.decay = True
        else:
            self.decay = False

        # interactive(True)
        self.fig, self.axs = plt.subplots(2,1)
        self.fig.show()

    def pull(self, test=False):
        # Update parameters of environment, run simulation, and collect reward
        if self.decay: # eps-decay strategy
            p = np.random.rand()  # generate random probability between 0-1
            if p < 1 / (1 + self.n / self.k):
                a = np.random.choice(self.k) # randomly select an action
            else: # take greedy action
                a = np.argmax(self.k_reward)

        else: # eps-greedy strategy
            p = np.random.rand() # generate random probability between 0-1
            if self.eps == 0 or self.n == 0: # 1st iteration or greedy (eps = 0)
                a = np.random.choice(self.k)
            elif p < self.eps: # probability below eps
                a = np.random.choice(self.k) # randomly select an action
            else: # take greedy action
                a = np.argmax(self.k_reward) # choose arm with maximum expected reward

        # Get reward
        params = self.param_map[a,:] # get parameters correponding to chosen arm
        if test:
            self.env.setFunc(params) # update environment
        else:
            self.env.setGains(*params) # update environment

        reward, panel_data, time_vector = self.env.act(self.action_time, self.rest_time)  # interact with environment

        # Store action data
        self.action_data = panel_data
        self.time_data = time_vector

        # Store current arm choice & reward
        self.k_iter[self.n] = a  # arm choice at current iteration
        self.reward_iter[self.n] = reward # reward at current iteration

        # Update counts
        self.n += 1
        self.k_n[a] += 1 # update # of times this arm was chosen

        # Update mean reward
        self.mean_reward = self.mean_reward + (reward - self.mean_reward) / self.n

        # Update results for a_k
        self.k_reward[a] = self.k_reward[a] + (reward - self.k_reward[a]) / self.k_n[a]

    def run(self, filepath=None, test=False):
        # Run through iterations and extract rewards
        for i in range(self.iters):
            self.pull(test)
            self.reward[i] = self.mean_reward
            print("EP: " + str(i) + " Score: " + str(self.reward[i]) + " ")
            sys.stdout.flush()
            # print("EP: " + str(i) + " Score: " + str(self.reward[i]) + " ", end="\r", flush=False)

            self.k_sort = np.squeeze(np.flipud(np.argsort(self.k_reward, axis=0)))
            self.k_reward_sort = self.k_reward[self.k_sort, :]
            self.param_sort = self.param_map[self.k_sort, :]

            self.plot_reward()
            self.save(filepath=filepath)

    def save(self, filepath=None):
        # Save data
        if filepath is not None:
            currentObj = self.__dict__ # get current state of object as a dict
            fullpath = filepath + self.save_format # add file extension

            if self.save_format == '.pickle': # save as .pickle
                with open(fullpath, 'wb') as f:
                    pickle.dump(self, f, pickle.HIGHEST_PROTOCOL) # dump current iteration to pickle

            elif self.save_format == '.json':  # save as .json
                with open(fullpath, 'wb') as f:
                    json.dump(currentObj, f) # dump current iteration to json

            else:
                warnings.warn('NOT SAVING: save file format must be .pickle or .json')

            # Save current iteration data in folder as .json
            iter_root = os.path.join(filepath, '')
            if not os.path.exists(iter_root):
                os.makedirs(iter_root)

            itername = 'iter_' + str(self.n - 1) + '.json'
            iterpath = os.path.join(filepath, itername)
            save_dict = util.getDictFields(dict=currentObj, fields=self.save_fields)
            env_dict = util.getDictFields(dict=self.env.__dict__, fields=self.env_save_fields)
            save_dict['env'] = env_dict
            with open(iterpath, 'wb') as f:
                json.dump(save_dict, f)  # dump current action data to json

    def plot_reward(self):
        self.axs[0].clear()
        self.axs[1].clear()
        self.axs[0].set_xlabel('time (s)')
        self.axs[0].set_ylabel('bar (deg)')
        self.axs[1].set_xlabel('iteration')
        self.axs[1].set_ylabel('total reward')
        self.axs[0].set_title('Current reward = ' + str(self.reward_iter[self.n-1]))
        self.axs[0].plot(self.time_data, self.action_data, 'r-')
        self.axs[1].plot(self.reward[0:self.n-1], '-bo')
        plt.pause(0.05)
        time.sleep(0.2)

    def reset(self):
        # Resets results while keeping settings
        self.n = 0
        self.k_n = np.zeros(self.k)
        self.mean_reward = 0
        self.reward = np.zeros(self.iters)
        self.k_reward = np.zeros(self.k)


if __name__ == '__main__':
    env_init = Environment(init_wing_gain=1, init_head_gain=0, init_arena_gain=0)