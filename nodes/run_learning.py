
import numpy as np
import os
# import matplotlib.pyplot as plt
# import pickle
import utilities as util
from MAB import eps_bandit
from arena_environment import Environment

saveB = True
root = r"/home/jean-michel/learning_project/test_v1"
basename = 'fly_0'
basepath = os.path.join(root, basename)

init_agent = util.loadjson(root=root)

env_init = Environment(init_wing_gain=1, init_head_gain=0, init_arena_gain=1)

kwing = util.stepArray(0, 5, 50)
khead = np.asarray([0])
Kmap = util.combvec(kwing, khead)
k = Kmap.shape[0]

episodes = 1
iters = 50
rewards = np.zeros(iters)
selection = np.zeros(k)

# Run experiments
eps_all = np.zeros((iters,episodes))
selection_all = np.zeros((k,episodes))
for i in range(episodes):
    # Initialize bandit
    Agent = eps_bandit(env_init, iters, Kmap)

    # Set file to dump learning data
    if saveB:
        fpath = basepath + '_eps_' + str(i)
    else:
        fpath = None

    # Run learning process
    Agent.run(fpath)

    # Collect rewards & selections
    eps_all[:,i] = Agent.reward # collect rewards & selections
    selection_all[:,i] = Agent.k_n # collect selections

    # Update long-term averages
    rewards = rewards + (Agent.reward - rewards) / (i + 1)

    # Average actions per episode
    selection = selection + ( Agent.k_n - selection) / (i + 1)

# Get best parameters
best_params = Agent.param_sort[0,:].T
# Agent.env.update(*best_params)
# Agent.env.act(T=5, f=2, A=1, fs=200, showplot=True)
# print('Best parameters:')
# print(best_params)

# plt.figure(figsize=(12, 8))
# for i in range(eps_all.shape[1]):
#     plt.plot(eps_all[:,i], color=(0.9,0,0), alpha=0.5)
# plt.plot(rewards, 'k-', linewidth=3, label="$\epsilon=0.1$")
# plt.legend(bbox_to_anchor=(1.2, 0.5))
# plt.xlabel("Iterations")
# plt.ylabel("Average Reward")
# plt.title("Average $\epsilon-decay$ Rewards after "
#           + str(episodes) + " Episodes")
#
# plt.figure(figsize=(5,5))
# bins = np.linspace(0, k-1, k)
# plt.bar(bins+0.66, selection,
#         width=0.33, color='r',
#         label="$\epsilon=0.2$")
# plt.legend(bbox_to_anchor=(1.2, 0.5))
# plt.xlim([0,k])
# plt.title("Actions Selected by Each Algorithm")
# plt.xlabel("Action")
# plt.ylabel("Number of Actions Taken")
# plt.show()