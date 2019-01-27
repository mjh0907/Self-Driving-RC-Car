# -*- coding: utf-8 -*-
"""
Created on Sun Feb 18 13:30:20 2018

@author: Jerry
"""

# Importing libraries

import numpy as np
import random
import os
import sys
import torch
import torch.nn as nn
import torch.nn.functional as F
import torch.optim as optim
import torch.autograd as autograd
from torch.autograd import Variable

# Creating the architecture of the Neural Network

class Network(nn.Module):
    
    def __init__(self, input_size, nb_action):
        super(Network, self).__init__()
        self.input_size = input_size
        self.nb_action = nb_action
        self.fc1 = nn.Linear(input_size, 50)
        self.fc2 = nn.Linear(50, nb_action)
        
    def forward(self, state):
        x = F.relu(self.fc1(state))
        q_values = self.fc2(x)
        return q_values
    
# Implementing Experience Replay
        
class ReplayMemory(object):
    
    def __init__(self, capacity):
        self.capacity = capacity
        self.memory = []
        
    def push(self, event):
        self.memory.append(event)
        if len(self.memory) > self.capacity:
            del self.memory[0]
            
    def sample(self, batch_size):
        samples = zip(*random.sample(self.memory, batch_size))
        #intlist = range(len(self.memory))
        #candidates = [self.memory[i] for i in intlist if self.memory[i][3] < threshold]
        #samples = zip(*random.sample(candidates, min(len(candidates), batch_size)))
        return map(lambda x: Variable(torch.cat(x, 0)), samples)
        
# Implementing Deep Q Learning
        
class Dqn():
    
    def __init__(self, input_size, nb_action, gamma):
        self.gamma = gamma
        self.reward_window = []
        self.model = Network(input_size, nb_action)
        memory_capacity = 10000
        # terrible/bad/good/excellent memory
        self.memory = [ReplayMemory(memory_capacity), ReplayMemory(memory_capacity), ReplayMemory(memory_capacity), ReplayMemory(memory_capacity)]
        self.reward_threshold = [-2, 0, 0.5, sys.maxint]
        self.memory_threshold = [100, 50, 50, 100]
        self.optimizer = optim.Adam(self.model.parameters(), lr = 0.001)
        self.last_state = torch.Tensor(input_size).unsqueeze(0)
        self.last_action = 0
        self.last_reward = 0
        
    def select_action(self, state):
        #print(state)
        m2 = self.model(Variable(state, volatile = True))*10 # T(Temperature)=10
        #print(m2)
        probs = F.softmax(m2)
        #probs = F.softmax(self.model(Variable(state, volatile = True))*10) # T(Temperature)=10
        #print(probs)
        action = probs.multinomial(num_samples=1) #????
        return action.data[0,0]
    
    def learn(self, batch_state, batch_next_state, batch_reward, batch_action):
        outputs = self.model(batch_state).gather(1, batch_action.unsqueeze(1)).squeeze(1)
        next_outputs = self.model(batch_next_state).detach().max(1)[0]
        target = self.gamma*next_outputs + batch_reward
        td_loss = F.smooth_l1_loss(outputs, target)
        self.optimizer.zero_grad()
        td_loss.backward() #retain_variables = True)
        self.optimizer.step()
        
    def update(self, reward, new_signal):
        new_state = torch.Tensor(new_signal).float().unsqueeze(0)
        for i in range(len(self.memory)):
            if (self.last_reward <= self.reward_threshold[i]):
                #print("last reward is "+repr(self.last_reward))
                #print("push into memory "+repr(i))
                self.memory[i].push((self.last_state, new_state, torch.LongTensor([int(self.last_action)]), torch.Tensor([self.last_reward])))
                break
        for i in range(len(self.memory)):
            if len(self.memory[i].memory) > self.memory_threshold[i]:
                #print("learn from memory "+repr(i))
                batch_state, batch_next_state, batch_action, batch_reward = self.memory[i].sample(self.memory_threshold[i]) #max(100, len(self.memory.memory)/3))
                self.learn(batch_state, batch_next_state, batch_reward, batch_action)
        action = self.select_action(new_state)
        #print("state and action")
        #print(new_state)
        #print(action)
        self.last_action = action
        self.last_state = new_state
        self.last_reward = reward
        self.reward_window.append(reward)
        if len(self.reward_window) > 1000:
            del self.reward_window[0]
        return action
        
    def score(self):
        return sum(self.reward_window)/(len(self.reward_window)+1.)
    
    def save(self):
        torch.save({'state_dict': self.model.state_dict(),
                    'optimizer': self.optimizer.state_dict(),
                    }, 'last_brain.pth')
    
    def load(self):
        if os.path.isfile('last_brain.pth'):
            print("=> loading checkpoint...")
            checkpoint = torch.load('last_brain.pth')
            self.model.load_state_dict(checkpoint['state_dict'])
            self.optimizer.load_state_dict(checkpoint['optimizer'])
            print("done !")
        else:
            print("no checkpoint found...")
        
        
        
        
        
        
        
        
        
        
        
        
        
        
        
        
        
        
