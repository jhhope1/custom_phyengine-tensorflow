import sys
import pylab
import environment
import numpy as np
import random
import plotter
from collections import deque
from keras.layers import Dense
from keras.optimizers import Adam
from keras.models import Sequential
from keras import backend as K
#  초기에 학습을 빨리 하게 하기 위해서 weight를 초기화 시켜주는 것.