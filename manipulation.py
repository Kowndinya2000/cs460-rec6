import cv2
import numpy as np
import time

from percept import localize_objects
from robot import PickPlaceEnv

# Define and reset environment.
env = PickPlaceEnv()
time.sleep(1)
config = {'pick':  ['yellow block', 'green block', 'blue block'],
          'place': ['yellow bowl', 'green bowl', 'blue bowl']}
env.reset(config)
