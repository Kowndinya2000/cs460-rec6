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

top_view_rgb_img  = env.get_camera_image_top()
top_view_rgb_img = cv2.cvtColor(top_view_rgb_img, cv2.COLOR_RGB2BGR)
top_view_rgb_img = np.flipud(top_view_rgb_img.transpose(1, 0, 2))
top_view_rgb_img = cv2.pyrUp(
    top_view_rgb_img, 
    dstsize=(
        top_view_rgb_img.shape[1]*2,
        top_view_rgb_img.shape[0]*2
    )
)

cv2.imwrite("top_view.png", top_view_rgb_img)

# Perform localization of the objects
perception_results = localize_objects(top_view_rgb_img)

# Pick and place 
actions = {
    "pick": perception_results['yellow block'],
    "place" : perception_results['yellow bowl']
}
env.step(action=actions)

actions = {
    "pick": perception_results['blue block'],
    "place" : perception_results['blue bowl']
}
env.step(action=actions)

actions = {
    "pick": perception_results['green block'],
    "place" : perception_results['green bowl']
}
env.step(action=actions)

time.sleep(5)
