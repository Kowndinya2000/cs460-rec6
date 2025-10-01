# This code is based on code from google SayCan (Retrieved on 04/10/2023)
#
# https://github.com/google-research/google-research/tree/master/saycan
#
# This file contains a minimum working example of a pyBullet simulation 
# of a UR-5e robot arm pick and place a few colored cubes. In contrast
# to the original code, the mirroring control of the robotiq gripper is
# done without the use of threads, which caused segmentation fault on 
# MacOS (as of 04/10/2023).

import os
import time
import numpy as np
import pybullet
import pybullet_data
from pybullet_utils import bullet_client
from gripper import Robotiq2F85
from variables import PICK_TARGETS, COLORS, PLACE_TARGETS, BOUNDS, PIXEL_SIZE


class PickPlaceEnv():

  def __init__(self):
    self.dt = 1/240
    self.sim_step = 0

    # Initialize PyBullet client in GUI mode
    self._pb = bullet_client.BulletClient(connection_mode=pybullet.GUI)
    self._client_id = self._pb._client

    # Add search paths for assets
    # TODO: Add pybullet_data and local assets to search path

    # Set simulation timestep
    # TODO: set time step using self._pb.setTimeStep()

    # Configure camera view (angle, zoom, target position)
    # TODO: use resetDebugVisualizerCamera()

    # Robot Home Configuration
    # TODO: home joint pos, link IDs of the robot and End Effector (EE).


  def reset(self, config):
    self._pb.resetSimulation(pybullet.RESET_USE_DEFORMABLE_WORLD)
    self._pb.setGravity(0, 0, -9.8)
    self.cache_video = []
  
    # Load Plane
    # TODO: Your Code here

    # Load UR5e arm
    # TODO: Your Code here

    self.joint_ids = [self._pb.getJointInfo(self.robot_id, i) for i in range(self._pb.getNumJoints(self.robot_id))]
    self.joint_ids = [j[0] for j in self.joint_ids if j[2] == pybullet.JOINT_REVOLUTE]

    # Move robot to home configuration.
    for i in range(len(self.joint_ids)):
      # TODO: Your code here
      pass

    # Add gripper.
    self.gripper = Robotiq2F85(self._pb, self.robot_id, self.ee_link_id)
    self.gripper.release()

    # Add workspace.
    plane_shape = self._pb.createCollisionShape(pybullet.GEOM_BOX, halfExtents=[0.3, 0.3, 0.001])
    plane_visual = self._pb.createVisualShape(pybullet.GEOM_BOX, halfExtents=[0.3, 0.3, 0.001])
    plane_id = self._pb.createMultiBody(0, plane_shape, plane_visual, basePosition=[0, -0.5, 0])
    self._pb.changeVisualShape(plane_id, -1, rgbaColor=[0.2, 0.2, 0.2, 1.0])

    # Load objects according to config.
    self.config = config
    self.obj_name_to_id = {}
    obj_names = list(self.config["pick"]) + list(self.config["place"])
    obj_xyz = np.zeros((0, 3))
    for obj_name in obj_names:
      if ("block" in obj_name) or ("bowl" in obj_name):

        # Get random position 15cm+ from other objects.
        while True:
          rand_x = np.random.uniform(BOUNDS[0, 0] + 0.1, BOUNDS[0, 1] - 0.1)
          rand_y = np.random.uniform(BOUNDS[1, 0] + 0.1, BOUNDS[1, 1] - 0.1)
          rand_xyz = np.float32([rand_x, rand_y, 0.03]).reshape(1, 3)
          if len(obj_xyz) == 0:
            obj_xyz = np.concatenate((obj_xyz, rand_xyz), axis=0)
            break
          else:
            nn_dist = np.min(np.linalg.norm(obj_xyz - rand_xyz, axis=1)).squeeze()
            if nn_dist > 0.15:
              obj_xyz = np.concatenate((obj_xyz, rand_xyz), axis=0)
              break
        
        object_color = COLORS[obj_name.split(" ")[0]]
        object_type = obj_name.split(" ")[1]
        object_position = rand_xyz.squeeze()
        if object_type == "block":
          object_shape = self._pb.createCollisionShape(pybullet.GEOM_BOX, halfExtents=[0.02, 0.02, 0.02])
          object_visual = self._pb.createVisualShape(pybullet.GEOM_BOX, halfExtents=[0.02, 0.02, 0.02])
          object_id = self._pb.createMultiBody(0.01, object_shape, object_visual, basePosition=object_position)
        elif object_type == "bowl":
          object_position[2] = 0
          object_id = self._pb.loadURDF("bowl/bowl.urdf", object_position, useFixedBase=1)
        self._pb.changeVisualShape(object_id, -1, rgbaColor=object_color)
        self.obj_name_to_id[obj_name] = object_id

    for _ in range(200):
      self.gripper.update()
      self._pb.stepSimulation()

    # return object positions and observations
    return obj_xyz, self.get_observation()

  def servoj(self, joints):
    """Move to target joint positions with position control."""
    # TODO: Your code here
    pass
  
  def movep(self, position):
    """Move to target end effector position."""
    # TODO: Your code here
    pass

  def step(self, action=None):
    """Do pick and place motion primitive."""
    pick_xyz, place_xyz = action["pick"].copy(), action["place"].copy()

    # Set fixed primitive z-heights.
    hover_xyz = pick_xyz.copy() + np.float32([0, 0, 0.2])
    pick_xyz[2] = 0.025
    place_xyz[2] = 0.15

    # TODO: Move to hover above pick 

    # TODO: Descend to pick


    # TODO: Close gripper

    # TODO: Lift back up

    # TODO: Move to place location

    # TODO: Open gripper (release object)

  def step_sim_and_render(self):
    self.gripper.update()
    self._pb.stepSimulation()
    self.sim_step += 1

    # Slow things down a bit... 
    #if self.sim_step % 10 == 0:
    #  time.sleep(0.02)  

    # Render current image at 16 FPS.
    if self.sim_step % 30 == 0:
      self.cache_video.append(self.get_camera_image())
