import numpy as np
import pybullet
import time

class Robotiq2F85:
  """Gripper handling for Robotiq 2F85."""

  def __init__(self, bulletClient, robot, tool):
    self._pb = bulletClient
    self.robot = robot
    self.tool = tool
    
    # TODO: Set an initial position (x, y, z) for the gripper base.
    
    # TODO: Convert desired orientation from Euler angles to quaternion.
    
    # TODO: Load the Robotiq 2F-85 URDF into simulation.
   
    # TODO: Get the number of joints in the gripper model.
   
    # TODO: Create a fixed joint constraint between robot tool and gripper base.
   
    # TODO: Set dynamics (friction coefficients) for all gripper fingers.
   
    # TODO: Define which joint is the motor joint (controls open/close).
   
  def update(self):
    """Update gripper finger joint positions to mimic motor joint."""
    # TODO: Read current joint states.
   
    # TODO: Define which joints should mimic the motor joint.
   
    # TODO: Define target joint values as a function of the motor joint.
   
    # TODO: Send position control commands to mimic joints.
   
  def activate(self):
    """Close gripper fingers."""
    # Apply velocity control to close motor joint.
    self._pb.setJointMotorControl2(
      self.body, self.motor_joint,
      pybullet.VELOCITY_CONTROL,
      targetVelocity=1, force=10
    )
    self.activated = True

  def release(self):
    """Open gripper fingers."""
    # Apply velocity control to open motor joint.
    self._pb.setJointMotorControl2(
      self.body, self.motor_joint,
      pybullet.VELOCITY_CONTROL,
      targetVelocity=-1, force=10
    )
    self.activated = False

  def detect_contact(self):
    """
    Detect contact depending on activation state:
      - If activated & grasping something: check object contact.
      - If activated & empty: check gripper contact.
      - If released: TODO (currently disabled).
    """
    # TODO: Use raycast proximity to detect possible contact.
    obj, _, ray_frac = self.check_proximity()

    if self.activated:
      # TODO: Check if fingers are nearly closed (empty grasp).
      
      # TODO: Ensure we are not detecting collisions with self or world (id=0).
      
      # TODO: Return whether external contacts exist.
        pass    
    pass
  
  def external_contact(self, body=None):
    """Check if given body is in contact with something external."""
    if body is None:
      body = self.body
    # Get all contact points involving this body.
    pts = self._pb.getContactPoints(bodyA=body)
    # Filter out self-collisions.
    pts = [pt for pt in pts if pt[2] != self.body]
    return len(pts) > 0

  def check_grasp(self):
    """Wait until gripper stops moving, then check if grasp succeeded."""
    #Wait until gripper finishes moving.
    while self.moving():
      time.sleep(0.001)
    # TODO: Decide success if grasp width is above threshold.
    pass 
  def grasp_width(self):
    """Compute distance between left and right fingertips."""
    # TODO: Get world positions of left & right finger pads.
    
    # TODO: Compute Euclidean distance minus offset.
    pass

  def check_proximity(self):
    """Raycast from end-effector to tool to detect nearby objects."""
    # Get end-effector and tool positions.
    ee_pos = np.array(self._pb.getLinkState(self.robot, self.tool)[0])
    tool_pos = np.array(self._pb.getLinkState(self.body, 0)[0])

    # Normalize direction vector.
    vec = (tool_pos - ee_pos) / np.linalg.norm((tool_pos - ee_pos))

    # Create ray endpoint slightly ahead of EE.
    ee_targ = ee_pos + vec

    # Cast ray and unpack result.
    ray_data = self._pb.rayTest(ee_pos, ee_targ)[0]
    obj, link, ray_frac = ray_data[0], ray_data[1], ray_data[2]
    return obj, link, ray_frac
