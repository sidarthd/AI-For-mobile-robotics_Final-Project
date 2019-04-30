#!/usr/bin/env python

import rospy
import numpy as np
import math
import sys

import utils

from sensor_msgs.msg import LaserScan
from ackermann_msgs.msg import AckermannDriveStamped
from geometry_msgs.msg import PoseStamped, PoseArray, Pose

SCAN_TOPIC = '/scan' # The topic to subscribe to for laser scans
CMD_TOPIC = '/vesc/high_level/ackermann_cmd_mux/input/nav_0' # The topic to publish controls to
POSE_TOPIC = '/sim_car_pose/pose' # The topic to subscribe to for current pose of the car
                                  # NOTE THAT THIS IS ONLY NECESSARY FOR VIZUALIZATION
VIZ_TOPIC = '/laser_wanderer/rollouts' # The topic to publish to for vizualizing
                                       # the computed rollouts. Publish a PoseArray.
MAP_TOPIC = 'static_map'

MAX_PENALTY = 10000 # The penalty to apply when a configuration in a rollout
                    # goes beyond the corresponding laser scan
                    

'''
Wanders around using minimum (steering angle) control effort while avoiding crashing
based off of laser scans. 
'''
class LaserWanderer:

  '''
  Initializes the LaserWanderer
    rollouts: An NxTx3 numpy array that contains N rolled out trajectories, each
              containing T poses. For each trajectory, the t-th element represents
              the [x,y,theta] pose of the car at time t+1
    deltas: An N dimensional array containing the possible steering angles. The n-th
            element of this array is the steering angle that would result in the 
            n-th trajectory in rollouts
    speed: The speed at which the car should travel
    compute_time: The amount of time (in seconds) we can spend computing the cost
    laser_offset: How much to shorten the laser measurements
  '''
  def __init__(self, rollouts, deltas, speed, compute_time, laser_offset):
    # Store the params for later
    self.rollouts = rollouts
    self.deltas = deltas
    self.speed = speed
    self.compute_time = compute_time
    self.laser_offset = laser_offset
    #self.map_img, self.map_info = utils.get_map(MAP_TOPIC)
    
    # YOUR CODE HERE
    self.cmd_pub = rospy.Publisher(CMD_TOPIC, AckermannDriveStamped, queue_size = 10) # Create a publisher for sending controls
    self.laser_sub = rospy.Subscriber(SCAN_TOPIC, LaserScan, self.wander_cb) # Create a subscriber to laser scans that uses the self.wander_cb callback
    self.viz_pub = rospy.Publisher(VIZ_TOPIC, PoseArray, queue_size = 10) # Create a publisher for vizualizing trajectories. Will publish PoseArrays
    self.viz_sub = rospy.Subscriber(POSE_TOPIC, PoseStamped, self.viz_sub_cb) # Create a subscriber to the current position of the car
    # NOTE THAT THIS VIZUALIZATION WILL ONLY WORK IN SIMULATION. Why?
    
  '''
  Vizualize the rollouts. Transforms the rollouts to be in the frame of the world.
  Only display the last pose of each rollout to prevent lagginess
    msg: A PoseStamped representing the current pose of the car
  '''  
  def viz_sub_cb(self, msg):
    # Create the PoseArray to publish. Will contain N poses, where the n-th pose
    # represents the last pose in the n-th trajectory
    # PP - get current pose from mgs
    cur_pose = np.array([msg.pose.position.x,
                         msg.pose.position.y,
                         utils.quaternion_to_angle(msg.pose.orientation)])

    pa = PoseArray()
    pa.header.frame_id = '/map'
    pa.header.stamp = rospy.Time.now()
    

    # Transform the last pose of each trajectory to be w.r.t the world and insert into
    for i in range (self.rollouts.shape[0]):

      # PP - below code is similar to code in line_follower for transformation
      trans_mat = [cur_pose[0], cur_pose[1]]
      rot_mat = utils.rotation_matrix(cur_pose[2])

      arrow_pose = (self.rollouts[i][-1][0], self.rollouts[i][-1][1])
      arrow_pose = np.reshape(arrow_pose, (2, 1))

      trans_mat = np.reshape(trans_mat, (2, 1))

      arrow_wrt_cur_pose = rot_mat * arrow_pose + trans_mat # co ordinate transformation
      
      # create Pose to add in PoseArray
      pose = Pose()
      pose.position.x = arrow_wrt_cur_pose[0]
      pose.position.y = arrow_wrt_cur_pose[1]
      pose.position.z = 0
      pose.orientation = utils.angle_to_quaternion(self.rollouts[i][-1][2] + cur_pose[2])  # PP - is ths correct? verify with Patrick

      #print last_pose_in_traj
      pa.poses.append(pose)

    self.viz_pub.publish(pa)
    
  '''
  Compute the cost of one step in the trajectory. It should penalize the magnitude
  of the steering angle. It should also heavily penalize crashing into an object
  (as determined by the laser scans)
    delta: The steering angle that corresponds to this trajectory
    rollout_pose: The pose in the trajectory 
    laser_msg: The most recent laser scan
  '''  
  def compute_cost(self, delta, rollout_pose, laser_msg):
  
    # Initialize the cost to be the magnitude of delta
    # Consider the line that goes from the robot to the rollout pose
    # Compute the angle of this line with respect to the robot's x axis
    # Find the laser ray that corresponds to this angle
    # Add MAX_PENALTY to the cost if the distance from the robot to the rollout_pose 
    # is greater than the laser ray measurement - np.abs(self.laser_offset)
    # Return the resulting cost
    # Things to think about:
    #   What if the angle of the pose is less (or greater) than the angle of the
    #   minimum (or maximum) laser scan angle
    #   What if the corresponding laser measurement is NAN or 0?
    # NOTE THAT NO COORDINATE TRANSFORMS ARE NECESSARY INSIDE OF THIS FUNCTION
    
    # YOUR CODE HERE
    
    cost = np.abs(delta) # PP -initiating cost with angle magnitude
    #curr_pose = self.rollouts[0][0]         # PP - how to get current pose ?

    curr_rollout_angle = np.arctan(rollout_pose[1]/ rollout_pose[0]) # PP - compute angle between x axis of current pose with rollout pose, how?? :P
    curr_rollout_pose_dist = np.sqrt(pow(rollout_pose[0], 2) + pow(rollout_pose[1], 2))

    laser_index = (int)((curr_rollout_angle - laser_msg.angle_min) / laser_msg.angle_increment)
    # find corresponding laser angle and its range
    
    laser_range = laser_msg.ranges[laser_index]
    #if curr_rollout_pose_dist

    #invalid_laser_range = (laser_range == 0.0) or (np.isnan(laser_range)) or (np.isinf(laser_range))
    valid_laser_range = np.isfinite(laser_range) or laser_range != 0.0


    if valid_laser_range and curr_rollout_pose_dist >  laser_range - np.abs(self.laser_offset):
      cost = cost + MAX_PENALTY
      return cost

    # if an angle of the pose is less or greater that the angle of the min or max
    if curr_rollout_angle < laser_msg.angle_min or curr_rollout_angle > laser_msg.angle_max:
      cost = cost + MAX_PENALTY
      return cost
    
    

    return cost # PP - return cost
    
  '''
  Controls the steering angle in response to the received laser scan. Uses approximately
  self.compute_time amount of time to compute the control
    msg: A LaserScan
  '''
  def wander_cb(self, msg):
    start = rospy.Time.now().to_sec() # Get the time at which this function started
    
    # A N dimensional matrix that should be populated with the costs of each
    # trajectory up to time t <= T
    delta_costs = np.zeros(self.deltas.shape[0], dtype=np.float) 
    #delta_costs= deltas[3]
    traj_depth = 0
    
    # Evaluate the cost of each trajectory. Each iteration of the loop should calculate
    # the cost of each trajectory at time t = traj_depth and add those costs to delta_costs
    # as appropriate
    
    # Pseudo code
    # while(you haven't run out of time AND traj_depth < T):
    #   for each trajectory n:
    #       delta_costs[n] += cost of the t=traj_depth step of trajectory n
    #   traj_depth += 1 
    # YOUR CODE HERE

    # PP - adding cases as mentioned in above pseudo code
    while(rospy.Time.now().to_sec() - start < self.compute_time and traj_depth < self.rollouts.shape[1]): 
      for i in range (self.rollouts.shape[0]): # PP - no of elements i.e N from rollouts array (NxTx3)
         # PP - calling compute cost by sending deltas and rollouts and laser msg
        delta_costs[i] += self.compute_cost(self.deltas[i], self.rollouts[i][traj_depth], msg)
        #print delta_costs[i]
      traj_depth += 1

    # Find the delta that has the smallest cost and execute it by publishing
    # YOUR CODE HERE

    # Setup the publish message
    delta_index = np.argmin(delta_costs)
    ads = AckermannDriveStamped()
    ads.header.frame_id = '/map' # PP - adding AckermannDriveStamped topic attributes
    ads.header.stamp = rospy.Time.now() # PP - adding AckermannDriveStamped topic attributes
    ads.drive.steering_angle = self.deltas[delta_index] # PP - adding min cost delta from delta_cost array
    ads.drive.speed = self.speed # PP - passing on speed from class args
    
    # publish message
    self.cmd_pub.publish(ads)
    
'''
Apply the kinematic model to the passed pose and control
  pose: The current state of the robot [x, y, theta]
  control: The controls to be applied [v, delta, dt]
  car_length: The length of the car
Returns the resulting pose of the robot
'''
def kinematic_model_step(pose, control, car_length):
  # Apply the kinematic model
  # Make sure your resulting theta is between 0 and 2*pi
  # Consider the case where delta == 0.0
  control_delta = control[1]
  
  if control_delta == 0.0:
    theta_next = pose[2]
    x_next = pose[0] + control[0] * control[2] * np.cos(pose[2])
    y_next = pose[1] + control[0] * control[2] * np.sin(pose[2])
    #y_next = 0.0
    return(x_next, y_next, theta_next)


  beta = np.arctan(np.tan(control_delta)/2)
  #print beta  
  
  if beta == 0.0:
    theta_next = pose[2]
    x_next = pose[0] + control[0] * control[2] * np.cos(pose[2])
    y_next = pose[1] + control[0] * control[2] * np.cos(pose[2])
  
  else:
    theta_next = pose[2] + ((control[0]/car_length) * np.sin(2*beta) * control[2])  
    if theta_next < 0.0:
      theta_next = 2*np.pi + theta_next
    elif theta_next > 2*np.pi:
      theta_next = theta_next - 2*np.pi
    x_next = pose[0] + (car_length/np.sin(2*beta)) * (np.sin(theta_next) - np.sin(pose[2]))
    y_next = pose[1] + (car_length/np.sin(2*beta)) * (-np.cos(theta_next) + np.cos(pose[2]))

  
  #print x_next, y_next, theta_next
  return(x_next, y_next, theta_next)
  
  # YOUR CODE HERE
    
'''
Repeatedly apply the kinematic model to produce a trajectory for the car
  init_pose: The initial pose of the robot [x,y,theta]
  controls: A Tx3 numpy matrix where each row is of the form [v,delta,dt]
  car_length: The length of the car
Returns a Tx3 matrix where the t-th row corresponds to the robot's pose at time t+1
'''
def generate_rollout(init_pose, controls, car_length):
  # YOUR CODE HERE
  # call kinematic model to calculate rollout
  numPoses = controls.shape[0]
  rollout = np.zeros((numPoses,3), dtype=np.float)
  poseX = init_pose
  for i in range(numPoses):
    control = np.zeros(3, dtype=np.float)
    control[0] = controls[i][0]
    control[1] = controls[i][1]
    control[2] = controls[i][2]
    rollout[i] = kinematic_model_step(poseX, control, car_length)
    poseX = rollout[i]
  return rollout  
  
   
'''
Helper function to generate a number of kinematic car rollouts
    speed: The speed at which the car should travel
    min_delta: The minimum allowed steering angle (radians)
    max_delta: The maximum allowed steering angle (radians)
    delta_incr: The difference (in radians) between subsequent possible steering angles
    dt: The amount of time to apply a control for
    T: The number of time steps to rollout for
    car_length: The length of the car
Returns a NxTx3 numpy array that contains N rolled out trajectories, each
containing T poses. For each trajectory, the t-th element represents the [x,y,theta]
pose of the car at time t+1
'''
def generate_mpc_rollouts(speed, min_delta, max_delta, delta_incr, dt, T, car_length):

  # PP - np.arrage Return evenly spaced values within a given interval.
  # PP - here its returning an array of deltas between min & max in steps of delta_incr
  deltas = np.arange(min_delta, max_delta, delta_incr) 

  # PP - deltas.shape is returning no. of elements in deltas --which is N rolled out trajectories
  N = deltas.shape[0]
  
  #
  init_pose = np.array([0.0,0.0,0.0], dtype=np.float)
  
  rollouts = np.zeros((N,T,3), dtype=np.float)
  for i in xrange(N):
    controls = np.zeros((T,3), dtype=np.float)
    controls[:,0] = speed
    controls[:,1] = deltas[i]
    controls[:,2] = dt
    rollouts[i,:,:] = generate_rollout(init_pose, controls, car_length)
    #print rollouts[i][-1] 
  #print rollouts[:,-1,:]
  #exit()
  return rollouts, deltas
    

def main():

  rospy.init_node('laser_wanderer', anonymous=True)

  # Load these parameters from launch file
  # We provide suggested starting values of params, but you should
  # tune them to get the best performance for your system
  # Look at constructor of LaserWanderer class for description of each var
  # 'Default' values are ones that probably don't need to be changed (but you could for fun)
  # 'Starting' values are ones you should consider tuning for your system  
  # YOUR CODE HERE
  
  # PP - updated below values as per recommended start point
  speed = 1.0 # Default val: 1.0
  min_delta =  -0.34 # Default val: -0.34
  max_delta =  0.341 # Default val: 0.341
  delta_incr = 0.34/3 # Starting val: 0.34/3 (consider changing the denominator) 
  dt = 0.01 # Default val: 0.01
  T = 300 # Starting val: 300
  compute_time = 0.09 # Default val: 0.09
  laser_offset = 1.0 # Starting val: 1.0
  '''
  # PP - updated below to get params from launch file.
  speed = rospy.get_param('speed')# Default val: 1.0
  min_delta = rospy.get_param('min_delta')# Default val: -0.34
  max_delta = rospy.get_param('max_delta')# Default val: 0.341
  delta_incr = rospy.get_param('delta_incr')# Starting val: 0.34/3 (consider changing the denominator)
  dt = rospy.get_param('dt')# Default val: 0.01
  T = rospy.get_param('T')# Starting val: 300
  compute_time = rospy.get_param('compute_time') # Default val: 0.09
  laser_offset = rospy.get_param('laser_offset')# Starting val: 1.0
  delta_incr = delta_incr/3  
  '''
  # DO NOT ADD THIS TO YOUR LAUNCH FILE, car_length is already provided by teleop.launch
  car_length = rospy.get_param("car_kinematics/car_length", 0.33) 
  
  # Generate the rollouts
  rollouts, deltas = generate_mpc_rollouts(speed, min_delta, max_delta,
                                           delta_incr, dt, T, car_length)
  
  # Create the LaserWanderer                                         
  lw = LaserWanderer(rollouts, deltas, speed, compute_time, laser_offset)
  
  # Keep the node alive
  rospy.spin()
  

if __name__ == '__main__':
  main()