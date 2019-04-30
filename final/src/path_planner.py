#!/usr/bin/env python

import collections
import sys
import os
import rospy
import numpy as np
from geometry_msgs.msg import PoseWithCovarianceStamped, Pose, PoseArray, PoseStamped
from ackermann_msgs.msg import AckermannDriveStamped
from numpy.linalg import inv
import time
import utils

INITIALPOSE_TOPIC = '/initialpose'
GOAL_TOPIC = "/move_base_simple/goal"
PLANER_NODE_POSE_ARRAY_TOPIC = "/planner_node/car_plan"
FINAL_PLAN_TOPIC = '/planner_node/final_car_plan'

def createInitPose(init_pose):
  init_pose_msg = PoseWithCovarianceStamped()
  init_pose_msg.header.stamp = rospy.Time.now()
  init_pose_msg.header.frame_id = 'map'
  init_pose_msg.pose.pose.position.x = init_pose[0]
  init_pose_msg.pose.pose.position.y = init_pose[1]
  init_pose_msg.pose.pose.position.z = 0
  init_pose_msg.pose.pose.orientation.x = 0
  init_pose_msg.pose.pose.orientation.y = 0
  init_pose_msg.pose.pose.orientation.z = init_pose[2]
  init_pose_msg.pose.pose.orientation.w = init_pose[3]

  return init_pose_msg

def createGoalPose(goal_pose):
  goal_pose_msg = PoseStamped()
  goal_pose_msg.header.stamp = rospy.Time.now()
  goal_pose_msg.header.frame_id = 'map'
  goal_pose_msg.pose.position.x = goal_pose[0]
  goal_pose_msg.pose.position.y = goal_pose[1]
  goal_pose_msg.pose.position.z = 0
  goal_pose_msg.pose.orientation.x = 0
  goal_pose_msg.pose.orientation.y = 0
  goal_pose_msg.pose.orientation.z = goal_pose[2]
  goal_pose_msg.pose.orientation.w = goal_pose[3]

  return goal_pose_msg

def main():
  rospy.init_node('path_planner', anonymous=True)  # Initialize the node

  initial_pose_pub = rospy.Publisher(INITIALPOSE_TOPIC, PoseWithCovarianceStamped, queue_size = 10) # Publishes the most recent laser scan
  goal_pose_pub = rospy.Publisher(GOAL_TOPIC, PoseStamped, queue_size = 10) # Publishes the path of the car
  pose_array_pub = rospy.Publisher(FINAL_PLAN_TOPIC, PoseArray, queue_size=10)

  plan_array_poses= [] #array for poses from plan array
  pose_array = PoseArray()
  good_waypoints = [
                    [49.99999888, 12.79999971, 0.542342069743, 0.840157770533],
                    [52.11999884, 11.1999997, 0.945164118288, -0.326595758546],
                    [47.3542327881, 10.4503288269, 0.987244066665, 0.159214172846],
                    [39.5492019653, 13.1202926636, 0.98515892037, 0.171644695857],
                    [37.59999916, 16.7999998, 0.967361322004, 0.247087236826],
                    [33.38514328, 15.2581377029, -0.800676142281, 0.253401011621],
                   
                    [28.69999936, 12.89999976, 0.896279989118, 0.443488648228],
                    [24.99999944, 14.19999979, -0.972833980528, 0.231503879732],
                    [21.3340473175, 15.462470054, 0.999556369522, 0.0297836221581],
                    [10.79999976, 7.69999963, -0.475298562116, 0.879824571634]     ]

  a = len(good_waypoints)
  #print "size:", a
  #print good_waypoints[1][0]
  
  for i in range(0, a - 1):
    if i is 0:
      print "Waiting for initial pose.."
      initPose = rospy.wait_for_message(INITIALPOSE_TOPIC, PoseWithCovarianceStamped)
    else:
      initPose = createInitPose(good_waypoints[i])
    print "pub initial pose :", initPose.pose.pose.position.x, initPose.pose.pose.position.y
    initial_pose_pub.publish(initPose)
    goalPose = createGoalPose(good_waypoints[i+1])
    print "pub goal pose :", goalPose.pose.position.x, initPose.pose.pose.position.y
    goal_pose_pub.publish(goalPose)

    current_plan = rospy.wait_for_message('/planner_node/car_plan', PoseArray) # wait for planner node to calculate plan
    k= len(current_plan.poses)

    # Convert the plan msg to a list of 3-element numpy arrays
    # Each array is of the form [x,y,theta]
    
    for i in range(k):
      pose_array.header.stamp = rospy.Time.now()  # set header timestamp value
      pose_array.header.frame_id = "map"  # set header frame id value
      tempPose = Pose()
      tempPose.position.x = current_plan.poses[i].position.x
      tempPose.position.y = current_plan.poses[i].position.y
      tempPose.position.z = current_plan.poses[i].position.z
      tempPose.orientation = current_plan.poses[i].orientation
      pose_array.poses.append(tempPose)

  for i in range(1000):
    print "Pubplishing pose array to topic", FINAL_PLAN_TOPIC
    pose_array_pub.publish(pose_array)
    time.sleep(0.5) 
  rospy.spin() # Prevents node from shutting down


if __name__ == '__main__':
  main()