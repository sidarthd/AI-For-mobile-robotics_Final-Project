#!/usr/bin/env python

import rospy
import numpy as np
from threading import Lock
import random

'''
  Provides methods for re-sampling from a distribution represented by weighted samples
'''
class ReSampler:

  '''
    Initializes the resampler
    particles: The particles to sample from
    weights: The weights of each particle
    state_lock: Controls access to particles and weights
  '''
  def __init__(self, particles, weights, state_lock=None):
    self.particles = particles 
    self.weights = weights
    
    # For speed purposes, you may wish to add additional member variable(s) that 
    # cache computations that will be reused in the re-sampling functions
    # YOUR CODE HERE?
    
    if state_lock is None:
      self.state_lock = Lock()
    else:
      self.state_lock = state_lock
  
  '''
    Performs independently, identically distributed in-place sampling of particles
  '''
  def resample_naiive(self):

    self.state_lock.acquire()
    
    self.weights /= np.sum(self.weights)
    k=len(self.weights)
    #resampled_particles= np.zeros((self.particles.shape[0]), dtype= np.float)
    resampled_particles_indices= np.arange(0,k,1)

    resampled_particles_indices = np.random.choice(resampled_particles_indices,size=k,replace= True, p= self.weights)
    resampled_particles_naiive = self.particles[resampled_particles_indices]
    self.particles[:]= resampled_particles_naiive[:]
     # YOUR CODE HERE
    self.state_lock.release()

  '''
    Performs in-place, lower variance sampling of particles
    (As discussed on pg 110 of Probabilistic Robotics)
  '''
  def resample_low_variance(self):
    self.state_lock.acquire()
    '''
    # YOUR CODE HERE
    self.weights /= np.sum(self.weights)
    M= len(self.weights)
    r= np.random.rand(1) * 1.0/M
    c= self.weights[0]
    U_array= np.zeros(M, dtype=float)
    last_element= r+ ((M-1.0)/M)
    U_array= np.linspace(r,last_element, M)#, endpoint=True, retstep= False)
    particles_resampled = np.zeros((self.particles.shape), dtype= np.float)
    weights_cumsum = np.cumsum(self.weights, dtype= np.float)
    particle_bins= np.digitize(U_array, weights_cumsum, right=False)
    print particle_bins
    particles_resampled= self.particles[particle_bins]
    self.particles[:]= particles_resampled[:]

    '''
    resampled_particles= np.zeros((self.particles.shape), dtype= np.float)
    #print resampled_particles
    M= len(self.weights)
    r= np.random.rand(1) * 1.0/M
    c= self.weights[0]
    i=0
    for m in xrange(M):
      U= r+ (m* 1.0/M)
      while U>c:
        i+=1
        c=c + self.weights[i]
      resampled_particles[m,:]= self.particles[i,:]  
      #print resampled_particles[m,:]
    self.particles[:] = resampled_particles[:]
    #print resampled_particles
    #print self.particles 
   
    self.state_lock.release()
    
import matplotlib.pyplot as plt

if __name__ == '__main__':

  rospy.init_node("sensor_model", anonymous=True) # Initialize the node

  n_particles = int(rospy.get_param("~n_particles",100)) # The number of particles    
  k_val = int(rospy.get_param("~k_val", 80)) # Number of particles that have non-zero weight
  resample_type = rospy.get_param("~resample_type", "naiive") # Whether to use naiive or low variance sampling
  trials = int(rospy.get_param("~trials", 10)) # The number of re-samplings to do
  
  histogram = np.zeros(n_particles, dtype=np.float) # Keeps track of how many times
                                                    # each particle has been sampled
                                                    # across trials
  
  for i in xrange(trials):
    particles = np.repeat(np.arange(n_particles)[:,np.newaxis],3, axis=1) # Create a set of particles
                                                                          # Here their value encodes their index
    # Have increasing weights up until index k_val
    weights = np.arange(n_particles, dtype=np.float)
    weights[k_val:] = 0.0
    weights[:] = weights[:] / np.sum(weights)
    
    rs = ReSampler(particles, weights) # Create the Resampler
  
    # Resample
    if resample_type == "naiive":
      rs.resample_naiive()
    elif resample_type == "low_variance":
      rs.resample_low_variance()
    else:
      print "Unrecognized resampling method: "+ resample_type     

    # Add the number times each particle was sampled    
    for j in xrange(particles.shape[0]):
      histogram[particles[j,0]] = histogram[particles[j,0]] + 1
    
  # Display as histogram
  plt.bar(np.arange(n_particles), histogram)
  plt.xlabel('Particle Idx')
  plt.ylabel('# Of Times Sampled')
  plt.show()    

