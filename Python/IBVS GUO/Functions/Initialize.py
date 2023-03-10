"""
	Wednesday July 18th, 23:43:12 2018
	@author: robotics lab (Patricia Tavares)
	@email: patricia.tavares@cimat.mx
	version: 1.0
	This code has generic functions to initialize a random amount of cameras and 
	some other functions for initialization.
		- simple_random
		- verified_random_restricted
		- random_evs
		- copy_cameras
		- order_cameras
		- find_nearest
"""

from Functions.PlanarCamera import PlanarCamera
import numpy as np
from random import random
from math import cos, sin, pi
from Functions.Geometric import Rodrigues

"""
	function: simple_random
	description: creates cameras type PlanarCamera with a random pose.
	params: 
		n: how many cameras do you want?
		bounds: the bounds for every param of the camera. It can
			be a list or an np array (12 elements);
			[xlower, xupper,ylower,yupper,zlower,zupper, 
			rolllower, rollupper,pitchlower,pitchupper,yawlower,yawupper]
		noise: noise for the cameras images, defaults to 0 pixels
	returns: 
		cameras: a list of cameras (PlanarCamera) with random pose.
		poses: poses of all the cameras created. This is a numpy array
		with the shape (n,6). The i-th row has the following form:
			x,y,z,roll,pitch,yaw
		that is the init pose of the i-th camera.
"""
def simple_random(n, bounds,noise=None):
	#to store cameras and poses 
	cameras = []
	poses = []	
	#create every camera, considering only the bounds
	for i in range(n):
		#get random params for camera
		x = bounds[0] + random()*(bounds[1]-bounds[0])
		y = bounds[2] + random()*(bounds[3]-bounds[2])
		z = bounds[4] + random()*(bounds[5]-bounds[4])
		roll = bounds[6] + random()*(bounds[7]-bounds[6])
		pitch = bounds[8] + random()*(bounds[9]-bounds[8])
		yaw = bounds[10] + random()*(bounds[11]-bounds[10])
		#create camera
		camera = PlanarCamera()
		camera.set_noise(noise)
		#set position with params
		camera.set_position(x,y,z,roll,pitch,yaw)
		#add to the list
		cameras.append(camera)
		#save pose 
		poses.append([x,y,z,roll,pitch,yaw])

	#return everything
	return cameras,np.array(poses)

"""
	function: verified_random_restricted
	description: creates n cameras type PlanarCamera with a random pose. 		
	params: 
		n: how many cameras do you want?
		min_distance: min distance between two cameras.
		bounds: limits for every axis [xlower,xupper,ylower,yupper,zlower,zupper]
		noise: camera noise, defaults to 0 pixels
	returns: 
		cameras: a list of cameras Planaramera with random pose.
		poses: poses of all the cameras created. This is a numpy array
		with the shape (n,6). The i-th row has the following form:
			x,y,z,yaw,pitch,roll
		that is the init pose of the i-th camera. Because we only have yaw movements
		the roll and pitch are 0.
"""
def verified_random_restricted(n,min_distance,bounds,noise=None):
	#to store cameras and poses 
	cameras = []
	poses = []
	
	for i in range(n):
		camera = PlanarCamera()
		camera.set_noise(noise)
		while(1):
			#get random params for camera
			x = bounds[0] + random()*(bounds[1]-bounds[0])
			y = bounds[2] + random()*(bounds[3]-bounds[2])
			z = bounds[4] + random()*(bounds[5]-bounds[4])
			roll = bounds[6] + random()*(bounds[7]-bounds[6])
			pitch = bounds[8] + random()*(bounds[9]-bounds[8])
			yaw = bounds[10] + random()*(bounds[11]-bounds[10])		
			#set position with params
			camera.set_position(x,y,z,roll,pitch,yaw)
			
			#verify if they are not too close
			ok = True
			for k in range(i):
				prev = np.array([poses[k][0],poses[k][1]])
				actual = np.array([x,y])
				dist = np.linalg.norm(prev-actual)
				if dist < min_distance:
						ok = False						
						break

			#if the distance it's ok, then add it to the list
			if ok:					
				camera.set_position(x,y,z,roll,pitch,yaw)					
				break
		
		#add to the list
		cameras.append(camera)
		#save pose 
		poses.append([x,y,z,roll,pitch,yaw])

	#return everything
	return cameras,np.array(poses)

"""
	function: random_evs
	description: creates n cameras type PlanarCamera with a random rotation. Used
		for epipolar visual servo.		
	params: 
		n: how many cameras do you want?
		noise: camera noise, defaults to 0.
	returns: 
		cameras: a list of cameras Planaramera with random pose.
		poses: poses of all the cameras created. This is a numpy array
		with the shape (n,6). The i-th row has the following form:
			x,y,z,yaw,pitch,roll
		that is the init pose of the i-th camera. Because we only have yaw movements
		the roll and pitch are 0.
"""
def random_evs(n,noise=None):
	#to store cameras and poses 
	cameras = []
	poses = []
	
	for i in range(n):
		camera = PlanarCamera()
		camera.set_noise(2.)
		while(1):
			#get random params for camera
			x = random()*2.
			y = random()*2.
			z = 1.5
			roll = 0. 
			pitch = -pi/2.0 + random()*pi
			yaw = 0.
			#set position with params
			camera.set_position(x,y,z,roll,pitch,yaw)
			
			Ri = camera.euler_to_rotmat(0,pitch,0)
			ok = 0
			for j in range(i):
				Rj = camera.euler_to_rotmat(0,poses[j][4],0)
				Rij = Ri.T.dot(Rj)
				thetaij = np.sum(Rodrigues(Rij))									
				#now, verify they have less than |pi/2| as relative orientation
				#verify is far enough
				if abs(thetaij) < pi/2.0 and abs(poses[j][0]-x)>0.25:
					ok+=1

			#if the distance it's ok, then add it to the list
			if ok == i:					
				break
		
		#add to the list
		cameras.append(camera)
		#save pose 
		poses.append([x,y,z,roll,pitch,yaw])

	#return everything
	return cameras,np.array(poses)

"""
	function: copy_cameras
	description: creates a list with cameras (PlanarCameras) with the 
	params given.
	params: 
		n: how many cameras do you want?
		poses: the poses for the cameras. This is a numpy array
		with the shape (n,6). The i-th row has the following form:
			x,y,z,yaw,pitch,roll
		that is the init pose of the i-th camera. 
	returns: 
		cameras: a list of cameras (PlanarCamera) with the copied pose.
"""
def copy_cameras(n,poses):
	#list of cameras and poses
	cameras = []
	for i in range(n):
		#create camera
		camera = PlanarCamera()		
		#position 
		camera.set_position(poses[i][0],poses[i][1],poses[i][2],poses[i][3],poses[i][4],poses[i][5])
		#add camera
		cameras.append(camera)
	#return what you did
	return cameras

"""
	function: order_cameras
	description: afeter the cameras have been created, we order them to
	take the closest desired pose as reference. This way we can evade those 
	moments when the baseline is near 0.
	params:
		n: the amount of cameras
		cameras: the (PlanarCameras) obtained with methods like random_cameras.
		poses: the poses (numpy array) specifying the camera poses
		desired_poses: the desired poses (formation, numpy array given when created 
			using circle_formation, etc)
	returns:
		new_cameras: the cameras given as param, but with the desired order.
		new_poses: the poses of the cameras given as param, but with the desired order.
"""	
def order_cameras(n,cameras,poses,desired_poses):
	#aux variables for ordering	
	min = 1e20
	min_index = -1
	used = np.ones(n)*(-1.0)	
	#check which camera is closer
	for i in range(n):
		for j in range(n):
			if used[j]==-1:
				dist = np.linalg.norm(poses[i][:3]-desired_poses[j][:3])				
				if dist < min:
					min = dist
					min_index = j
		#save the result
		used[min_index] = i
		min = 1e20
		min_index = -1
	
	#redorder the cameras
	new_cameras = []
	new_poses = []
	for i in range(n):
		new_cameras.append(cameras[int(used[i])])
		new_poses.append(poses[int(used[i])])

	#return
	return new_cameras, np.array(new_poses)

"""
	function: find_nearest 
	description: finds the point nearest to the center in the point cloud and returns its index.
	params: 
		nPoints: amount of points
		points: point cloud. The cloud is generated by the script rand.py in the could folder.
		center: center of the cloud. Is is chosen knowing the limits of the cloud used in the 
		rand.py script.
	returns: 
		index: the index of the point closest to the center in the point cloud.
"""
def find_nearest(nPoints, points, center):
	min = 1e20 #to obtain the point with minimun distance to the center
	index = 0
	for i in range(nPoints):
		distance = np.linalg.norm(points[:,i]-center)
		#save the one closest to the center
		if distance < min:
			min = distance
			index = i
	#return the index of the choosen point.
	return index
