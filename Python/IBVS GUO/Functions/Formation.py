"""
	Monday September 24th, 21:51:13 2018
	@author: robotics lab (Patricia Tavares)
	@email: patricia.tavares@cimat.mx
	version: 3.0

	This code contains functions to compute necessary elements for a formation.
		- line_formation
		- get_relative
		- circle_formation
		- get_L_radius
		- get_A
		- get_relative
		- get_epipoles
"""

from Functions.PlanarCamera import PlanarCamera
from math import cos, sin, pi
import numpy as np
import cv2
from Functions.Geometric import homog_to_rt, Rodrigues, H_from_points
from Functions.Aux import closer_element

"""
	function: line_formation
	description: creates cameras type PlanarCamera with a fixed pose, in
	the end al cameras should be formed in a line.
	params:
		n: how many cameras do you have?
		bounds: the bounds for every param of the camera. It can
			be a list or an np array (2 elements);
			[xlower, xupper], for facility, we don't consider them to have a
			rotation in the final formation and also, we dont need
			x and y bounds since it is a line.
		noise: the noise for the images of the camera (pixels) defaults 0
	returns:
		cameras: a list of cameras (PlanarCamera) with de desired pose.
		poses: poses of all the cameras created. This is a numpy array
		with the shape (n,6). The i-th row has the following form:
			x,y,z,yaw,pitch,roll
		that is the init pose of the i-th camera. Because we want
		the cameras to be facing the "ground", roll and yaw are 0 for every
		camera.
"""
def line_formation(n,bounds,noise=None):
	#list of cameras and poses
	cameras = []
	poses = []
	for i in range(n):
		#create camera
		camera = PlanarCamera()
		camera.set_noise(noise)
		#position to have a line for this camera
		camera.set_position(bounds[0]+i*((bounds[1]-bounds[0])/(n-1)),1,1,0,0,0)
		#add camera
		cameras.append(camera)
		poses.append([bounds[0]+i*((bounds[1]-bounds[0])/(n-1)),1,1,0,0,0])
	#return what you did
	return cameras,np.array(poses)

"""
	function: circle_formation
	description: creates cameras type PlanarCamera with a fixed pose, in
	the end al cameras should be formed in a circle.
	params:
		n: how many cameras do you have?
		radius: the radius of the formation (float)
		center: the coordinates of the center of the circle (list or
			array, 2 elements) [cx,cy]
		scaled: if we want a 3D formation, defaults to no
		noise: the noise for the images of the camera (pixels) defaults 0
	returns:
		cameras: a list of cameras (PlanarCamera) with de desired pose.
		poses: poses of all the cameras created. This is a numpy array
		with the shape (n,6). The i-th row has the following form:
			x,y,z,yaw,pitch,roll
		that is the init pose of the i-th camera. Because we want
		the cameras to be facing the "ground", roll and yaw are 0 for every
		camera.
"""
def circle_formation(n,radius,center,scaled=0.,noise =None):
	#list of cameras and poses
	cameras = []
	poses = []
	for i in range(n):
		#create camera
		camera = PlanarCamera()
		camera.set_noise(noise)
		#position to have a line for this camera
		camera.set_position(center[0]+radius*cos((2*pi/n)*i),center[1]+radius*sin((2*pi/n)*i),1+scaled*i,0,0,0)
		#add camera
		cameras.append(camera)
		poses.append([center[0]+radius*cos((2*pi/n)*i),center[1]+radius*sin((2*pi/n)*i),1+scaled*i,0,0,0])
	#return what you did
	return cameras,np.array(poses)

"""
	function: get_L_radius
	description: computes a Laplacian. Is makes connections with the
	neighbors inside a spicified sphere.
	params:
		n_cameras: how many cameras do you have?
		poses: poses of the cameras.
		r: radius of the sphere.
	returns:
		L: Laplacian
"""
def get_L_radius(n_cameras,poses,r):
	L = np.zeros((n_cameras,n_cameras))
	for i in range(n_cameras):
		c = 0 #counting adyacenes
		p1 = poses[i,:2]
		for j in range(n_cameras):
			p2 = poses[j,:2]
			if np.linalg.norm(p1-p2)<r and i!=j:
				L[i][j] = 1
				c = c+1
		L[i][i] = - c
	return L

"""
	function: get_A
	description: computes the doubly stochastic matrix A for scale consensus as seen in paper
	Fast linear iterations for distributed averaging.
	params:
		n_cameras: how many cameras do you have?
		L: Laplacian
	returns:
		A: the doubly stochastic matrix A
"""
def get_A(n_cameras,L):
	A = np.zeros((n_cameras,n_cameras))

	u,s,V = np.linalg.svd(L)
	alpha = 2.0/(s[0]**2+s[n_cameras-2]**2)
	print(L)
	for i in range(n_cameras):
		for j in range(n_cameras):
			if L[i][j]==1 and i!=j:
				A[i][j] = alpha
			elif i==j:
				A[i][j] = 1 - (-L[i][j])*alpha
	return A

"""
	function: get_relative
	description: computes the relative position and relative Rotation
	between the cameras, using a laplacian.
	params:
		n: how many cameras do you have?
		cameras: list of cameras to be computed
		L: Laplacian matrix showing how the graph is connected.
	returns:
		p: a dictionary with al the relative positions, if you
		call p[(i,j)] it returns the relative position between
		camera i and camera j.
		R: a dictionary with al the relative rotations, if you
		call R[(i,j)] it returns the relative rotarion between
		camera i and camera j.
"""
def get_relative(n,cameras,L,normalize=False):
	#init the dictionary
	p = {}
	R = {}
	#for every i,j made with the list
	for i in range(n):
		for j in range(n):
			if L[i][j]==1 and j!=i:
				#compute Rij and pij
				p[(i,j)] = cameras[i].R.T.dot(cameras[j].t-cameras[i].t)
				R[(i,j)] = cameras[i].R.T.dot(cameras[j].R)
	#return results
	return p,R

"""
	function: get_bearings
	description: computes the epipoles for the consensus using gt, it
		is usually used for the desired formation
	params:
		n: the amount of cameras
		cameras: the cameras in the formation
		L: Laplacian
	returns.
		e: dict with the epipoles between pairs
		R: relative rotation between pairs
"""
def get_bearings(n,cameras,L):
	B = {}
	R = {}

	for i in range(n):
		for j in range(n):
			if L[i][j]==1 and i!=j:
				R[(i,j)] = cameras[i].R.T.dot(cameras[j].R)
				b = cameras[i].R.T.dot(cameras[j].t-cameras[i].t)
				b = b / np.linalg.norm(b)
				B[(i,j)]= b
	return B, R
