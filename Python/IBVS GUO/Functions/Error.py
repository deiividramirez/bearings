"""
	Monday September 24th, 21:54:28 2018
	@author: robotics lab (Patricia Tavares)
	@email: patricia.tavares@cimat.mx
	version: 2.0

	This code contains function used to compute the error.
		- get_error
		- get_error_rot
		- distances
		- transform_to_frame
"""

import numpy as np
from Functions.Aux import clip
from math import acos

"""
	function: get_error
	description: computes the error between the actual pose and the desired (relative)
		in a fixed reference frame
	params: 
		p_n: relative positions transformed to the desired frame
		p_n_a: aster: relative desired positions transformed to the desired frame
		R: relative rotations
		R_aster: relative desired rotations
		cameras: actual cameras (PlanarCamera)
		desired_cameras: list of PlanarCamera
		dist: real distances between every pair of drones, use distances 
			function to compute it
		dist_aster: same as dist, but for desired cameras
	returns: 
		err_tr: error in relative translation.
		err_rot: error in relative rotation.
		err_sc: error scale.
"""
def get_error(p_n,p_n_a,R,R_aster,cameras,desired_cameras, dist, dist_aster):
	error_sc = 0.0
	n = 1.

	#error for scale
	if dist != None:#if we want to measure scale error (when not estimating it)
		for (i,j) in dist:		
			scale = dist[(i,j)]/dist_aster[(i,j)]
			error_sc+=scale			
			n+=1.
		n-=1.
	
	error_sc/=n
	error_tr = 0
	error_rot = 0	
	n = 0.
	scale = -1

	#check the error for translation and rotation
	for (i,j) in p_n:	
		if scale == -1:
			scale = np.linalg.norm(p_n_a[(i,j)])/np.linalg.norm(p_n[(i,j)])		
		p_n[(i,j)] = p_n[(i,j)]*scale
		error_tr += np.linalg.norm(p_n[(i,j)]-p_n_a[(i,j)])		
		if R != None:
			arg =  (np.trace(R[(i,j)].dot(R_aster[(i,j)].T))-1.)/2.			
			error_rot += abs(acos(clip(arg,-1.,1.)))
		n+=1.
	
	#return error
	return error_tr/n,error_rot/n, error_sc

"""
	function: get_error_rot
	description: computes the relative orientation between two cameras, we desired
		to be 0. This is used for Epipolar Visual Servo scheme
	params: 
		n: the amount of cameras
		cameras: list PlanarCameras
		poses: the poses of the cameras
		L: Laplacian
	returns: 
		error: the average error between orientations.
"""
def get_error_rot(n,cameras,poses,L):
	error = 0.0
	c = 0.0	
	#check the error
	for i in range(n):
		for j in range(n):			
			if L[i][j] == 1 and i!=j:
				Ri = cameras[i].euler_to_rotmat(0,poses[i][4],0)
				Rj = cameras[i].euler_to_rotmat(0,poses[j][4],0)
				Rij = Ri.T.dot(Rj)	
				error += abs(Rodrigues(Rij)[1])
				c+=1.0				
	
	#return average error
	return error/c


"""
	function: distances
	description: computes de distance between every camera pair in the formation
		and saves it in a dictionary
	params:
		n: amount of cameras
		cameras: cameras to compute distance
	returns:
		dist: a dict with distances between every pair of cameras
"""
def distances(n,cameras):
	dist = {}
	for i in range(n):
		for j in range(i+1,n):
			dist[(i,j)] = np.linalg.norm(cameras[j].t-cameras[i].t)			
	return dist

"""
	function: transform_to_frame
	description: transforms the given relatives positions p to the Rf reference frame
	params:
		Rf: Reference frame (rotation matrix 3x3)
		p: relative positions to transform (dict)
		cameras: cameras of the formation (list of PlanarCameras)
	returns:
		p_n: transformed p to Rf (dict).
"""
def transform_to_frame(Rf,p,cameras):
	#transform to world frame	
	p_n = {}
	for (i,j) in p:
		p_n[(i,j)] = np.dot(cameras[i].R,p[(i,j)])
		p_n[(i,j)] = np.dot(Rf.T,p_n[(i,j)])

	return p_n 

"""
	function: filter_error
	description: filters the error using the average of a window of iterations
	params:
		e: actual error
		errors: previous error (list)
		window: iterations to take in count
	returns:
		e: new error filtered
		errors: the new list to take in count for next iteration
"""		
def filter_error(e,errors,window):
	if len(errors) == window:
		errors.pop(0)
		errors.append(e)
	else:
		errors.append(e)

	return np.average(errors), errors

