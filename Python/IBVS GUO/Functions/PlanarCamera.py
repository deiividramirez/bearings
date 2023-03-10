"""
	Monday May 21st, 21:46:59 2018
	@author: robotics lab (Patricia Tavares & Gerardo Rodriguez)
	@email: patricia.tavares@cimat.mx, gerardo.rodriguez@cimat.mx
	version: 1.0
	This code implements a camera.
"""

"""
	Implementation of a simple camera.
"""
import numpy as np
from math import cos, sin, pi,sqrt
from Utils import Rotations
from Utils import Arrow3D

class PlanarCamera:
	
	#object constructor
	def __init__(self):

		#Image properties
		self.height = 480
		self.width = 640

		#Intrinsic camera parameters
		self.focal = 0.002
		self.rho = np.array([1.e-5,1.e-5]) #pixel dimensions
		self.pp = np.array([self.width/2,self.height/2]) #principal point
		self.K = np.array([[self.focal/self.rho[0], 0.0, self.pp[0]],
                        [0.0, self.focal/self.rho[1],self.pp[1]],
                        [0.0, 0.0, 1.0]]) #calibration matrix

		#Extrinsic camera parameter		
		self.R = None #rotation
		self.t = None #traslation
		self.P = None

		#noise
		self.noise = False
		self.sigma_noise = 0.
	
	"""
		function: set_position
		description: places the camera in a new position
		params:
			x: x camera position in world frame
			y: y camera position in world frame
			z: z camera position in world frame
	"""	
	def set_position(self,x,y,z,rot_x,rot_y,rot_z):		
		self.t = np.array([x,y,z])
		self.R = self.euler_to_rotmat(rot_x,rot_y,rot_z)
		self.P = np.dot(self.K,np.concatenate((self.R, np.dot(-self.R,self.t).reshape(3,1)),axis=1))
		
	"""
		function: set_noise
		description: set noise to image or turns it off
		params:
			sigma: sigma from a normal distribution of noise with mean 0.
				if we dont want noise, None is given
	"""
	def set_noise(self,sigma):
		if sigma is None:
			self.noise = False
			self.sigma = 0.
		else:
			self.noise = True
			self.sigma_noise = sigma

	"""
		function: projection
		description: projects a 3D points cloud into the camera using the 
				left upper corner of the image as origin.
		params:			
			Xs: Point cloud vstack([x,y,z]) 3xn
			n: amount of points
		returns:
			points: projected points. (np array 2 x n)
	"""
	def projection(self, Xs, n):		
		#to save the results
		points = np.zeros((2,n))		
		#project every point
		for i in range(n):
			#get the 3D point in projective coordinates X = [x,y,z,1]
			X = np.array([Xs[0][i],Xs[1][i],Xs[2][i],1.0])
			x = np.dot(self.P,X)
			points[0][i]= x[0]/x[2] #x
			points[1][i]= x[1]/x[2] #y
		
		if self.noise:
			noise_matrix = np.random.normal(0.,sqrt(self.sigma_noise),size=points.shape)
			points+=noise_matrix	
			
		#return the projected points
		return points

	"""
		function: projection_centering
		description: projects a 3D points cloud into the camera using the center
			of the image as the origin.
		params:			
			Xs: Point cloud vstack([x,y,z]) 3xn
			n: amount of points
		returns:
			points: projected points. (np array 2 x n)
	"""
	def projection_centering(self, Xs, n):		
		#to save the results
		points = np.zeros((2,n))
				
		#project every point
		for i in range(n):
			#get the 3D point in projective coordinates X = [x,y,z,1]
			X = np.array([Xs[0][i],Xs[1][i],Xs[2][i],1.0])
			x = np.dot(self.P,X)
			points[0][i]= x[0]/x[2] - self.pp[0] #x
			points[1][i]= self.pp[1] - x[1]/x[2] #y
		
		if self.noise:
			noise_matrix = np.random.normal(0.,sqrt(self.sigma_noise),size=points.shape)
			points+=noise_matrix
		
		#return the projected points
		return points

	"""
		function: euler_to_rotmat
		description: transforms euler angles to a rotation matrix containing them.
		params:
			rot_x : rotation in x axis
			rot_y : rotation in y axis
			rot_z : rotation in z axis
		returns:
			R: rotation matrix
	"""
	def euler_to_rotmat(self,rot_x,rot_y,rot_z):	
		Rx = np.array([[1.0,0.0,0.0],[0.0,cos(rot_x),-sin(rot_x)],[0.0,sin(rot_x),cos(rot_x)]])
		Ry = np.array([[cos(rot_y),0.0,sin(rot_y)],[0.0,1.0,0.0],[-sin(rot_y),0.0,cos(rot_y)]])
		Rz = np.array([[cos(rot_z),-sin(rot_z),0.0],[sin(rot_z),cos(rot_z),0.0],[0.0,0.0,1.0]])
		R = np.dot(Rz,np.dot(Ry,Rx))
	
		return R

	"""
		function: draw_camera
		description: draws this camera actual position
		params:
			ax: matplotlib component
			color: color of the camera
			scale: scale of the camera
	"""
	def draw_camera(self, ax, color='cyan', scale=1.0,linestyle='solid',alpha=0.):
		#CAmera points: to be expressed in the camera frame;
		CAMup=scale*np.array([[-1,-1,  1, 1, 1.5,-1.5,-1, 1 ],[ 1, 1,  1, 1, 1.5, 1.5, 1, 1 ],[ 2,-2, -2, 2,   3,   3, 2, 2 ],])
		Ri2w    = np.dot(Rotations.rotox(pi), self.R)
		trasl   = self.t.reshape(3, -1)
		CAMupTRASF = Ri2w.dot(CAMup) + trasl;
		CAMdwn=scale*np.array([[-1,-1,  1, 1, 1.5,-1.5,-1, 1  ],[ -1,-1, -1,-1,-1.5,-1.5,-1,-1 ],[  2,-2, -2, 2,   3,   3, 2, 2 ]])
		CAMdwnTRASF     = Ri2w.dot( CAMdwn ) + trasl
		CAMupTRASFm     = CAMupTRASF
		CAMdwnTRASFm    = CAMdwnTRASF
		ax.plot(CAMupTRASFm[0,:],CAMupTRASFm[1,:],CAMupTRASFm[2,:],c=color,ls=linestyle)
		ax.plot(CAMdwnTRASFm[0,:],CAMdwnTRASFm[1,:],CAMdwnTRASFm[2,:],c=color,ls=linestyle)
		ax.plot([CAMupTRASFm[0,0],CAMdwnTRASFm[0,0]],[CAMupTRASFm[1,0],CAMdwnTRASFm[1,0]],[CAMupTRASFm[2,0],CAMdwnTRASFm[2,0]],c=color,ls=linestyle)
		ax.plot([CAMupTRASFm[0,1],CAMdwnTRASFm[0,1]],[CAMupTRASFm[1,1],CAMdwnTRASFm[1,1]],[CAMupTRASFm[2,1],CAMdwnTRASFm[2,1]],c=color,ls=linestyle)
		ax.plot([CAMupTRASFm[0,2],CAMdwnTRASFm[0,2]],[CAMupTRASFm[1,2],CAMdwnTRASFm[1,2]],[CAMupTRASFm[2,2],CAMdwnTRASFm[2,2]],c=color,ls=linestyle)
		ax.plot([CAMupTRASFm[0,3],CAMdwnTRASFm[0,3]],[CAMupTRASFm[1,3],CAMdwnTRASFm[1,3]],[CAMupTRASFm[2,3],CAMdwnTRASFm[2,3]],c=color,ls=linestyle)
		ax.plot([CAMupTRASFm[0,4],CAMdwnTRASFm[0,4]],[CAMupTRASFm[1,4],CAMdwnTRASFm[1,4]],[CAMupTRASFm[2,4],CAMdwnTRASFm[2,4]],c=color,ls=linestyle)
		ax.plot([CAMupTRASFm[0,5],CAMdwnTRASFm[0,5]],[CAMupTRASFm[1,5],CAMdwnTRASFm[1,5]],[CAMupTRASFm[2,5],CAMdwnTRASFm[2,5]],c=color,ls=linestyle)

	"""
		function: draw_frame
		description: draws this camera frame
		params:
			ax: matplotlib component
			c: color of the frame
			scale: scale of the frame
			_scale_x: scale for x axis
			_scale_y: scale for y axis
			_scale_z: scale for z axis
	"""
	def draw_frame(self, ax, c='red', scale=2.0, _scale_x=None, _scale_y=None, _scale_z=None):
		if ax is None:
			ax = plt.gca()
		scale_x = _scale_x
		scale_y = _scale_y
		scale_z = _scale_z			
		if scale_x is None:
			scale_x = scale
		if scale_y is None:
			scale_y = scale
		if scale_z is None:
			scale_z = scale
		R = np.dot(Rotations.rotox(pi), self.R)
		t = self.t.reshape(3, -1)
		# Camera reference frame
		Oc = scale   * np.array([[0.,0,0]]).T
		Xc = scale_x * np.array([[1.,0,0]]).T
		Yc = scale_y * np.array([[0.,1,0]]).T
		Zc = scale_z * np.array([[0.,0,1]]).T
		Ri2w    = R;
		Oc1     = Ri2w.dot(Oc) + t
		Xc1     = Ri2w.dot(Xc) + t
		Yc1     = Ri2w.dot(Yc) + t
		Zc1     = Ri2w.dot(Zc) + t
		a1 = Arrow3D.Arrow3D([Oc1[0,0],Xc1[0,0]],[Oc1[1,0],Xc1[1,0]],[Oc1[2,0],Xc1[2,0]], mutation_scale=20, lw=1, arrowstyle="-|>", color=c)
		a2 = Arrow3D.Arrow3D([Oc1[0,0],Yc1[0,0]],[Oc1[1,0],Yc1[1,0]],[Oc1[2,0],Yc1[2,0]], mutation_scale=20, lw=1, arrowstyle="-|>", color=c)
		a3 = Arrow3D.Arrow3D([Oc1[0,0],Zc1[0,0]],[Oc1[1,0],Zc1[1,0]],[Oc1[2,0],Zc1[2,0]], mutation_scale=20, lw=1, arrowstyle="-|>", color=c)
		ax.add_artist(a1)
		ax.add_artist(a2)
		ax.add_artist(a3)
		ax.text(Xc1[0,0], Xc1[1,0], Xc1[2,0], (r'$X_{cam}$'))
		ax.text(Yc1[0,0], Yc1[1,0], Yc1[2,0], (r'$Y_{cam}$'))
		ax.text(Zc1[0,0], Zc1[1,0], Zc1[2,0], (r'$Z_{cam}$'))
