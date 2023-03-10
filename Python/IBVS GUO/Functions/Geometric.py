"""
	Monday September 24th, 22:24:10 2018
	@author: robotics lab (Patricia Tavares)
	@email: patricia.tavares@cimat.mx
	version: 3.0
	This code contains functions relative to homography decomposition, triangulation,
	rotation matrix, geomtric constraints, etc.
		- is_rotmat
		- Rodrigues
		- euler_to_rotmat
		- H_from_points
		- scale_estimation_essential
		- triangulate
		- homog_to_rt
		- decompose_E
		- rigidity_function
		- RMF
		- Jacobian
		- move_wf
"""

import numpy as np
from math import sqrt,cos,sin
from Functions._Aux import unitize, closer_element

"""
		function: is_rotmat
		description: Verifies if the given matrix is a rotation matrix. It 
			is true if the R^T R is almost the Identity.
		params: 
			R: rotation matrix
		returns: 
			true: if the R^T T product is almost identity.		
"""	
def is_rotmat(R):		
	product=np.dot(R.T, R)
	I=np.eye(3)	
	diff=np.linalg.norm(I-product)	
	return diff < 1e-6

"""
		function: Rodrigues
		description: Checks if the given matrix is a rotation matrix and then transform it to
			a vector.
		params:
			R: rotation matrix
		returns:
			x: rotation in x
			y: rotation in y
			z: rotation in z
"""	
def Rodrigues(R):
	if((0.5*(np.trace(R)-1))  > 1.):
		theta = np.arccos(1.0)
	elif((0.5*(np.trace(R)-1))  < -1.):
		theta = np.arccos(-1.0)
	else:
		theta = np.arccos(0.5*(np.trace(R)-1))
	
	if(theta==0.):
		return np.array([0.,0.,0.]).T
	else:
		coef = (0.5*theta)/(np.sin(theta) )
		u = coef*np.array([R[2,1]-R[1,2],-R[2,0]+R[0,2],R[1,0]-R[0,1]])
		return u

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
def euler_to_rotmat(rot_x,rot_y,rot_z):
	
	Rx = np.array([[cos(rot_z),-sin(rot_z),0.0],[sin(rot_z),cos(rot_z),0.0],[0.0,0.0,1.0]])
	Ry = np.array([[cos(rot_y),0.0,sin(rot_y)],[0.0,1.0,0.0],[-sin(rot_y),0.0,cos(rot_y)]])
	Rz = np.array([[1.0,0.0,0.0],[0.0,cos(rot_x),-sin(rot_x)],[0.0,sin(rot_x),cos(rot_x)]])
	R = np.dot(Rz,np.dot(Ry,Rx))
	
	return R

""" 
	function: H_from_points
	description: Find homography H, such that fp is mapped to tp using the 
		linear DLT method. Points are conditioned automatically. 
	params:
		fp: points image 1
		tp: points image 2
	returns:
		H: Homography
"""
def H_from_points(fp,tp,normalized=True):        
    if fp.shape != tp.shape:
        raise RuntimeError('number of points do not match')
        
    #     condition points (important for numerical reasons)
        #     --from points--
    m = np.mean(fp[:2], axis=1)
    maxstd = max(np.std(fp[:2], axis=1)) + 1e-9
    C1 = np.diag([1/maxstd, 1/maxstd, 1]) 
    C1[0][2] = -m[0]/maxstd
    C1[1][2] = -m[1]/maxstd
    leng  =len(fp[0,:])
    fp = np.vstack([fp[0,:],fp[1,:],np.ones(leng)])
    fp = np.dot(C1,fp)
    
    # --to points--
    m = np.mean(tp[:2], axis=1)
    maxstd = max(np.std(tp[:2], axis=1)) + 1e-9
    C2 = np.diag([1/maxstd, 1/maxstd, 1])
    C2[0][2] = -m[0]/maxstd
    C2[1][2] = -m[1]/maxstd
    tp = np.vstack([tp[0,:],tp[1,:],np.ones(leng)])
    tp = np.dot(C2,tp)
    
    # create matrix for linear method, 2 rows for each correspondence pair
    nbr_correspondences = fp.shape[1]
    A = np.zeros((2*nbr_correspondences,9))
    for i in range(nbr_correspondences):        
        A[2*i] = [-fp[0][i],-fp[1][i],-1,0,0,0,
                    tp[0][i]*fp[0][i],tp[0][i]*fp[1][i],tp[0][i]]
        A[2*i+1] = [0,0,0,-fp[0][i],-fp[1][i],-1,
                    tp[1][i]*fp[0][i],tp[1][i]*fp[1][i],tp[1][i]]
    
    U,S,V = np.linalg.svd(A)
    H = V[8].reshape((3,3))
    
    # decondition
    H = np.dot(np.linalg.inv(C2),np.dot(H,C1))
    
    # normalize and return
    if normalized:
        return H/H[2,2]

    return H

"""
	function: scale_estimation_essential
	description: reconstructs a point and recovers depth of the reconstructed 3D point
	params:
		Ki: Calibration matrix from camera i (3x3)
		Kj: Calibration matrix from camera j (3x3)		
		qi: image point to reconstruct seen from camera i and in homogeneous coords
		qi: image point to reconstruct seen from camera j and in homogeneous coords
		yaw: relative rotation in z between agent i and j
		tij: relative translation between agent i and j
	returns:
		s: depth of reconstructed 3D point
		r_e: reconstruction error for this pair of P and points
"""
def scale_estimation_essential(Ki,Kj,qi,qj,yaw,tij):
	Rz = np.array([[cos(yaw),-sin(yaw),0.0],[sin(yaw),cos(yaw),0.0],[0.0,0.0,1.0]])#rotation matrix						
	Pj = np.dot(Kj,np.concatenate((Rz.T,-np.dot(Rz.T,tij).reshape((3,1))),axis=1))#camera matrix
	Pi = np.dot(Ki,np.eye(3,4))#camera matrix	
	rec = triangulate(Pi,Pj,qi,qj)										
	s = -rec[2] #due camera framework
	##############for error
	xi = np.dot(Pi,rec)
	xi /=xi[2]
	xj = np.dot(Pj,rec)
	xj /=xj[2]
	r_e = sqrt((xi[0]-qi[0])*(xi[0]-qi[0])+(xi[1]-qi[1])*(xi[1]-qi[1]))
	r_e += sqrt((xj[0]-qj[0])*(xj[0]-qj[0])+(xj[1]-qj[1])*(xj[1]-qj[1]))	
	return s, r_e/2.0	

"""
	function: triangulate
	description: uses the camera matrix of two cameras and a point in common to 
	reconstruct a 3D point. It uses differents metods seen in Multiple View Geometry (p.312)
	params:
		P1: camera matrix of the first camera. P1 = K1 [I | 0]
		P2: camera matrix of the second camera. P2 = K2 [R | t]
		p1: point coordinate in image 1 (x,y) in pixels.
		p2: point coordinate in image 2 (x,y) in pixels.
		method: a string with the name of the method to use: 'DLT' (p.312)
	returns:
		point3D: The reconstructed point from triangulation method. (np.array([x,y,z,1]))
"""
def triangulate(P1,P2,p1,p2):
	#creating matrix for AX = 0
	A = []
	A.append(p1[0]*P1[2]-P1[0])
	A.append(p1[1]*P1[2]-P1[1])
	A.append(p2[0]*P2[2]-P2[0])
	A.append(p2[1]*P2[2]-P2[1])
	A = np.array(A)

	u,s,vt = np.linalg.svd(A)
	point3D = vt[3]		
	point3D = point3D / point3D[3]	

	return point3D	

"""
	funtion: homog_to_rt
	description: transforms a homography to a relative rotation and traslation
	between the cameras that computed the homography. Uses triggs appendix 1 method.
		params: homography
	returns: 
		R: rotation matrix with relative rotation
		t: relative traslation vector up to scale
		n: normal vector
"""
def homog_to_rt(H):
	U, S, V = np.linalg.svd(H, full_matrices=True)
	s1      = S[0]/S[1]
	s3      = S[2]/S[1]
	zeta    = s1-s3
	a1      =  sqrt(1-s3**2)
	b1      =  sqrt(s1**2-1)
	a,b     = unitize(a1,b1)
	c,d     = unitize( 1+s1*s3, a1*b1 )
	e,f     = unitize( -b/s1, -a/s3 )
	v1      = V[0,:] # V es la transpuesta de la que regresa matlab
	#v1      = V[:,0]
	v3      = V[2,:]
	#v3      = V[:,2]
	n1      = b*v1-a*v3
	n2      = b*v1+a*v3
	R1      = U.dot(np.array([[c,0,d], [0,1,0], [-d,0,c]]).dot(V) )
	R2      = U.dot(np.array([[c,0,-d], [0,1,0], [d,0,c]]).dot(V) )
	t1      = e*v1+f*v3
	t2      = e*v1-f*v3
	if (n1[2]<0): 
		t1 = -t1
		n1 = -n1
	if (n2[2]<0): 
		t2 = -t2
		n2 = -n2	
	if (n1[2]>n2[2]):
		R = R1.T	
		t = zeta*t1
		n = n1
		return R,t,n
	else:
		R = R2.T
		t = zeta*t2
		n = n2
		return R,t,n

"""
	function: decompose_E
	description: decomposes essential matrix
	params: 
		E: essential matrix
		pf: position filter to use for choosing the right decomposition
		Rf: rotation filter for choosing the right rotation
	returns:
		tij: estimated relative translation
		yaw: estimated relative rotation in z axis		
"""
def decompose_E(E, pf, Rf):
	u,s,vt = np.linalg.svd(E)
	tij = u[:,2]
	W = np.array([[0.,-1.,0.],[1.,0.,0.],[0.,0.,1.]])
	Ra = np.dot(u,np.dot(W,vt))
	Rb = np.dot(u,np.dot(W.T,vt))						
	#choose the right decomposition using filter
	#we would need to project a 3D point if this filter doesnt exist		
	yaw = closer_element(Rodrigues(Rf)[2],[Rodrigues(Ra)[2],Rodrigues(Rb)[2],Rodrigues(-Ra)[2],Rodrigues(-Rb)[2]])
	#choose the right traslation
	if np.linalg.norm(pf+tij) < np.linalg.norm(pf-tij):
					tij = -tij
	return tij, yaw

"""
	function: rigidity_function
	description: computes the rigidity function from the given relative positions
	params:
		positions: dict with the relative positions
	returns:
		f: rigidity function vector size 3|E|
"""
def rigidity_function(positions):
	f = []	
	for (i,j) in positions:
		for l in range(3):
			f.append(positions[(i,j)][l])

	return np.array(f)

"""
	function: RMF
	description: computes rigidity matrix as Fabrizio's Tesis
	params:
		f: rigidity function
		n: amount of agents
		L: Laplacian
		cameras: list of Planar Cameras
		p: estimated relative positions (dict)
		R: estimated relative rotatiosn (dict)
	returns:
		J: rigidity matrix size 3|E| x 4n 
"""
def RMF(f,n,L,cameras,p,R):
	rows = 0
	J = np.zeros((f.shape[0],4*n))
	S = np.array([[0.,-1.,0.],[1.,0.,0.],[0.,0.,0.]])
	for i in range(n):
		for j in range(n):
			if L[i][j]==1:
				Bij = cameras[i].R.T.dot(cameras[j].t-cameras[i].t)
				dij = np.linalg.norm(Bij)
				Bij = p[(i,j)]
				Pij = np.eye(3)-np.outer(Bij,Bij)
				Rij = R[(i,j)]
				i_block = (-1.0/dij)*Pij
				j_block = (1.0/dij)*(Pij.dot(Rij))
				l_block = -S.dot(Bij)

				for k in range(3):
					J[rows+k][3*n+i] = l_block[k]
					for l in range(3):
						J[rows+k][i*3+l] = i_block[k][l]
						J[rows+k][j*3+l] = j_block[k][l]

				rows+=3
	return J

"""
	function: RMF
	description: omputes rigidity matrix as in EMRigidity script.
	params:
		f: rigidity function
		n: amount of agents
		L: Laplacian
		cameras: list of Planar Cameras
		p: estimated relative positions (dict)
		R: estimated relative rotatiosn (dict)
	returns:
		J: rigidity matrix size 3|E| x 4n 
"""
def Jacobian(f,n,L,cameras,p,R):
	rows = 0
	E = f.shape[0]/3 #number of relations |E|
	J = np.zeros((4*E,4*n))
	S = np.array([[0.,-1.,0.],[1.,0.,0.],[0.,0.,0.]])
	for i in range(n):
		for j in range(n):
			if L[i][j]==1:
				Bij = cameras[i].R.T.dot(cameras[j].t-cameras[i].t)
				dij = np.linalg.norm(Bij)
				Bij = p[(i,j)]
				Pij = np.eye(3)-np.outer(Bij,Bij)
				Rij = R[(i,j)]
				i_block = (-1.0/dij)*Pij
				j_block = (1.0/dij)*(Pij.dot(Rij))
				l_block = -S.dot(Bij)

				for k in range(3):
					J[rows+k][3*n+i] = l_block[k]
					for l in range(3):
						J[rows+k][i*3+l] = i_block[k][l]
						J[rows+k][j*3+l] = j_block[k][l]

				J[rows+3][3*n+i] = -1.
				J[rows+3][3*n+j] = 1.
				rows+=4
	return J

"""
	function: move_wf
	description: it takes de velocities and obtains the changes in the poses of 
		every camera in the world frame.
	params: 
		n: how many cameras do you have?
		cameras: a list of cameras (PlanarCamera)
		v: velocities of every camera, its shape is (n,3) where every 
			row indicates de velocities for the i-th camera. Every element of the 
			row indicates the velocity on the x,y,z axis.
		w: angular velocities of every camera, its shape is (n,3) where every 
			row indicates de velocities for the i-th camera. Every element of the 
			row indicates the velocity on the x,y,z axis.
		dt: time elapsed
	returns: 
		changes: the new position for the camera in x,y,z axis for the i-th
			camera is [i][:3], the new pose in roll, pitch and yaw for the i-th 
			camera is [i][3:]
"""
def move_wf(n,cameras,v,w,dt):
	#array to save positions
	poses = np.zeros((n,6))
	#for every camera
	for i in range(n):
			#get the change in the position
			p_d = cameras[i].R.dot(v[i])
			#get the change in the rotation
			S = np.array([[0,-w[i][2],w[i][1]],[w[i][2],0,-w[i][0]],[-w[i][1],w[i][0],0]])			
			#compute the new poses			
			R_d = cameras[i].R.dot(S)
			#move
			poses[i,:3] = cameras[i].t + p_d*dt
			poses[i,3:] = Rodrigues(cameras[i].R + R_d*dt)

	#return poses
	return poses
