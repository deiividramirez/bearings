"""
	Wednesday July 18th, 23:43:12 2018
	@author: robotics lab (Patricia Tavares)
	@email: patricia.tavares@cimat.mx
	version: 1.0
	This code contains generic functions needed:
		- configure_plot
		- plot_cameras
		- plot_all_cameras
		- plot_quarter
"""
import matplotlib.pyplot as plt
from math import pi
import numpy as np
from Functions.Geometric import Rodrigues
from matplotlib import colors as mcolors
from random import  seed, sample
import shutil, os
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
import numpy as np

"""
	function: configure_plot
	description: configures de figure to make subplots.
	params:
		n: rows of the grid plot
		m: columns of the grid plot
		fig_w: figure width
		fig_h: figure oheight
		title: title of the figure
	returns:
		ax: fig with subplot (see matplotlib docs)
		fig: configured figure
"""
def configure_plot(n,m,fig_w,fig_h,title):
	fig = plt.figure(figsize=(fig_h,fig_w))
	fig.suptitle(title)
	ax = fig.add_subplot(n,m,1,projection = '3d')
	ax = fig.gca()

	return ax,fig

"""
	function: plot_cameras
	description: it plots the cameras pased as list with the params given.
	params:
		cameras: list cameras to plot (PlanarCamera)
		ax: axis of the plt figure
		camera_scale: scale of the camera
		axis_scale: that, axis scale for ax used for the axis related to
			the camera
		cameras_color: color for the cameras, example: 'black','green','blue'..
			if you don't want to show cameras 'None'. It is a list and the ith color
			correspond to the ith camera
		frames_color: color for the frame of the cameras, example: 'black','green','blue'..
			if you don't want to show frames 'None' It is a list and the ith color
			correspond to the ith frame
		linestyle: The desired lynestile (optional9 defualts to solid.
"""
def plot_cameras(cameras,ax,camera_scale,axis_scale,cameras_color,frames_color,linestyle='solid'):
	i = 0
	for camera in cameras:
		if cameras_color != 'None':
			camera.draw_camera(ax,scale=camera_scale,color=cameras_color[i],linestyle=linestyle)
		if frames_color != 'None':
			camera.draw_frame(ax,scale=axis_scale,c=frames_color[i])
		i+=1

"""
	function: plot_all_cameras
	description: plots all the cameras given as param.
	params:
		ax: fig with subplot (see matplotlib docs)
		fig: configured figure
		n_cameras: cameras used.
		final: the end position of the cameras (list of PlanarCamera)
		desired: the desired position of the cameras (list of Planar Camera)
		init: initial positions of the cameras (list of PlanarCamera)
		final_poses: array with poses of final cameras (returned by function random cameras, and used in the process)
		desired_poses: array with the desired poses (returned by function line_formation/circle_formation)
		x: trayectory for every camera in x axis (n x k )
		y: trayectory for every camera in y axis (n x k)
		z: trayectory for every camera in z axis (n x k) where n=n_cameras and k=iterations made
		axis_scale: scale for the axis (frame) of every camera)
		camera_scale: scale to control de size of the camera in the subplot.
		bounds: the given bounds for the 3d graph [lower,upper], because we wanted it to be square
		so we can appreciate the formation.
	returns:
		ax: fig with subplot (see matplotlib docs)
		fig: configured figure
"""
def plot_all_cameras(ax,fig,n_cameras,final,desired,init,final_poses,desired_poses,x,y,z,axis_scale,camera_scale,bounds,legend=False):
	ax.set_xlim3d(bounds[0],bounds[1])
	ax.set_ylim3d(bounds[0],bounds[1])
	ax.set_zlim3d(bounds[0],bounds[1])
	ax.set_xlabel("$w_x (m)$")
	ax.set_ylabel("$w_y (m)$")
	ax.set_zlabel("$w_z (m)$")
	ax.grid(True)

	#for colors
	colors = ['r', 'g', 'b', 'y', 'c', 'm', 'maroon', 'tomato', 'limegreen', 'teal,', 'fuchsia', 'orangered', 'purple', 'crimson', 'seagreen', 'steelblue', 'sandybrown', 'olive', 'springgreen', 'darkorchid']

	#plot cameras
	plot_cameras(desired,ax,camera_scale,axis_scale,['plum']*n_cameras,'None')
	plot_cameras(final,ax,camera_scale,axis_scale,colors,'None')
	plot_cameras(init,ax,camera_scale,axis_scale,colors,'None',linestyle='dotted')

	#plot trayectories
	for i in range(n_cameras):
		ax.plot(x[i],y[i],z[i], label=str(i+1),color=colors[i])
		plt.grid(True)
		if legend:
			plt.legend(loc=2,prop={'size': 15})

	return ax,fig

"""
	function: plot_quarter
	description: plots a quarter of a 4x4 plot.
	params:
		ax: fig with subplot (see matplotlib docs)
		fig: configured figure
		place: corresponding place [2,2,X] with X as the number of the place
		x: the x data, list or numpy array [ite elements]
		y: the y data, numpy array [ite x n] where n can be the amount
			of data to represent. For example, if we have 100 iterations
			with the average velocity in x, y z, it will be [100 x 3]
			so, it will plot the x, y, z velocities along the 100 iterations.
		label: the label for the y data, if its more than one, it should be a list.
		ite: the number of iterations made.
		size: size of the label
	returns:
		ax: fig with subplot (see matplotlib docs)
		fig: configured figure
"""
def plot_quarter(ax,fig,place,x,y,label,ite,size,labels, location=1):
	ax = fig.add_subplot(place[0],place[1],place[2])
	plt.xlabel(labels[0])
	plt.ylabel(labels[1])

	#if we only have 1 data
	if y.shape == (ite,):
		ax.plot(x,y,label=label)
	else:
		#if we have more
		data = y.shape[1]
		for i in range(data):
			ax.plot(x,y[:,i],label=label[i])

	ax.grid(True)
	if size == None:
		plt.legend(loc=location)
	else:
		plt.legend(loc=location,prop={'size': size})
	return ax,fig

"""
	function: plot_3D.
	descriptions: plots 3D movement of cameras.
	params:
		xx: x coordinate of the world points
		yy: y coordinate of the world points
		zz: z coordinate of the world points
		n_cameras: amount of cameras
		x: trajectories of the agents in x
		y: trajectories of the agents in y
		z: trajectories of the agents in z
		init_cameras: initial cameras (list of Planar Camera)
		final_cameras: final cameras (list of Planar camera)
		desired_cameras: desired cameras (list of planar camera)
		final_poses: array with the poses related to final_cameras
		desired_poses: array with the poses related to desired_cameras
		ite: iterations
		t_arr: array with the timestamps
		spi: index of the selected point to triangulate
"""
def plot_3D(xx,yy,zz,n_cameras,x,y,z,init_cameras,final_cameras,desired_cameras,final_poses,desired_poses,ite,spi=-1):

	dirs = os.listdir('.')
	if ite == 0:
		if 'vid' in dirs:
			shutil.rmtree("vid")
		os.mkdir('vid')

	#plotting every section
	fig = plt.figure(figsize=(12,6))
	ax = fig.add_subplot(1,2,1,projection = '3d')
	ax = fig.gca()
	size  = 7
	for tick in ax.xaxis.get_major_ticks():
                tick.label.set_fontsize(size)
	for tick in ax.yaxis.get_major_ticks():
                tick.label.set_fontsize(size)
	for tick in ax.zaxis.get_major_ticks():
                tick.label.set_fontsize(size)
	bounds = [min(-1.0,np.min(x)-0.2,np.min(y)-0.2,np.min(z)-0.2),max(2.5,np.max(x)+0.2,np.max(y)+0.2,np.max(z)+0.2)]
	ax,fig = plot_all_cameras(ax,fig,n_cameras,final_cameras,desired_cameras,init_cameras,final_poses,desired_poses,x,y,z,0.5,0.09,bounds)
	ax.plot(xx,yy,zz,'o', alpha=0.2,color='k') #plot points
	if spi!=-1:
		ax.scatter(xx[spi],yy[spi],zz[spi],color='r')

	ax.view_init(90,-90)
	ax.dist = 7
	ax.axes.set_zticks([])
	ax.set_zlabel("")
	ax = fig.add_subplot(1,2,2,projection = '3d')
	ax = fig.gca()
	ax,fig = plot_all_cameras(ax,fig,n_cameras,final_cameras,desired_cameras,init_cameras,final_poses,desired_poses,x,y,z,0.5,0.09,bounds)
	ax.plot(xx,yy,zz,'o', alpha=0.2,color='k') #plot points
	if spi!=-1:
		ax.scatter(xx[spi],yy[spi],zz[spi],color='r')
	ax.view_init(0,90)
	for tick in ax.xaxis.get_major_ticks():
                tick.label.set_fontsize(size)
	for tick in ax.yaxis.get_major_ticks():
                tick.label.set_fontsize(size)
	for tick in ax.zaxis.get_major_ticks():
                tick.label.set_fontsize(size)
	fig.subplots_adjust(left=0, bottom=0, right=1, top=1, wspace=0, hspace=0)
	ax.dist = 7.5
	ax.axes.set_yticks([])
	ax.set_ylabel("")
	plt.savefig('vid/'+str(ite)+'.png',bbox_inches='tight')
	plt.close()

"""
	function: plot_all.
	descriptions: plots everything.
	params:
		title: title of the consensus
		xx: x coordinate of the world points
		yy: y coordinate of the world points
		zz: z coordinate of the world points
		n_cameras: amount of cameras
		x: trajectories of the agents in x
		y: trajectories of the agents in y
		z: trajectories of the agents in z
		init_cameras: initial cameras (list of Planar Camera)
		final_cameras: final cameras (list of Planar camera)
		desired_cameras: desired cameras (list of planar camera)
		final_poses: array with the poses related to final_cameras
		desired_poses: array with the poses related to desired_cameras
		ite: iterations
		t_arr: array with the timestamps
		scale: array with information about the scale
		v: average of the linear velocities
		w: average of the angular velocities
		err_t: evaluation error in translation
		err_psi: evaluation error in rotation
		dists: distances between agents measured from groundtruth
		eig: evolution of the sixth eigenvalue of rigidity matrix
		r_e: reconstruction eror over time
		depth: z coordinate from te reconstructed point over the time
		spi: index of the selected point to triangulate
"""
def plot_all(title,xx,yy,zz,n_cameras,x,y,z,init_cameras,final_cameras,desired_cameras,final_poses,desired_poses,ite,t_arr,scale,gamma,v,w,err_t,err_psi,dists,eig=[],r_e=[],depth={},spi=-1):
	#clearing

	dirs = os.listdir('.')
	if 'graphs' in dirs:
		shutil.rmtree("graphs")
	os.mkdir('graphs')

	#plotting everything together
	rows = 3
	cols = 2
	ax,fig = configure_plot(rows,cols,15,20,title)
	bounds =  [ min(np.min(x)-0.2,np.min(xx)-0.2,np.min(y)-0.2,np.min(yy)-0.2,np.min(z)-0.2,np.min(zz)-0.2), max(np.max(x)+0.2,np.max(xx)+0.2,np.max(y)+0.2,np.max(yy)+0.2,np.max(z)+0.2,np.max(zz)+0.2)]
	ax,fig = plot_all_cameras(ax,fig,n_cameras,final_cameras,desired_cameras,init_cameras,final_poses,desired_poses,x,y,z,0.5,0.09,bounds)
	ax.plot(xx,yy,zz,'o', alpha=0.2) #plot points
	if spi!=-1:
		ax.scatter(xx[spi],yy[spi],zz[spi],color='r')

	if len(gamma)==0:
		ax,fig = plot_quarter(ax,fig,[rows,cols,2],t_arr,scale,'$e^s$',ite,None,['Time $(s)$','$Scale$'],4)
	else:
		ax,fig = plot_quarter(ax,fig,[rows,cols,2],t_arr,gamma,'$\gamma(t)$',ite,None,['Time $(s)$','Altitude $(m)$'],4)
	ax,fig = plot_quarter(ax,fig,[rows,cols,3],t_arr,v,['$v_x$','$v_y$','$v_z$'],ite,None,['Time $(s)$','Average linear velocity $(m/s)$'])
	ax,fig = plot_quarter(ax,fig,[rows,cols,4],t_arr,w[:,2],'$\omega_z$',ite,None,['Time $(s)$','Average absolute angular velocity $(rad/s)$'])
	ax,fig = plot_quarter(ax,fig,[rows,cols,5],t_arr,err_t,'$e^t$',ite,None,['Time $(s)$','Translation error $(m)$'])
	ax,fig = plot_quarter(ax,fig,[rows,cols,6],t_arr,err_psi,'$e^\psi$',ite,None,['Time $(s)$','Rotation error $(rad)$'])
	plt.savefig('graphs/complete.pdf',bbox_inches='tight')
	plt.show()

	####################plotting every section
	ax,fig = configure_plot(1,1,6,6,"")
	fig.subplots_adjust(0,0,1,1)
	bounds =  [ min(np.min(x)-0.2,np.min(xx)-0.2,np.min(y)-0.2,np.min(yy)-0.2,np.min(z)-0.2,np.min(zz)-0.2), max(np.max(x)+0.2,np.max(xx)+0.2,np.max(y)+0.2,np.max(yy)+0.2,np.max(z)+0.2,np.max(zz)+0.2)]
	ax,fig = plot_all_cameras(ax,fig,n_cameras,final_cameras,desired_cameras,init_cameras,final_poses,desired_poses,x,y,z,0.5,0.09,bounds)
	ax.plot(xx,yy,zz,'o', alpha=0.2,color='k') #plot points
	if spi!=-1:
		ax.scatter(xx[spi],yy[spi],zz[spi],color='r')

	##########to print seen from upside
	size  = 10
	for tick in ax.xaxis.get_major_ticks():
                tick.label.set_fontsize(size)
	for tick in ax.yaxis.get_major_ticks():
                tick.label.set_fontsize(size)
	for tick in ax.zaxis.get_major_ticks():
                tick.label.set_fontsize(size)

	ts = ax.axes.get_zticks()
	ax.axes.set_zticks([])
	ax.set_zlabel("")
	ax.view_init(90,-90)
	plt.savefig('graphs/3d.pdf',bbox_inches='tight')
	#########to print seen from side
	ax.axes.set_zticks(ts)
	ax.set_zlabel("$w_z (m)$")
	ax.axes.set_yticks([])
	ax.set_ylabel("")
	ax.view_init(0,90)
	plt.savefig('graphs/3d1.pdf',bbox_inches='tight')
	"""for angle in range(0, 360):
	    ax.view_init(30, angle)
	    plt.draw()
	    plt.pause(.001)"""
	plt.close()

	plt.rcParams.update({'font.size': 20})
	######################SCALE
	plt.xlabel('Time $(s)$')
	plt.ylabel('$Scale$')
	plt.plot(t_arr,scale,label='$e^s$')
	plt.legend(loc=0)
	plt.grid(True)
	plt.savefig('graphs/scale.pdf',bbox_inches='tight')
	plt.close()
	######################Gamma
	if len(gamma) != 0:
		plt.xlabel('Time $(s)$')
		plt.ylabel('Altitude $(m)$')
		plt.plot(t_arr,gamma,label='$\gamma(t)$')
		plt.legend(loc=0)
		plt.grid(True)
		plt.savefig('graphs/gamma.pdf',bbox_inches='tight')
		plt.close()
	######################LINEAR VELOCITIES
	plt.xlabel('Time $(s)$')
	plt.ylabel('Average linear velocity $(m/s)$')
	labels = ['$v_x$','$v_y$','$v_z$']
	for i in range(3):
		plt.plot(t_arr,v[:,i],label=labels[i])
	plt.legend(loc=0)
	plt.grid(True)
	plt.savefig('graphs/v.pdf',bbox_inches='tight')
	plt.close()
	######################ANGULAR VELOCITIES
	plt.xlabel('Time $(s)$')
	plt.ylabel('Average absolute angular velocity $(rad/s)$')
	plt.plot(t_arr,w[:,2],label='$\omega_z$')
	plt.legend(loc=0)
	plt.grid(True)
	plt.savefig('graphs/w.pdf',bbox_inches='tight')
	plt.close()
	######################TRANSLATION ERROR
	plt.xlabel('Time $(s)$')
	plt.ylabel('Translation error $(m)$')
	plt.plot(t_arr,err_t,label='$e^t$')
	plt.legend(loc=0)
	plt.grid(True)
	plt.savefig('graphs/e_t.pdf',bbox_inches='tight')
	plt.close()
	######################ROTATION ERROR
	plt.xlabel('Time $(s)$')
	plt.ylabel('Rotation error $(rad)$')
	plt.plot(t_arr,err_psi,label='$e^\psi$')
	plt.legend(loc=0)
	plt.grid(True)
	plt.savefig('graphs/e_psi.pdf',bbox_inches='tight')
	plt.close()
	######################INTER AGENTS DISTANCES
	plt.xlabel('Time $(s)$')
	plt.ylabel('Distance inter-agents $(m)$')
	for (i,j) in dists:
		if i == j-1:
			plt.plot(t_arr,dists[(i,j)])#,label='$agents ('+str(i+1)+','+str(j+1)+')$')
	plt.plot(t_arr,dists[(0,n_cameras-1)])#,label='$agents ('+str(1)+','+str(n_cameras)+')$')
	plt.legend(loc=0)
	plt.grid(True)
	plt.savefig('graphs/distances.pdf',bbox_inches='tight')
	plt.close()

	########################SIXTH EIGENVALUE
	if len(eig)!=0:
		plt.xlabel('Time $(s)$')
		plt.ylabel('')
		plt.plot(t_arr,eig,label='$\lambda(J(q))_6$')
		plt.legend(loc=0)
		plt.grid(True)
		plt.savefig('graphs/eig.pdf',bbox_inches='tight')
		plt.close()
	########################average error reconstruction
	if len(r_e)!=0:
		plt.xlabel('Time $(s)$')
		plt.ylabel('Average reconstruction error $(pixels)$')
		plt.plot(t_arr,r_e,label='$e^r$')
		plt.legend(loc=0)
		plt.grid(True)
		plt.savefig('graphs/r_e.pdf',bbox_inches='tight')
		plt.close()
	#########################depth between agents and central point
	if len(depth)!=0:
		plt.xlabel('Time $(s)$')
		plt.ylabel('$Depth (m)$')
		for (i,j) in depth:
			plt.plot(t_arr,depth[(i,j)],label='$e_{'+str(i)+str(j)+'}$')
		plt.legend(loc=0)
		plt.grid(True)
		plt.savefig('graphs/depth.pdf',bbox_inches='tight')
		plt.close()

"""
	function: run_info
	description: writes to a file the information provided as parameter in a file named run_info.txt saved
		into de graphs folder.
	parameters:
		title: title of the executed consensus
		seed: seed used to initialize the positions
		ite: iterations made
		dt: frequency
		threshold: error threshold
		n_camaras: amount of cameras
		rank: rank of the rigidity matrix
		sixth: sixth eigenvalue of the rigiditiy matrix
		e_t: evaluation error in translation
		e_psi: evaluation error in rotation
		sd: standard deviation for noise
		L: Laplacian matrix
		p_aster: desired relative positions (dict)
		R-aster: desired relative rotations (dict)
		copy_poses: copy of the initial poses
		init_poses: final poses of the formation
		scale: final scale
"""
def run_info(title,seed_r,seed_p,ite,dt,threshold,n_cameras,rank,sixth,e_t,e_psi,sd,L,p_aster,R_aster,copy_poses,init_poses,scale):
	f = open("graphs/runinfo.txt", "w")
	f.write(title+'\n\n')
	f.write('seed: '+str(seed_r)+'\n')
	f.write('seed pixels: '+str(seed_p)+'\n')
	f.write('Iterations: '+str(ite)+'\n')
	f.write('dt: '+str(dt)+'\n')
	f.write('time: '+str(ite*dt)+'\n')
	f.write('scale: '+str(scale)+'\n')
	f.write('threshold: '+str(threshold)+'\n')
	f.write('n: '+str(n_cameras)+'\n')
	f.write('RMR: '+str(rank)+'\n')
	f.write('sixth eigen value: '+str(sixth)+'\n')
	f.write('e_t: '+str(e_t)+'\n')
	f.write('e_psi: '+str(e_psi)+'\n')
	f.write('sd noise: '+str(sd)+'\n\n')
	f.write('Laplacian\n')
	copy_poses = np.round(copy_poses,4)
	init_poses = np.round(init_poses,4)
	for i in range(n_cameras):
		for j in range(n_cameras):
			f.write(str(L[i][j])+' ')
		f.write('\n')
	f.write('\nDesired Relative Poses: \n\n')
	for (i,j) in p_aster:
		f.write(str(i)+','+str(j)+' ')
		p_aster[(i,j)] = np.round(p_aster[(i,j)],4)
		for l in range(3):
			f.write(str(p_aster[(i,j)][l])+' ')
		f.write(str(np.round(Rodrigues(R_aster[(i,j)])[2],4))+'\n')
	f.write('\nInitial Poses: \n\n')
	for i in range(n_cameras):
		for j in range(6):
			f.write(str(copy_poses[i][j])+' ')
		f.write('\n')
	f.write('\nFinal Poses: \n\n')
	for i in range(n_cameras):
		for j in range(6):
			f.write(str(init_poses[i][j])+' ')
		f.write('\n')
