"""
	Monday September 24th, 21:14:00 2018
	@author: robotics lab (Patricia Tavares)
	@email: patricia.tavares@cimat.mx
	version: 2.0
	This code contains generic functions needed:
		- clip
		- closer_value
		- unitize
"""

from math import sqrt
import numpy as np

"""
	function: clip
	description: limits a value between min and max bounds
	params: 
		val: value to limit
		min: lower bound
		max: upper bound
	returns
		a value between min and max.
"""
def clip(val,min,max):
	return min if val < min else max if val > max else val

"""
	function: closer_value
	description: checks which element is closer to the desired element
	and returns it
	params:
		desired: value we set as desired
		elements: list of elements to compare
	returns:
		val: value closest to desired
		
"""
def closer_element(desired,elements):	
	min_dif = 10e5
	val = desired

	for element in elements:		 
		if np.linalg.norm(element-desired) < min_dif:
			val = element
			min_dif = np.linalg.norm(element-desired)

	return val

"""
	funtion: unitize
	description: normalices a vector with two components
	params:
		x: x component of the vector
		y: y component of the vector
	return:
		l: normalized x component
		m: normalized y component
"""
def unitize(x,y):
    l = x/(sqrt(x**2+y**2)) 
    m = y/(sqrt(x**2+y**2))
    return l,m
