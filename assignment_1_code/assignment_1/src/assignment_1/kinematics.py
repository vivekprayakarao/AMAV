import math
from math import hypot, pi
from wheeled_robot_kinematics.srv import *
from wheeled_robot_kinematics.msg import *
from assignment_1.kinematics import *

def forward(p, a, rd):

  (x,y,theta) = p
  (vl,vr,t) = a
  (axle_length, wheel_radius, max_speed) = rd

  dt = 0.1

  if vl == vr:
  	#print "Robot will move straight in this case"
  	#print "Case where vr = vl", vr, vl
  	x += (vl * math.cos(theta) * t) * wheel_radius
  	y += (vl * math.sin(theta) * t) * wheel_radius
  elif vl == (-vr):
  	#print "Robot will rotate at its own position"
  	theta += (2 * vr * t) / axle_length
  else:
  	#print "Else case"
  	R = (axle_length/2) * ( (vl+vr) / (vr-vl) )

  	w = ((vr-vl)/axle_length) * wheel_radius

  	for i in range(int(t/dt)):
  		icc_x = x - (R* math.sin(theta) )
  		icc_y = y + (R* math.cos(theta) )

  		new_x = ( ( math.cos(w*dt) ) * ( x - icc_x ) ) - ( ( math.sin(w*dt) ) * ( y - icc_y ) ) + icc_x
  		new_y = ( ( math.sin(w*dt) ) * ( x - icc_x ) ) + ( ( math.cos(w*dt) ) * ( y - icc_y ) ) + icc_y
  		new_theta = theta + ( w*dt )

  		#new_x = x + ( (new_x - x) / (wheel_radius*wheel_radius) )
  		#new_y = y + ( (new_y - y) / (wheel_radius*wheel_radius) )

  		#Update values for next iteration
  		x = new_x #* wheel_radius
  		y = new_y #* wheel_radius
  		theta = new_theta

  	# if (abs(theta)) > 6.28:
  	# 	print "theta is out of bound", theta
  	# 	if theta > 0:
  	# 	  	theta = theta%6.28
  	# 	else:
  	# 		theta = - (theta%6.28)
  	# 	print "New value of theta", theta

  return x, y, theta

def move_straight_line(p0,p1,rd):
	(x0,y0,theta0) = p0
	(x1,y1,theta1) = p1
	(axle_length, wheel_radius, max_speed) = rd

	vl = max_speed
	vr = max_speed
	distance = ( math.sqrt( math.pow((y1-y0),2) + math.pow((x1-x0),2) ) ) / wheel_radius
	t = distance / vl

	return vl,vr,t

def rotate(theta0,theta1,rd):
	(axle_length, wheel_radius, max_speed) = rd

	print "Time to rotate, theta0 & theta1: ", theta0, theta1

	vl = - (max_speed)
  	vr = max_speed
  	distance = (axle_length/2) * (theta1 - theta0)
  	t = distance / vr

  	return vl,vr,t

def inverse_solver(p0,p1,rd):

	(x0,y0,theta0) = p0
	(x1,y1,theta1) = p1
	(axle_length, wheel_radius, max_speed) = rd

	print "p0 is:" ,p0
	print "p1 is:" ,p1

	t = 0.1
  	solution_found = 0

  	R = (x1-x0)/ ( math.sin(theta1) - math.sin(theta0) )

  	print "R is:" , R 

  	while solution_found == 0:
  		
  		w = (theta1-theta0) / t 

  		vr = w * ( R + (axle_length/2) )
  		vl = w * ( R - (axle_length/2) )

  		#print "w, vl, vr, t: " ,w,vl,vr,t

  		(x_new,y_new,theta_new) = forward(p0, (vl,vr,t), rd)
  		#p_new = forward(p0, (vl,vr,t), rd)
  		#print "p_new:" ,p_new
  		#print "\n\n"

  		#if p1 == p_new:
  		if (abs(x_new-x1)) <= 0.05 and (abs(y_new-y1)) <= 0.05: # or (theta_new-theta1) <= 0.05:
  			#print "Solution found at t:", t 
  			solution_found = 1
  			if (abs(vl)) > max_speed or (abs(vr)) > max_speed:
  				#print "Too fast"
  				solution_found = 0
  				t += 0.1
  		elif t > 20:
  			print "Timeout"
  			solution_found = 1
  		else:
  			t += 0.1

  	return vl,vr,t


def inverse(p0, p1, rd):
  (x0,y0,theta0) = p0
  (x1,y1,theta1) = p1
  (axle_length, wheel_radius, max_speed) = rd

  print "I am in kinematics.py > inverse"

  vl = 0
  vr = 0
  t = 0

  resp = DiffDriveIKResponse()

  if p0 == p1:
  	print "No need to move"

  elif theta0 == theta1:
  	print "Action is straight line"
  	vl,vr,t = move_straight_line(p0,p1,rd)	

	if p1 == forward(p0,(vl,vr,t),rd):
		print "Solution correct"
		resp.actions.append( DiffDriveAction(vl,vr,t) )
	else:
		print "Solution incorrect"
		actual_theta1 = theta1
		#theta1 += 0.5
		#print "theta1 is now:" , theta1
		#print "p1 is now:" , p1
		vl,vr,t = inverse_solver(p0,(x1,y1,(theta1+1.57)),rd)
		resp.actions.append( DiffDriveAction(vl,vr,t) )
		vl,vr,t = rotate((theta1+1.57),theta1,rd)
		resp.actions.append( DiffDriveAction(vl,vr,t) )

  elif (x0,y0) == (x1,y1):
  	print "Rotate at same place"

  	vl, vr, t = rotate(theta0,theta1,rd)
  	resp.actions.append( DiffDriveAction(vl,vr,t) )

  else:
  	print "Do something else"

  	vl,vr,t = inverse_solver(p0,p1,rd)
  	resp.actions.append( DiffDriveAction(vl,vr,t) )

  return resp
  #return vl, vr, t