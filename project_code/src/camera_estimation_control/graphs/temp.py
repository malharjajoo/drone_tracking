import csv
import matplotlib.pyplot as plt
import numpy as np

from numpy.linalg import inv
from math import *
import cv2
import sys



NUM_CAMS = 3

summaryString_list = []


M_PI = 3.14159265358979323846


# Simple helper for plotting found Drone "regions" in subplots in a Figure
def plotFoundRegionsForEachSubplot(axs,region_list):
	for i in range(len(axs)):

		for j in range(len(region_list)):
			axs[i].axvspan(region_list[j][0], region_list[j][1], color='green', alpha=0.2)







class State:

	def __init__(self,x,y,z,roll,pitch,yaw):
		self.x = x
		self.y = y
		self.z = z
		self.roll = roll
		self.pitch = pitch
		self.yaw = yaw



class SummaryObj:

	#TODO: Add a member variable for latency measurement.
	def __init__(self, 
				timestamp,
				refState_list,
			    avg_refState, 
			    camid_list,
			    foundpose_per_camera,
			    leds_per_camera,
			    droneWorldstate_per_camera):

		self.timestamp = timestamp
		self.refState_list = refState_list
		self.avg_refState = avg_refState
		self.camid_list = camid_list
		self.foundpose_per_camera = foundpose_per_camera
		self.leds_per_camera = leds_per_camera
		self.droneWorldstate_per_camera = droneWorldstate_per_camera
		




def radianToDegree_(rad):
	global M_PI
	return rad*(180.0/M_PI)




def degreeToRadian_(degrees):
	global M_PI
	return degrees*(M_PI/180.0)




def degreeToRadian(rpy):
	rpy_rad = ( 
				degreeToRadian_(rpy[0]),
				degreeToRadian_(rpy[1]),
				degreeToRadian_(rpy[2])
			  )

	return rpy_rad


def getRotationMatrix(rpy_degrees):

	rpy_rad = degreeToRadian(rpy_degrees)

	roll  = rpy_rad[0]
	pitch = rpy_rad[1]
	yaw   = rpy_rad[2]

	R = np.zeros((3,3))
	R[0,0]= cos(yaw) * cos(pitch)
	R[0,1]= ( cos(yaw) * sin(pitch) * sin(roll) ) - ( cos(roll) * sin(yaw) )
	R[0,2]= ( sin(yaw) * sin(roll) ) + ( cos(yaw) * cos(roll) * sin(pitch) )

	R[1,0]= cos(pitch) * sin(yaw);
	R[1,1]= ( cos(yaw) * cos(roll) ) + ( sin(yaw) * sin(pitch) * sin(roll) )   
	R[1,2]= ( cos(roll) * sin(yaw) * sin(pitch) ) - ( cos(yaw) * sin(roll) ) 

	R[2,0]= -1 * sin(pitch);
	R[2,1]= cos(pitch) * sin(roll);
	R[2,2]= cos(pitch) * cos(roll);

	return R




def findOrientationError(ref_rpy_degrees, est_rpy_degrees):

	error = -1;

	R = getRotationMatrix(est_rpy_degrees)
	R_true = getRotationMatrix(ref_rpy_degrees)


	error_mat = np.matmul(R_true, R.transpose() )
	
	#print(error_mat)

	
	error_vec,J = cv2.Rodrigues(error_mat);
	
	#print(error_vec)
	
	# The norm of the openCV axis-angle representatio is the angle !
	dummy_vec = np.zeros((3,1))

	error = radianToDegree_( cv2.norm(error_vec,dummy_vec, cv2.NORM_L2) );

	error_method2 = np.rad2deg(np.arccos((np.trace(error_mat) - 1) / 2))


	#print("Error [radian] = " + str(error))
	#print("Error [degree] = " + str(radianToDegree_(error)))
	#print("Error method 2 [radian]= " + str(error_method2) )
	return error








# Plots whether the cameras found a pose or not.
def plotFoundPose(summaryList):



	# Number of (syncrhonized) frames
	x = []

	y_vals = [[] for i in range(NUM_CAMS)]

	# For each reading
	for frame_number in range(len(summaryList)):

		sumObj = summaryList[frame_number]

		# store found pose value for each camera
		for camid in sumObj.camid_list:
			y_vals[camid].append(sumObj.foundpose_per_camera[camid])


		x.append(frame_number)



	

	for i in range(NUM_CAMS):
		plt.plot(x, y_vals[i], label = 'cam'+str(i))


	# For each frame, find how many cameras have found a pose.
	number_of_poses_found_per_frame = []

	for i in range(len(x)):

		# for each entry/frame, we check how many cameras have found a pose.
		foundpose = 0
		for camid in range(len(y_vals)):
			foundpose = foundpose + y_vals[camid][i]

		number_of_poses_found_per_frame.append(foundpose)




	# 1) Regions with found pose value from at least 1 camera
	at_least_one_foundpose = [elem for elem in number_of_poses_found_per_frame if elem > 0]
	val = len(at_least_one_foundpose)/float(len(x))
	summaryString_list.append('% of Frames received for which at least 1 camera found a pose = '+str(val*100.0) )

	# 2) Regions with no overlap from any camera
	# Could have also done as 100 - (% of frames for which at least one cam found pose)
	noPoseFound = [elem for elem in number_of_poses_found_per_frame if elem == 0]
	val = len(noPoseFound)/float(len(x))
	summaryString_list.append('% of Frames received for which no camera found pose = '+str(val*100.0) )



	# 3) Regions with (overlap) value from only 1 camera (can be any camera)
	one_cam_only = [elem for elem in number_of_poses_found_per_frame if elem == 1]
	val = len(one_cam_only)/float(len(x))
	summaryString_list.append('% of Frames for which only 1 camera found pose = '+ str(val*100.0) )


	# 4) Region with overlap between only 2 cameras (can be any 2 cameras)
	if(len(y_vals) > 1):
		two_cams_only = [elem for elem in number_of_poses_found_per_frame if elem == 2]
		val = len(two_cams_only)/float(len(x))
		summaryString_list.append('% of Frames received for which (any) 2 cameras found pose = '+str(val*100.0) )


	# 5) Regions with overlap between all 3 cameras
	if(len(y_vals) > 2):
		all_three_cams = [elem for elem in number_of_poses_found_per_frame if elem == 3]
		val = len(all_three_cams)/float(len(x))
		summaryString_list.append('% of Frames received for which all 3 cameras found pose = '+str(val*100.0) )



	# TODO: Find out which camera needs to be positioned differntly ?
	# But this also depends on the trajectory of the drone.

	
	ax = plt.gca()
	plt.legend()

	ax.set_ylim([-1,2])
	plt.xlabel('frame_number')
	plt.ylabel('Found pose')
	plt.title('Pose found or not by each camera')










def findDistance(p1,p2):
	dist = -1.0
	#dist =  abs(p1[0] - p2[0]) + abs(p1[1] - p2[1]) + abs(p1[2] - p2[2])

	dist_sq =  	 ( 
				   pow( (p1[0] - p2[0]), 2 ) +
				   pow( (p1[1] - p2[1]), 2 ) + 
				   pow( (p1[2] - p2[2]), 2 ) 
				  ) 

	dist = dist_sq ** 0.5
	return dist


#nput is either x,y,z (or) roll,pitch,yaw.
def plotErrorforAllCombinations_pos(dof1_estimate, avg_ref_dof1_estimates, 
								   dof2_estimate, avg_ref_dof2_estimates,
								   dof3_estimate, avg_ref_dof3_estimates,
								   pose_found, 
								   frame_list, 
								   parameterTypeString,
								   plotcams):

	

	# Simple check for input data.
	assert(len(dof1_estimate[0]) == len(dof2_estimate[0]) == len(dof3_estimate[0]))
	assert(len(dof1_estimate[0]) == len(avg_ref_dof1_estimates) == len(avg_ref_dof2_estimates)==len(avg_ref_dof3_estimates))

	mean_error_list = []
	max_error_list = []
	min_error_list = []
	std_dev_list = []

	
	# Plot for 1 camera - 3 combinations
	combinations = [0,1,2]

	plot1cam = plotcams[0]

	for cam_index in combinations:


		error_list = []
		filtered_frame_list = []

		# Loop over each estimate
	
		for i in range(len( dof1_estimate[cam_index] )):

			if(pose_found[cam_index][i] == True):

				p_ref = (avg_ref_dof1_estimates[i], avg_ref_dof2_estimates[i], avg_ref_dof3_estimates[i])
				p_est = (dof1_estimate[cam_index][i], dof2_estimate[cam_index][i], dof3_estimate[cam_index][i])

				error = findDistance(p_ref, p_est)

				#error = abs(p_ref[0] -  p_est[0]) + abs(p_ref[1] -  p_est[1]) + abs(p_ref[2] -  p_est[2])
				#if( error > 0.3):
					#print("i=",i)
					#print("Reference estimate= ", p_ref)
					#print("My estimate = ", p_est)
					#print("\n")


				error_list.append(error)
				filtered_frame_list.append(frame_list[i])



		# Calculte various statistics
		[mean_error, max_error, min_error, sd] = findStats(error_list)

		mean_error_list.append(mean_error)
		max_error_list.append(max_error)
		min_error_list.append(min_error)
		std_dev_list.append(sd)


		if(plot1cam):
			plt.figure()

			plt.scatter(filtered_frame_list, error_list)

			plt.xlabel('Frame_number')
			plt.ylabel(parameterTypeString + 'Error (metres)')
			plt.grid()
			plt.title('Drone '+parameterTypeString+' Error of Camera'+str(cam_index+1)+' estimates')




	# Plot for 2 cameras - 3 combinations
	combinations = [(0,1),(1,2),(0,2)]
	plot2cam = plotcams[1]

	for (cam_index1, cam_index2) in combinations:

		error_list = []
		filtered_frame_list = []

		for i in range(len( dof1_estimate[cam_index1] )):
			
			# if pose is found in both cameras
			if(pose_found[cam_index1][i] == pose_found[cam_index2][i] == True):


				# error = ref - (avg of the 2 cameras)
				p_ref = (avg_ref_dof1_estimates[i], avg_ref_dof2_estimates[i], avg_ref_dof3_estimates[i])

				p_est = ( 
						  float(dof1_estimate[cam_index1][i] + dof1_estimate[cam_index2][i])/2.0, 
						  float(dof2_estimate[cam_index1][i] + dof2_estimate[cam_index2][i])/2.0,
						  float(dof3_estimate[cam_index1][i] + dof3_estimate[cam_index2][i])/2.0 
						 )

				error = findDistance(p_ref, p_est)
				#error = abs(p_ref[0] -  p_est[0]) + abs(p_ref[1] -  p_est[1]) + abs(p_ref[2] -  p_est[2])
			

				error_list.append(error)
				filtered_frame_list.append(frame_list[i])


		# Calculte various statistics
		[mean_error, max_error, min_error, sd] = findStats(error_list)

		mean_error_list.append(mean_error)
		max_error_list.append(max_error)
		min_error_list.append(min_error)
		std_dev_list.append(sd)


		if(plot2cam):
			plt.figure()
			plt.scatter(filtered_frame_list, error_list)

			plt.xlabel('Frame_number')
			plt.ylabel(parameterTypeString + ' Error (metres)')
			plt.grid()
			plt.title('Drone '+parameterTypeString+' Error of Camera'+str(cam_index1+1)+' and '+str(cam_index2+1)+ ' estimates')

 
 	
	# Plot for 3 cameras - 1 combination
	combinations = [(0,1,2)]
	plot3cam = plotcams[2]

	for (cam_index1, cam_index2, cam_index3) in combinations:
		
		error_list = []
		filtered_frame_list = []

		for i in range(len( dof1_estimate[cam_index1] )):

			# if pose w
			if(pose_found[cam_index1][i] == pose_found[cam_index2][i] == pose_found[cam_index3][i] == True):

				
				p_ref = (avg_ref_dof1_estimates[i], avg_ref_dof2_estimates[i], avg_ref_dof3_estimates[i])

				p_est = (
							float(dof1_estimate[cam_index1][i] + dof1_estimate[cam_index2][i] + dof1_estimate[cam_index3][i])/3.0,
							float(dof2_estimate[cam_index1][i] + dof2_estimate[cam_index2][i] + dof2_estimate[cam_index3][i])/3.0,
							float(dof3_estimate[cam_index1][i] + dof3_estimate[cam_index2][i] + dof3_estimate[cam_index3][i])/3.0
						)


				error = findDistance(p_ref, p_est)
				#error = abs(p_ref[0] -  p_est[0]) + abs(p_ref[1] -  p_est[1]) + abs(p_ref[2] -  p_est[2])


				error_list.append(error)
				filtered_frame_list.append(frame_list[i])


		# Calculte various statistics
		[mean_error, max_error, min_error, sd] = findStats(error_list)

		mean_error_list.append(mean_error)
		max_error_list.append(max_error)
		min_error_list.append(min_error)
		std_dev_list.append(sd)



		if(plot3cam):
			plt.figure()
			plt.scatter(filtered_frame_list, error_list)

			plt.xlabel('Frame_number')
			plt.ylabel(parameterTypeString + ' Error (metres)')
			plt.grid()
			plt.title('Drone '+parameterTypeString+' Error of Camera'+str(cam_index1+1)+','+str(cam_index2+1)+' and '+str(cam_index3+1)+ ' estimates')
	
	

	
	return [mean_error_list, max_error_list, min_error_list, std_dev_list]








# Input is all cameras' location in world frame. And Drone pose estimates.
def plotDistanceforAllCombinations(camera_world_positions,
								   x_estimates, avg_ref_x_estimates,
								   y_estimates, avg_ref_y_estimates,
								   z_estimates, avg_ref_z_estimates,
								   pose_found, 
								   frame_list,
								   plotcams):

	

	# Simple check for input data.
	assert(len(x_estimates[0]) == len(y_estimates[0]) == len(z_estimates[0]))
	
	mean_distance_list = []
	max_distance_list = []
	min_distance_list = []



	
	# Plot for 1 camera - 3 combinations
	combinations = [0,1,2]
	plot1cam = plotcams[0]
	for cam_index in combinations:

		error_list = []
		distance_list = []
		

		# Loop over each estimate
		for i in range(len( x_estimates[cam_index] )):

			if(pose_found[cam_index][i] == True):

				p_ref = (avg_ref_x_estimates[i], avg_ref_y_estimates[i], avg_ref_z_estimates[i])
				p_est = (x_estimates[cam_index][i], y_estimates[cam_index][i], z_estimates[cam_index][i])

				error = findDistance(p_ref, p_est)
				cameraDist = findDistance(p_ref, camera_world_positions[cam_index])

				#error = abs(p_ref[0] -  p_est[0]) + abs(p_ref[1] -  p_est[1]) + abs(p_ref[2] -  p_est[2])
				

				error_list.append(error)
				distance_list.append(cameraDist)
			

		

		# Calculte various statistics ?

		[mean_dist, max_dist, min_dist, sd] = findStats(distance_list)

		mean_distance_list.append(mean_dist)
		max_distance_list.append(max_dist)
		min_distance_list.append(min_dist)


		if(plot1cam):
			plt.figure()
			plt.scatter(distance_list, error_list)

			plt.xlabel('Distance (metres)')
			plt.ylabel('Position Error (metres)')
			plt.grid()
			plt.title('Drone position Error with Distance from Camera'+str(cam_index+1))



	
	# Plot for 2 cameras - 3 combinations
	combinations = [(0,1),(1,2),(0,2)]
	plot2cam = plotcams[1]
	for (cam_index1, cam_index2) in combinations:

		error_list = []
		distance_list = []
		


		for i in range(len( x_estimates[cam_index1] )):
			
			# if pose is found in both cameras
			if(pose_found[cam_index1][i] == pose_found[cam_index2][i] == True):


				# error = ref - (avg of the 2 cameras)
				p_ref = (avg_ref_x_estimates[i], avg_ref_y_estimates[i], avg_ref_z_estimates[i])

				p_est = ( 
						  float(x_estimates[cam_index1][i] + x_estimates[cam_index2][i])/2.0, 
						  float(y_estimates[cam_index1][i] + y_estimates[cam_index2][i])/2.0,
						  float(z_estimates[cam_index1][i] + z_estimates[cam_index2][i])/2.0 
						 )

				error = findDistance(p_ref, p_est)
				#error = abs(p_ref[0] -  p_est[0]) + abs(p_ref[1] -  p_est[1]) + abs(p_ref[2] -  p_est[2])
				cameraDist1 = findDistance(p_ref, camera_world_positions[cam_index1])
				cameraDist2 = findDistance(p_ref, camera_world_positions[cam_index2])

				#print('cameraDist1 = %s' % cameraDist1)
				#print('cameraDist2 = %s' % cameraDist2)

				#cameraDist = float(cameraDist1 + cameraDist2)/2.0
				cameraDist = max(cameraDist1, cameraDist2)

				error_list.append(error)
				distance_list.append(cameraDist)
				


		# Calculte various statistics
		
		[mean_dist, max_dist, min_dist, sd] = findStats(distance_list)

		mean_distance_list.append(mean_dist)
		max_distance_list.append(max_dist)
		min_distance_list.append(min_dist)


		if(plot2cam):
			plt.figure()
			plt.scatter(distance_list, error_list)

			plt.xlabel('Max Distance from cameras (metres)')
			plt.ylabel('Position Error (metres)')
			plt.grid()
			plt.title('Drone position Error with Distance from Camera'+str(cam_index1+1)+'and '+str(cam_index2+1))



	
	# Plot for 3 cameras - 1 combination
	combinations = [(0,1,2)]
	plot3cam = plotcams[2]
	for (cam_index1, cam_index2, cam_index3) in combinations:
		
		error_list = []
		
		distance_list = []

		for i in range(len( x_estimates[cam_index1] )):

			# if pose is found in both cameras
			if(pose_found[cam_index1][i] == pose_found[cam_index2][i] == pose_found[cam_index3][i] == True):

				
				p_ref = (avg_ref_x_estimates[i], avg_ref_y_estimates[i], avg_ref_z_estimates[i])

				p_est = (
							float(x_estimates[cam_index1][i] + x_estimates[cam_index2][i] + x_estimates[cam_index3][i])/3.0,
							float(y_estimates[cam_index1][i] + y_estimates[cam_index2][i] + y_estimates[cam_index3][i])/3.0,
							float(z_estimates[cam_index1][i] + z_estimates[cam_index2][i] + z_estimates[cam_index3][i])/3.0
						)


				error = findDistance(p_ref, p_est)
				#error = abs(p_ref[0] -  p_est[0]) + abs(p_ref[1] -  p_est[1]) + abs(p_ref[2] -  p_est[2])

				cameraDist1 = findDistance(p_ref, camera_world_positions[cam_index1])
				cameraDist2 = findDistance(p_ref, camera_world_positions[cam_index2])
				cameraDist3 = findDistance(p_ref, camera_world_positions[cam_index3])

				#print('cameraDist1 = %s' % cameraDist1)
				#print('cameraDist2 = %s' % cameraDist2)
				#print('cameraDist3 = %s' % cameraDist3)

				#cameraDist = float(cameraDist1 + cameraDist2 + cameraDist3)/3.0
				cameraDist = max( max(cameraDist1, cameraDist2), cameraDist3) 


				error_list.append(error)
				distance_list.append(cameraDist)
				


		# Calculte various statistics
		[mean_dist, max_dist, min_dist, sd] = findStats(distance_list)

		mean_distance_list.append(mean_dist)
		max_distance_list.append(max_dist)
		min_distance_list.append(min_dist)



		if(plot3cam):
			plt.figure()

			print(len(distance_list))
			print(len(error_list))
			plt.scatter(distance_list, error_list)

			plt.xlabel('Max Distance from cameras (metres)')
			plt.ylabel('Position Error (metres)')
			plt.grid()
			plt.title('Drone position Error with Distance from Camera'+str(cam_index1+1)+','+str(cam_index2+1)+'and '+str(cam_index3+1))
		
	

	return [mean_distance_list, max_distance_list, min_distance_list]







def findStats(myList):

	

	mean_val = sum(myList)/float(len(myList))
	max_val = max(myList)
	min_val = min(myList)

	differences = [x - mean_val for x in myList]
	sq_differences = [d ** 2 for d in differences]
	N = len(myList)
	sd = sqrt(sum(sq_differences)/float(N))

	# print("myList = %s\n" % myList)
	# print("mean_val = %s\n" % mean_val)

	# print("differences list = %s\n" % differences)
	# print("sq_differences list = %s\n" % sq_differences)
	# print("sd = %s\n" % sd)

	return [mean_val, max_val, min_val, sd]


#nput is either x,y,z (or) roll,pitch,yaw.
def plotErrorforAllCombinations_orien(dof1_estimate, avg_ref_dof1_estimates, 
								      dof2_estimate, avg_ref_dof2_estimates,
								      dof3_estimate, avg_ref_dof3_estimates,
								      pose_found, 
								      frame_list, 
								      parameterTypeString,
								      plotcams):

	

	# Simple check for input data.
	assert(len(dof1_estimate[0]) == len(dof2_estimate[0]) == len(dof3_estimate[0]))
	assert(len(dof1_estimate[0]) == len(avg_ref_dof1_estimates) == len(avg_ref_dof2_estimates)==len(avg_ref_dof3_estimates))

	mean_error_list = []
	max_error_list = []
	min_error_list = []
	std_dev_list = []



	# Plot for 1 camera - 3 combinations
	combinations = [0,1,2]
	plot1cam = plotcams[0]
	for cam_index in combinations:

		error_list = []
		filtered_frame_list = []

		# Loop over each estimate
	
		for i in range(len( dof1_estimate[cam_index] )):

			if(pose_found[cam_index][i] == True):

				p_ref = (avg_ref_dof1_estimates[i], avg_ref_dof2_estimates[i], avg_ref_dof3_estimates[i])
				p_est = (dof1_estimate[cam_index][i], dof2_estimate[cam_index][i], dof3_estimate[cam_index][i])

				error = findOrientationError(p_ref, p_est)

				#if( error > 10):
					#print("Reference estimate= ", p_ref)
					#print("My estimate = ", p_est)
					#print("\n")

				error_list.append(error)
				filtered_frame_list.append(frame_list[i])

		#error_list.remove(175.6270725654811)
		#error_list.remove(77.55430727940902)
		#error_list.remove(75.39210998849325)
		[mean_error, max_error, min_error, sd] = findStats(error_list)

		mean_error_list.append(mean_error)
		max_error_list.append(max_error)
		min_error_list.append(min_error)
		std_dev_list.append(sd)


		if(plot1cam):
			plt.figure()
			plt.scatter(filtered_frame_list, error_list)

			plt.xlabel('Frame_number')
			plt.ylabel(parameterTypeString + 'Error (degrees)')
			plt.grid()
			plt.title('Drone Orientation Error for Camera '+str(cam_index+1)) 



	
	# Plot for 2 cameras - 3 combinations
	combinations = [(0,1),(1,2),(0,2)]
	plot2cam = plotcams[1]
	for (cam_index1, cam_index2) in combinations:

		error_list = []
		filtered_frame_list = []

		for i in range(len( dof1_estimate[cam_index1] )):
			
			# if pose is found in both cameras
			if(pose_found[cam_index1][i] == pose_found[cam_index2][i] == True):



				# error = ref - (avg of the 2 cameras)
				p_ref = (avg_ref_dof1_estimates[i], avg_ref_dof2_estimates[i], avg_ref_dof3_estimates[i])
				p_est = ( 
						  float(dof1_estimate[cam_index1][i] + dof1_estimate[cam_index2][i])/2.0, 
						  float(dof2_estimate[cam_index1][i] + dof2_estimate[cam_index2][i])/2.0,
						  float(dof3_estimate[cam_index1][i] + dof3_estimate[cam_index2][i])/2.0 
						 )


				error = findOrientationError(p_ref, p_est)


				error_list.append(error)
				filtered_frame_list.append(frame_list[i])	

		[mean_error, max_error, min_error, sd] = findStats(error_list)

		mean_error_list.append(mean_error)
		max_error_list.append(max_error)
		min_error_list.append(min_error)
		std_dev_list.append(sd)

		if(plot2cam):
			plt.figure()
			plt.scatter(filtered_frame_list, error_list)

			plt.xlabel('Frame_number')
			plt.ylabel(parameterTypeString + ' Error (degrees)')
			plt.grid()
			plt.title('Drone Orientation Error for Camera '+str(cam_index1+1)+'and '+str(cam_index2+1)) 

	
	# Plot for 3 cameras - 1 combination
	combinations = [(0,1,2)]
	plot3cam = plotcams[2]

	for (cam_index1, cam_index2, cam_index3) in combinations:
		
		error_list = []
		filtered_frame_list = []

		for i in range(len( dof1_estimate[cam_index1] )):

			# if pose w
			if(pose_found[cam_index1][i] == pose_found[cam_index2][i] == pose_found[cam_index3][i] == True):

				
				p_ref = (avg_ref_dof1_estimates[i], avg_ref_dof2_estimates[i], avg_ref_dof3_estimates[i])
				p_est = (
							float(dof1_estimate[cam_index1][i] + dof1_estimate[cam_index2][i] + dof1_estimate[cam_index3][i])/3.0,
							float(dof2_estimate[cam_index1][i] + dof2_estimate[cam_index2][i] + dof2_estimate[cam_index3][i])/3.0,
							float(dof3_estimate[cam_index1][i] + dof3_estimate[cam_index2][i] + dof3_estimate[cam_index3][i])/3.0
						)


				error = findOrientationError(p_ref, p_est)
				error_list.append(error)
				filtered_frame_list.append(frame_list[i])


		[mean_error, max_error, min_error, sd] = findStats(error_list)

		mean_error_list.append(mean_error)
		max_error_list.append(max_error)
		min_error_list.append(min_error)
		std_dev_list.append(sd)


		if(plot3cam):
			plt.figure()
			plt.scatter(filtered_frame_list, error_list)

			plt.xlabel('Frame_number')
			plt.ylabel(parameterTypeString + ' Error (degrees)')
			plt.grid()
			plt.title('Drone Orientation Error for Camera '+str(cam_index1+1)+','+str(cam_index2+1)+'and '+str(cam_index3+1)) 
	

	return [mean_error_list, max_error_list, min_error_list, std_dev_list]




# Get overview of all combinations used.
def plotAllCombinationStats(mean_error_list, max_error_list,
							min_error_list, std_dev_list,
							parameterTypeString):


	combination_label = ('C1', 'C2', 'C3', 'C1C2', 'C2C3','C1C3','C1C2C3')

	plt.figure()
	
	#plt.subplot(2,1,1)
	

	x = range(len(mean_error_list))
	plt.scatter(x, mean_error_list)
	plt.xticks(np.arange(7), combination_label)
	

	#plt.gca().set_ylim([-1,2])
	plt.xlabel('Combination used')
	if(parameterTypeString == 'Position'):
		y_label = 'Mean Position Error (metres)'
	elif(parameterTypeString == 'Orientation'):
		y_label = 'Mean Orientation Error (degrees)'

	plt.ylabel(y_label)
	plt.grid()
	title = 'Mean '+parameterTypeString+' error of each combination of cameras'
	plt.title(title)


	plt.figure()

	x = range(len(std_dev_list))
	plt.scatter(x, std_dev_list)
	plt.xticks(np.arange(7), combination_label)

	plt.xlabel('Combination used')

	if(parameterTypeString == 'Position'):
		y_label = 'Std deviation (metres)'
	elif(parameterTypeString == 'Orientation'):
		y_label = 'Std deviation (degrees)'

	plt.ylabel(y_label)
	plt.grid()
	title = 'Std deviation for '+parameterTypeString+' error of each combination of cameras'
	plt.title(title)





def plotPositionError(frame_list , region_list,
					  x_estimates, avg_ref_x_estimates,
					  y_estimates, avg_ref_y_estimates,
					  z_estimates, avg_ref_z_estimates,
					  pose_found,
					  plotcams):



	
	[mean_error_list, max_error_list, min_error_list, std_dev_list] = plotErrorforAllCombinations_pos( x_estimates, avg_ref_x_estimates,
																									   y_estimates, avg_ref_y_estimates,
																									   z_estimates, avg_ref_z_estimates,
																									   pose_found, 
																									   frame_list,
																									   "Position",
																									   plotcams)

	# Get stats on all combinations ....
	print("Mean error= \n %s" % mean_error_list); 
	#print("Max error= \n %s" % max_error_list); 
	#print("Min error= \n %s" % min_error_list); 
	#print("std_dev error= \n %s" % std_dev_list); 
	print('\n')


	
	parameterTypeString = 'Position'
	plotAllCombinationStats(mean_error_list, max_error_list,min_error_list, std_dev_list,parameterTypeString);

	
	

def plotOrientationError(frame_list, region_list,
						 roll_estimates, avg_ref_roll_estimates,
						 pitch_estimates, avg_ref_pitch_estimates,
						 yaw_estimates, avg_ref_yaw_estimates,
						 pose_found,
						 plotcams):

	[mean_error_list, max_error_list, min_error_list, std_dev_list] = plotErrorforAllCombinations_orien(roll_estimates, avg_ref_roll_estimates,
								  pitch_estimates, avg_ref_pitch_estimates,
								  yaw_estimates, avg_ref_yaw_estimates,
								  pose_found,
								  frame_list,
								  "Orientation",
								  plotcams)


	print("Mean error= \n %s" % mean_error_list); 
	#print("Max error= \n %s" % max_error_list); 
	#print("Min error= \n %s" % min_error_list); 
	print("std_dev error= \n %s" % std_dev_list); 
	print('\n')


	
	parameterTypeString = 'Orientation'

	plotAllCombinationStats(mean_error_list, max_error_list,min_error_list, std_dev_list,parameterTypeString)



#Plots the error between the (average) reference and the (average over cameras) state estimate
def plotError(frame_list, region_list,
			  x_estimates, avg_ref_x_estimates,
			  y_estimates, avg_ref_y_estimates,
			  z_estimates, avg_ref_z_estimates,
			  roll_estimates, avg_ref_roll_estimates,
			  pitch_estimates, avg_ref_pitch_estimates,
			  yaw_estimates, avg_ref_yaw_estimates,
			  pose_found,
			  plotcams):

		
			  
			  plotPositionError(frame_list, region_list,
							 x_estimates, avg_ref_x_estimates,
							 y_estimates, avg_ref_y_estimates,
							 z_estimates, avg_ref_z_estimates,
							 pose_found,
							 plotcams)

			 
			
			  
			  plotOrientationError(frame_list, region_list,
					 roll_estimates, avg_ref_roll_estimates,
					 pitch_estimates, avg_ref_pitch_estimates,
					 yaw_estimates, avg_ref_yaw_estimates,
					 pose_found,
					 plotcams)
			  
			  
	
	
	







def plotActualAndReference(
				 frame_list, region_list,
				 x_estimates, avg_ref_x_estimates,
				 y_estimates, avg_ref_y_estimates,
				 z_estimates, avg_ref_z_estimates,
				 roll_estimates, avg_ref_roll_estimates,
				 pitch_estimates, avg_ref_pitch_estimates,
				 yaw_estimates, avg_ref_yaw_estimates):

	



	# Plot x coordinates against frame number
	
	plt.subplot(1,3,1)
	

	
	for i in range(NUM_CAMS):
		plt.plot(frame_list, x_estimates[i], label = 'cam'+str(i))
	
	plt.plot(frame_list, avg_ref_x_estimates, label = 'ref')

	

	#plt.gca().set_ylim([-1,2])
	plt.xlabel('frame_number')
	plt.ylabel('X coordinate (metres)')
	plt.grid()
	plt.title('Actual vs reference X coordinate')


	# Colour the region where at least 1 camera found a pose.
	for i in range(len(region_list)):
		plt.axvspan(region_list[i][0], region_list[i][1], color='green', alpha=0.2)

		
	plt.legend()






	# Plot y coordinates against frame number
	plt.subplot(1,3,2)
	
	for i in range(NUM_CAMS):
		plt.plot(frame_list, y_estimates[i], label = 'cam'+str(i))
	plt.plot(frame_list, avg_ref_y_estimates, label = 'ref')


	# Colour the region where at least 1 camera found a pose.
	for i in range(len(region_list)):
		plt.axvspan(region_list[i][0], region_list[i][1], color='green', alpha=0.2)


	#plt.gca().set_ylim([-1,2])
	plt.xlabel('frame_number')
	plt.ylabel('Y coordinate (metres)')
	plt.grid()
	plt.title('Actual vs reference Y coordinate')


	
	plt.legend()


	# Plot z coordinates against frame number
	plt.subplot(1,3,3)
	
	for i in range(NUM_CAMS):
		plt.plot(frame_list, z_estimates[i], label = 'cam'+str(i))

	plt.plot(frame_list, avg_ref_z_estimates, label = 'ref')


	# Colour the region where at least 1 camera found a pose.
	for i in range(len(region_list)):
		plt.axvspan(region_list[i][0], region_list[i][1], color='green', alpha=0.2)


	plt.legend()
	#plt.gca().set_ylim([-1,2])
	plt.xlabel('frame_number')
	plt.ylabel('Z coordinate (metres)')
	plt.grid()
	plt.title('Actual vs reference Z coordinate')

	







	# =========== Plot RPY on a new figure =============
	
	# Plot roll angle (degrees) against frame number
	plt.figure()
	plt.subplot(1,3,1)
	
	for i in range(NUM_CAMS):
		plt.plot(frame_list, roll_estimates[i], label = 'cam'+str(i))

	plt.plot(frame_list, avg_ref_roll_estimates, label = 'ref')
	
	plt.legend()

	#ax.set_ylim([-1,2])
	plt.xlabel('frame_number')
	plt.ylabel('Roll estimate (degrees)')
	plt.grid()
	plt.title('Actual vs reference Roll angle')

	# Colour the region where at least 1 camera found a pose.
	for i in range(len(region_list)):
		plt.axvspan(region_list[i][0], region_list[i][1], color='green', alpha=0.2)



	
	# Plot pitch angle (degrees) against frame number
	plt.subplot(1,3,2)
	
	for i in range(NUM_CAMS):
		plt.plot(frame_list, pitch_estimates[i], label = 'cam'+str(i))

	plt.plot(frame_list, avg_ref_pitch_estimates, label = 'ref')

	
	plt.legend()

	#ax.set_ylim([-1,2])
	plt.xlabel('frame_number')
	plt.ylabel('Pitch estimate (degrees)')
	plt.grid()
	plt.title('Actual vs reference Pitch angle')


	# Colour the region where at least 1 camera found a pose.
	for i in range(len(region_list)):
		plt.axvspan(region_list[i][0], region_list[i][1], color='green', alpha=0.2)





	# Plot yaw angle (degrees) against frame number
	plt.subplot(1,3,3)
	
	for i in range(NUM_CAMS):
		plt.plot(frame_list, yaw_estimates[i], label = 'cam'+str(i))

	plt.plot(frame_list, avg_ref_yaw_estimates, label = 'ref')

	plt.legend()

	#ax.set_ylim([-1,2])
	plt.xlabel('frame_number')
	plt.ylabel('Yaw estimate (degrees)')
	plt.grid()
	plt.title('Actual vs reference Yaw angle')



	# Colour the region where at least 1 camera found a pose.
	for i in range(len(region_list)):
		plt.axvspan(region_list[i][0], region_list[i][1], color='green', alpha=0.2)
	







# Plots variaboility in state estimate if more than one camera is used.
def plotStateEstimateSummary(summaryList):

	# Number of (synchronized) frames
	frame_list = []


	"""
	# Reference estimates provided by each camera.
	ref_x_estimates = [[] for i in range(NUM_CAMS)]
	ref_y_estimates = [[] for i in range(NUM_CAMS)]
	ref_z_estimates = [[] for i in range(NUM_CAMS)]
	ref_roll_estimates = [[] for i in range(NUM_CAMS)]
	ref_pitch_estimates = [[] for i in range(NUM_CAMS)]
	ref_yaw_estimates = [[] for i in range(NUM_CAMS)]
	"""

	# AVERAGE Reference estimates provided by each camera.
	avg_ref_x_estimates = []
	avg_ref_y_estimates = []
	avg_ref_z_estimates = []
	avg_ref_roll_estimates = []
	avg_ref_pitch_estimates = []
	avg_ref_yaw_estimates = []


	# State Estimates provided by each camera.
	x_estimates = [[] for i in range(NUM_CAMS)]
	y_estimates = [[] for i in range(NUM_CAMS)]
	z_estimates = [[] for i in range(NUM_CAMS)]
	roll_estimates = [[] for i in range(NUM_CAMS)]
	pitch_estimates = [[] for i in range(NUM_CAMS)]
	yaw_estimates = [[] for i in range(NUM_CAMS)]




	# For each reading
	for frame_number in range(len(summaryList)):

		sumObj = summaryList[frame_number]

		avg_ref_x_estimates.append(sumObj.avg_refState.x)
		avg_ref_y_estimates.append(sumObj.avg_refState.y)
		avg_ref_z_estimates.append(sumObj.avg_refState.z)
		avg_ref_roll_estimates.append(sumObj.avg_refState.roll)
		avg_ref_pitch_estimates.append(sumObj.avg_refState.pitch)
		avg_ref_yaw_estimates.append(sumObj.avg_refState.yaw)


		# store state estimate value for each camera
		for camid in sumObj.camid_list:
			x_estimates[camid].append(sumObj.droneWorldstate_per_camera[camid].x)
			y_estimates[camid].append(sumObj.droneWorldstate_per_camera[camid].y)
			z_estimates[camid].append(sumObj.droneWorldstate_per_camera[camid].z)
			roll_estimates[camid].append(sumObj.droneWorldstate_per_camera[camid].roll)
			pitch_estimates[camid].append(sumObj.droneWorldstate_per_camera[camid].pitch)
			yaw_estimates[camid].append(sumObj.droneWorldstate_per_camera[camid].yaw)


			"""
			ref_x_estimates[camid].append(sumObj.refState_list[camid].x)
			ref_y_estimates[camid].append(sumObj.refState_list[camid].y)
			ref_z_estimates[camid].append(sumObj.refState_list[camid].z)
			ref_roll_estimates[camid].append(sumObj.refState_list[camid].roll)
			ref_pitch_estimates[camid].append(sumObj.refState_list[camid].pitch)
			ref_yaw_estimates[camid].append(sumObj.refState_list[camid].yaw)
			"""



		frame_list.append(frame_number)




#======================== Find regions where pose was found =================

	# Also find in which regions the pose was found ....
	pose_found = [[] for i in range(NUM_CAMS)]

	# For each reading
	for frame_number in range(len(frame_list)):

		sumObj = summaryList[frame_number]

		# store found pose value for each camera
		for camid in sumObj.camid_list:
			pose_found[camid].append(sumObj.foundpose_per_camera[camid])



	# For each frame, find how many cameras have found a pose.
	number_of_poses_found_per_frame = []

	for frame_number in range(len(frame_list)):

		# for each entry/frame, we check how many cameras have found a pose.
		foundpose = 0
		for camid in range(NUM_CAMS):
			foundpose = foundpose + pose_found[camid][frame_number]

		number_of_poses_found_per_frame.append(foundpose)



	# Find all "regions" (continuous 1s)
	region_list = []

	start_point = 0
	end_point = 0

	set_start_point = False

	# Loop over each frame, and check how many poses were found.
	# Create "regions" [start_point]
	for i in range(len(number_of_poses_found_per_frame)):

		# Case 1: Mark the start of every region
		if( number_of_poses_found_per_frame[i] > 0 and set_start_point == False ):
			start_point = i
			set_start_point = True


		# Case 2: The moment pose found is false means that pose was found tll previous frame.
		if( number_of_poses_found_per_frame[i] == 0 ):
			end_point = i-1
			region_list.append([start_point,end_point])

			set_start_point = False
			start_point = 0
			end_point = 0


		# Case 3: pose was found in all frames.
		elif( (i == len(number_of_poses_found_per_frame)-1) and (set_start_point == True) ):
			end_point = i
			region_list.append([start_point,end_point])

			set_start_point = False
			start_point = 0
			end_point = 0
	

#========================================================

	# Plot Actual estimates
	"""
	plotActualEstimates(frame_list,
				 region_list,
				 x_estimates,
				 y_estimates,
				 z_estimates,
				 roll_estimates,
				 pitch_estimates,
				 yaw_estimates)
	

	# Plot actual + reference estiamtes
	plotActualAndReference(frame_list,
				 region_list,
				 x_estimates, avg_ref_x_estimates,
				 y_estimates, avg_ref_y_estimates,
				 z_estimates, avg_ref_z_estimates,
				 roll_estimates, avg_ref_roll_estimates,
				 pitch_estimates, avg_ref_pitch_estimates,
				 yaw_estimates, avg_ref_yaw_estimates)
	

	
	# Calculates error for the cameras.
	# only finds error when a camera returns a pose estiamte.
	"""
	
	#TODO: find covariance of pose estimates ?
	#plotMeasurementVariability():

	
	# World pose of each camera as provided by April Tag.
	# Note that this will change for each configuration of cameras
	# so ensure that the corect one is used according to the configuration
	# for which the data was gathered.
	camera_world_positions = [ 
							   [1.66289, 4.51988, 2.1449] ,
							   [3.71851, 1.11042, 2.17068] ,
							   [6.0645, 4.28018, 2.17841] 
							 ]

	plotcams = [False, False, True]

	plotError(frame_list,
			 region_list,
			 x_estimates, avg_ref_x_estimates,
			 y_estimates, avg_ref_y_estimates,
			 z_estimates, avg_ref_z_estimates,
			 roll_estimates, avg_ref_roll_estimates,
			 pitch_estimates, avg_ref_pitch_estimates,
			 yaw_estimates, avg_ref_yaw_estimates,
			 pose_found,
			 plotcams)


	
	[mean_distance_list, max_distance_list, min_distance_list] = plotDistanceforAllCombinations(camera_world_positions,
								   x_estimates, avg_ref_x_estimates,
								   y_estimates, avg_ref_y_estimates,
								   z_estimates, avg_ref_z_estimates,
								   pose_found, 
								   frame_list,
								   plotcams)
	

	#print("mean_distance_list = %s" % mean_distance_list)
	#print("max_distance_list = %s" % max_distance_list)
	#print("min_distance_list = %s" % min_distance_list)
	

	"""
	#ref_rpy = (1,2,3)
	#est_rpy = (120,120,120)
	#err = findOrientationError(ref_rpy, est_rpy)
	"""

	summaryString_list.append('\n')
	summaryString_list.append('TODO: find mean and standard deviation of error plot = ??' )
	







#========================================================

# Top level function.
def plotSummary(summaryList):
	
	# This object holds summary strings from each function called below.
	global summaryString_list

	# Various functions for plotting. Each returns a string containing some summary.
	#plotFoundPose(summaryList)
	plotStateEstimateSummary(summaryList)

	#Display summary from each plot.q
	for sumString in summaryString_list:
		print(sumString)

	plt.show()


	"""
	# Press Ctrl+C to finish program.
	# This might be a hacky way
	# Got it from - https://gist.github.com/djwbrown/3e24bf4e0c5e9ee156a5
	try:
	    # Put matplotlib.pyplot in interactive mode so that the plots are shown in a background thread.
	    plt.ion()
	    while(True):
	    	plt.show()

	except KeyboardInterrupt:
		print ""
		sys.exit(0)
	"""


	


# Creates the summary Obj for 1 OBSERVATION.
def createSummaryObj(rowList):

	global NUM_CAMS


	# since this is common for all NUM_CAMS rows
	timestamp = int(rowList[0][0])
	


	refState_list = []
	camid_list = []
	foundpose_per_camera = []
	leds_per_camera = []
	stateEstimate_list = []

	avg_ref_x = 0.0
	avg_ref_y = 0.0
	avg_ref_z = 0.0
	avg_ref_roll = 0.0
	avg_ref_pitch = 0.0
	avg_ref_yaw = 0.0


	valid_refState = 0

	for row in rowList:
		
		refState = State( float(row[1]), float(row[2]), float(row[3]),
						  float(row[4]), float(row[5]), float(row[6]) ); 

		refState_list.append(refState)

		if(not (refState.x == refState.y == refState.z == refState.roll == refState.pitch == refState.yaw == 0)):
			valid_refState += 1

		avg_ref_x += refState.x
		avg_ref_y += refState.y
		avg_ref_z += refState.z
		avg_ref_roll += refState.roll
		avg_ref_pitch += refState.pitch
		avg_ref_yaw += refState.yaw


		camid = int(row[7]); camid_list.append(camid)
		foundPose = int(row[8]); foundpose_per_camera.append(foundPose) 
		leds = int(row[9]); leds_per_camera.append(leds)
		
		
		stateEstimate = State( float(row[10]), float(row[11]), float(row[12]),
							   float(row[13]), float(row[14]), float(row[15]))

		stateEstimate_list.append(stateEstimate)


	avg_ref_x = float(avg_ref_x)/float(valid_refState)
	avg_ref_y = float(avg_ref_y)/float(valid_refState)
	avg_ref_z = float(avg_ref_z)/float(valid_refState)
	avg_ref_roll = float(avg_ref_roll)/float(valid_refState)
	avg_ref_pitch = float(avg_ref_pitch)/float(valid_refState)
	avg_ref_yaw = float(avg_ref_yaw)/float(valid_refState)



	avg_refState = State(avg_ref_x, avg_ref_y, avg_ref_z,
						avg_ref_roll, avg_ref_pitch, avg_ref_yaw)


	summaryObj = SummaryObj(timestamp,
							refState_list,
							avg_refState, 
					   		camid_list,
					  	 	foundpose_per_camera,
					   		leds_per_camera,
					   		stateEstimate_list)

	return summaryObj


		

def printAverageRefState(summaryList):
	for summaryObj in summaryList:

				val= summaryObj.avg_refState
				print(val.x)
				print(val.y)
				print(val.z)
				print(val.roll)
				print(val.pitch)
				print(val.yaw)
		
				print('\n')

# Read the observations from the CSV file and stores it in a list.
def readSummaryFile():

	global NUM_CAMS

	# Read a CSV file and store
	with open('summary.txt') as csvfile:
		readcsv = csv.reader(csvfile, delimiter=',')

		# This stores the CSV file.
		summaryList = []

		rowList = []

		# skips the first line.
		next(csvfile)

		# After every 3 rows appends an object to the list
		for row in readcsv:

			rowList.append(row)

			if( len(rowList) == NUM_CAMS ):

				#Create SummaryObj and append to list.
				sumObj = createSummaryObj(rowList)
				summaryList.append(sumObj)

				# reset rowList
				rowList = []


		#printAverageRefState(summaryList)

		return summaryList




def main():

	summaryList = readSummaryFile()

	plotSummary(summaryList)


if __name__ == '__main__':
	main()
