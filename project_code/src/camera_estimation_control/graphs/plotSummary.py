import csv
import matplotlib.pyplot as plt
import numpy as np

import sys

NUM_CAMS = 3

summaryString_list = []

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
	def __init__(self, timestamp, 
					   camid_list,
					   foundpose_per_camera,
					   leds_per_camera,
					   confidence_list,
					   latency_list,
					   droneWorldstate_per_camera):

		self.timestamp = timestamp
		self.camid_list = camid_list
		self.foundpose_per_camera = foundpose_per_camera
		self.leds_per_camera = leds_per_camera
		self.confidence_list = confidence_list
		self.latency_list = latency_list
		self.droneWorldstate_per_camera = droneWorldstate_per_camera
		



# Plots frames found and lost frames (30 - frames found)
def plotFramesFoundAndLost(summaryList):


	plt.figure()

	# Gather all timestamps
	timestamp_list = [summaryObj.timestamp for summaryObj in summaryList]
	timestamp_list = sorted(set(timestamp_list))

	hist1 = dict((timestamp,0) for timestamp in timestamp_list)
	hist2 = dict((timestamp,0) for timestamp in timestamp_list)



	# Calculate frames lost or found at each second
	for summaryObj in summaryList:

		hist1[summaryObj.timestamp] = hist1.get(summaryObj.timestamp, 0) + 1
		# this is slightly wasteful way of doing
		hist2[summaryObj.timestamp] = 30.0 - hist1[summaryObj.timestamp]

	

	# Plot frames found
	plt.subplot(2,1,1)

	frames_found_hist = [ hist1[timestamp] for timestamp in timestamp_list ]
	plt.plot(timestamp_list, frames_found_hist)
	ax = plt.gca()
	max_value = max(frames_found_hist)

	ax.set_ylim([0,max_value+1])
	plt.xticks(np.arange(min(timestamp_list), max(timestamp_list)+1, 2.0))

	plt.grid() # show grid

	plt.xlabel('timestamp (seconds)')
	plt.ylabel('Frames found')
	plt.title('Frames found in every second')



	# Plot frames lost
	plt.subplot(2,1,2)

	frames_lost_hist = [ hist2[timestamp] for timestamp in timestamp_list ]
	plt.plot(timestamp_list, frames_lost_hist)
	ax = plt.gca()
	max_value = max(frames_lost_hist)
	ax.set_ylim([0,max_value+1])
	plt.xticks(np.arange(min(timestamp_list), max(timestamp_list)+1, 2.0))
	plt.xlabel('timestamp (seconds)')
	plt.ylabel('Frames Lost')

	plt.grid() # show grid


	plt.title('Frames Lost in every second')
	plt.tight_layout()
	
	


	# Print out some summary 
	avg_frames_found = sum(frames_found_hist)/float(len(frames_found_hist) )
	avg_frames_lost = sum(frames_lost_hist)/float(len(frames_lost_hist) )

	print(avg_frames_found)
	summaryString_list.append('\n')
	summaryString_list.append('Average Frames found per second = '+str(avg_frames_found))
	summaryString_list.append('Average Frames lost per second = '+str(avg_frames_lost))







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






# Plots latency of each measurement
# TODO: Need to think about whether to report only if a pose is found.
def plotLatency(summaryList):

	plt.figure()

	# Number of (syncrhonized) frames
	x = []

	y_vals = [[] for i in range(NUM_CAMS)]

	# For each reading
	for frame_number in range(len(summaryList)):

		sumObj = summaryList[frame_number]

		# store found pose value for each camera
		for camid in sumObj.camid_list:
			y_vals[camid].append(sumObj.latency_list[camid])


		x.append(frame_number)



	# Also find in which regions the pose was found ....
	pose_found = [[] for i in range(NUM_CAMS)]

	# For each reading
	for frame_number in range(len(summaryList)):

		sumObj = summaryList[frame_number]

		# store found pose value for each camera
		for camid in sumObj.camid_list:
			pose_found[camid].append(sumObj.foundpose_per_camera[camid])



	# For each frame, find how many cameras have found a pose.
	number_of_poses_found_per_frame = []

	for i in range(len(x)):

		# for each entry/frame, we check how many cameras have found a pose.
		foundpose = 0
		for camid in range(len(y_vals)):
			foundpose = foundpose + pose_found[camid][i]

		number_of_poses_found_per_frame.append(foundpose)



	# Find all "regions" (continuous 1s)
	region_list = []

	start_point = 0
	end_point = 0

	set_start_point = False

	
	for i in range(len(number_of_poses_found_per_frame)):

		if( number_of_poses_found_per_frame[i] > 0 and set_start_point == False ):
			start_point = i
			set_start_point = True

		if( (number_of_poses_found_per_frame[i] == 0 or i == len(number_of_poses_found_per_frame)-1) and set_start_point == True):
			end_point = i-1
			region_list.append([start_point,end_point])

			set_start_point = False
			start_point = 0
			end_point = 0
	

	



	plt.subplot(2,1,1)
	
	for i in range(NUM_CAMS):
		plt.plot(x, y_vals[i], label = 'cam'+str(i))

	plt.title('Total Latency for each synchronized frame')
	ax = plt.gca()
	plt.legend()

	#ax.set_ylim([-1,2])
	plt.xlabel('frame_number')
	plt.ylabel('Latency per camera')
	plt.grid()
	plt.title('Latency for each synchronized frame')



	# For each (syncrhonized) frame, find total latency
	total_latency_per_frame = []

	for i in range(len(x)):

		# for each entry/frame, we check how many cameras have found a pose.
		total_latency = 0.0
		for camid in range(len(y_vals)):
			total_latency = total_latency + y_vals[camid][i]

		total_latency_per_frame.append(total_latency)


	# Colour the region where at least 1 camera found a pose.
	for i in range(len(region_list)):
		plt.axvspan(region_list[i][0], region_list[i][1], color='green', alpha=0.2)



	# But this also depends on the trajectory of the drone.
	plt.subplot(2,1,2)
	plt.grid()
	plt.plot(x, total_latency_per_frame)

	ax = plt.gca()
	#ax.set_ylim([-1,2])
	plt.xlabel('frame_number')
	plt.ylabel('Total Latency')
	plt.title('Total Latency for each synchronized frame')

	plt.tight_layout()

	# Colour the region where at least 1 camera found a pose.
	for i in range(len(region_list)):
		plt.axvspan(region_list[i][0], region_list[i][1], color='green', alpha=0.2)


	
	avg_latency = sum(total_latency_per_frame)/float(len(total_latency_per_frame))

	summaryString_list.append('\n')
	summaryString_list.append('Average Latency for each synchronized frame =' + str(avg_latency))
	







# Plots variaboility in state estimate if more than one camera is used.
def plotStateEstimate(summaryList):

	plt.figure()

	# Number of (synchronized) frames
	x = []

	x_estimates = [[] for i in range(NUM_CAMS)]
	y_estimates = [[] for i in range(NUM_CAMS)]
	z_estimates = [[] for i in range(NUM_CAMS)]
	roll_estimates = [[] for i in range(NUM_CAMS)]
	pitch_estimates = [[] for i in range(NUM_CAMS)]
	yaw_estimates = [[] for i in range(NUM_CAMS)]


	# For each reading
	for frame_number in range(len(summaryList)):

		sumObj = summaryList[frame_number]

		# store found pose value for each camera
		for camid in sumObj.camid_list:
			x_estimates[camid].append(sumObj.droneWorldstate_per_camera[camid].x)
			y_estimates[camid].append(sumObj.droneWorldstate_per_camera[camid].y)
			z_estimates[camid].append(sumObj.droneWorldstate_per_camera[camid].z)
			roll_estimates[camid].append(sumObj.droneWorldstate_per_camera[camid].roll)
			pitch_estimates[camid].append(sumObj.droneWorldstate_per_camera[camid].pitch)
			yaw_estimates[camid].append(sumObj.droneWorldstate_per_camera[camid].yaw)


		x.append(frame_number)




#======================== Find regions where pose was found =================

	# Also find in which regions the pose was found ....
	pose_found = [[] for i in range(NUM_CAMS)]

	# For each reading
	for frame_number in range(len(summaryList)):

		sumObj = summaryList[frame_number]

		# store found pose value for each camera
		for camid in sumObj.camid_list:
			pose_found[camid].append(sumObj.foundpose_per_camera[camid])



	# For each frame, find how many cameras have found a pose.
	number_of_poses_found_per_frame = []

	for i in range(len(x)):

		# for each entry/frame, we check how many cameras have found a pose.
		foundpose = 0
		for camid in range(len(x_estimates)):
			foundpose = foundpose + pose_found[camid][i]

		number_of_poses_found_per_frame.append(foundpose)



	# Find all "regions" (continuous 1s)
	region_list = []

	start_point = 0
	end_point = 0

	set_start_point = False

	
	for i in range(len(number_of_poses_found_per_frame)):

		if( number_of_poses_found_per_frame[i] > 0 and set_start_point == False ):
			start_point = i
			set_start_point = True

		if( (number_of_poses_found_per_frame[i] == 0 or i == len(number_of_poses_found_per_frame)-1) and set_start_point == True):
			end_point = i-1
			region_list.append([start_point,end_point])

			set_start_point = False
			start_point = 0
			end_point = 0
	

#========================================================



	# Plot x coordinates against frame number
	plt.subplot(1,3,1)
	
	for i in range(NUM_CAMS):
		plt.plot(x, x_estimates[i], label = 'cam'+str(i))


	ax = plt.gca()
	plt.legend()

	#ax.set_ylim([-1,2])
	plt.xlabel('frame_number')
	plt.ylabel('X coordinate estimate (metres)')
	plt.grid()
	plt.title('X coordinate estimate by each camera')


	# Colour the region where at least 1 camera found a pose.
	for i in range(len(region_list)):
		plt.axvspan(region_list[i][0], region_list[i][1], color='green', alpha=0.2)

	
	# Plot y coordinates against frame number
	plt.subplot(1,3,2)
	
	for i in range(NUM_CAMS):
		plt.plot(x, y_estimates[i], label = 'cam'+str(i))


	ax = plt.gca()
	plt.legend()

	#ax.set_ylim([-1,2])
	plt.xlabel('frame_number')
	plt.ylabel('Y coordinate estimate (metres)')
	plt.grid()
	plt.title('Y coordinate estimate by each camera')


	# Colour the region where at least 1 camera found a pose.
	for i in range(len(region_list)):
		plt.axvspan(region_list[i][0], region_list[i][1], color='green', alpha=0.2)



	# Plot z coordinates against frame number
	plt.subplot(1,3,3)
	
	for i in range(NUM_CAMS):
		plt.plot(x, z_estimates[i], label = 'cam'+str(i))


	ax = plt.gca()
	plt.legend()

	#ax.set_ylim([-1,2])
	plt.xlabel('frame_number')
	plt.ylabel('Z coordinate estimate (metres)')
	plt.grid()
	plt.title('Z coordinate estimate by each camera')


	# Colour the region where at least 1 camera found a pose.
	for i in range(len(region_list)):
		plt.axvspan(region_list[i][0], region_list[i][1], color='green', alpha=0.2)



	# Plot roll angle (degrees) against frame number
	plt.figure()
	plt.subplot(1,3,1)
	
	for i in range(NUM_CAMS):
		plt.plot(x, roll_estimates[i], label = 'cam'+str(i))


	ax = plt.gca()
	plt.legend()

	#ax.set_ylim([-1,2])
	plt.xlabel('frame_number')
	plt.ylabel('Roll estimate (degrees)')
	plt.grid()
	plt.title('Roll estimate by each camera')

	# Colour the region where at least 1 camera found a pose.
	for i in range(len(region_list)):
		plt.axvspan(region_list[i][0], region_list[i][1], color='green', alpha=0.2)




	# Plot pitch angle (degrees) against frame number
	plt.subplot(1,3,2)
	
	for i in range(NUM_CAMS):
		plt.plot(x, pitch_estimates[i], label = 'cam'+str(i))


	ax = plt.gca()
	plt.legend()

	#ax.set_ylim([-1,2])
	plt.xlabel('frame_number')
	plt.ylabel('Pitch estimate (degrees)')
	plt.grid()
	plt.title('Pitch estimate by each camera')


	# Colour the region where at least 1 camera found a pose.
	for i in range(len(region_list)):
		plt.axvspan(region_list[i][0], region_list[i][1], color='green', alpha=0.2)




	# Plot yaw angle (degrees) against frame number
	plt.subplot(1,3,3)
	
	for i in range(NUM_CAMS):
		plt.plot(x, yaw_estimates[i], label = 'cam'+str(i))


	ax = plt.gca()
	plt.legend()

	#ax.set_ylim([-1,2])
	plt.xlabel('frame_number')
	plt.ylabel('Yaw estimate (degrees)')
	plt.grid()
	plt.title('Yaw estimate by each camera')





	# Colour the region where at least 1 camera found a pose.
	for i in range(len(region_list)):
		plt.axvspan(region_list[i][0], region_list[i][1], color='green', alpha=0.2)


	


	
	
	summaryString_list.append('\n')
	summaryString_list.append('TODO: find mean and standard deviation = ??' )
	







def plotSummary(summaryList):
	
	# This object holds summary strings from each function called below.
	global summaryString_list

	# Various functions for plotting. Each returns a string containing some summary.
	plotFoundPose(summaryList)
	plotFramesFoundAndLost(summaryList)
	plotLatency(summaryList)
	plotStateEstimate(summaryList)

	#Display summary from each plot.
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


	



def createSummaryObj(rowList):
	# since this is common for all NUM_CAMS rows
	timestamp = int(rowList[0][0])
	camid_list = []
	foundpose_per_camera = []
	leds_per_camera = []
	confidence_list = []
	latency_list = []
	stateEstimate_list = []

	for row in rowList:
		
		camid = int(row[1]); camid_list.append(camid)
		foundPose = int(row[2]); foundpose_per_camera.append(foundPose) 
		leds = int(row[3]); leds_per_camera.append(leds)
		confidence = float(row[4]); confidence_list.append(confidence)
		latency = float(row[5]);  latency_list.append(latency)

		stateEstimate = State(row[6], row[7], row[8],
							  row[9], row[10], row[11])

		stateEstimate_list.append(stateEstimate)


	summaryObj = SummaryObj(timestamp, 
					   		camid_list,
					  	 	foundpose_per_camera,
					   		leds_per_camera,
					   		confidence_list,
					   		latency_list,
					   		stateEstimate_list)

	return summaryObj


		

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

				

		
		return summaryList




def main():

	summaryList = readSummaryFile()

	plotSummary(summaryList)


if __name__ == '__main__':
	main()
