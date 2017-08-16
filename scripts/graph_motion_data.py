#!/usr/bin/env python
from remyJr.msg import remyjr_graphs
import rospy
import time
import math
import matplotlib.pyplot as plt
import numpy as np

#holds the x and y coordinates of unit vectors representing each direction a sonar is facing
sonar_x_cords = [0.0, 1/1.4142, 1.0, 1/1.4142, 0.0, -1/1.4142, -1.0, -1/1.4142, 0.0]
sonar_y_cords = [1.0, 1/1.4142, 0.0, -1/1.4142, -1.0, -1/1.4142, 0.0, 1/1.4142, 1.0]

num_of_sonar = 8

close_quarters_queue_len = 10
close_quarters_x_vals = np.arange(close_quarters_queue_len)


def setup():
	global receivingData, sonar_dis, x_cord, y_cord, x_avg, y_avg, edgeDis, deadzone, tooCloseDis
	global close_quarters_queue, close_quarters, close_quarters_edgefinding_lim
	close_quarters_edgefinding_lim = 0
	close_quarters = 0
	close_quarters_queue = np.zeros(close_quarters_queue_len)
	sonar_dis = [0]*num_of_sonar #holds the distances for each sonar
	x_cord = [0]*num_of_sonar #holds the x-cordinates for objects detected by each sonar
	y_cord = [0]*num_of_sonar #holds the y-cordinates for objects deteced by each sonar
	x_avg = 0 #weighted average of the x-coordinates of the sonar data, received from another node
	y_avg = 0 #weighted average of the y-coordinates of the sonar data, received from another node
	#these are all received from another node and represent the radius of different zones used in navigating
	edgeDis = 0 
	deadzone = 0
	tooCloseDis = 0
	receivingData = 0 #tracks whether this node has received data from the driving node
	plt.ion()
        fig = plt.figure(figsize=(16,20), dpi=40)
	findRings() #calculates the coordinates that represent the different zones used in navigating
	
def moveQueue(queue, newest_entry):
	for i in range(len(queue)-1):
		queue[i] = queue[i+1]
 	queue[len(queue)-1] = newest_entry
	return queue

def getRingCordX(dis): #calcluates the x-cordinates representing a radius around the robot
	cord = [0]*(num_of_sonar+1)
	for i in range(len(cord)):
		cord[i] = dis * sonar_x_cords[i] 
	return cord

def getRingCordY(dis): #calculates the y-coordinates representing a radius around the robot
        cord = [0]*(num_of_sonar + 1)
        for i in range(len(cord)):
                cord[i] = dis * sonar_y_cords[i]
        return cord

def findRings(): #calculates the coordiates representing various regions around the robot
	global deadzone_x, deadzone_y, receivingData, tooClose_x, tooClose_y, edge_x, edge_y, edgefinding_lim_y
	deadzone_x = getRingCordX(deadzone)
        deadzone_y = getRingCordY(deadzone)
	tooClose_x = getRingCordX(tooCloseDis)
        tooClose_y = getRingCordY(tooCloseDis)
        edge_x = getRingCordX( edgeDis)
        edge_y = getRingCordY(edgeDis)
	edgefinding_lim_y = np.ones(close_quarters_queue_len)*close_quarters_edgefinding_lim 
	if (deadzone != 0) & (receivingData < 3):
		receivingData = receivingData + 1

def plotData(): #plots the data
	print ("plotting...")
#	global x_avg_record, y_avg_record
	plt.clf()
	print edgefinding_lim_y

        plt.subplot2grid((5,1),(0,0), rowspan=1, colspan=1)
        plt.xlim(0,close_quarters_queue_len)
        plt.ylim(0,1)
        plt.plot(close_quarters_x_vals, edgefinding_lim_y, color="red")
        plt.plot(close_quarters_x_vals, close_quarters_queue)


	plt.subplot2grid((5,1), (1,0), rowspan=4, colspan=1)
        plt.xlim(-1*edgeDis - 25.0, edgeDis + 25.0)
        plt.ylim(-1*edgeDis - 25.0, edgeDis + 25.0)
	plt.scatter(x_cord, y_cord, color="blue", linewidth=10)
	plt.plot(deadzone_x, deadzone_y, color="red")
	plt.plot([x_cord[0], 0], [y_cord[0], 0], color="green") 
	plt.plot(tooClose_x, tooClose_y, color="yellow")
	plt.plot(edge_x, edge_y, color="green")
	plt.scatter(x_avg, y_avg, color="red", linewidth=15)

	plt.draw()

	
def motionDataListener(data):
	global x_cord, y_cord, x_avg, y_avg,close_quarters, deadzone, tooCloseDis, edgeDis, close_quarters_edgefinding_lim
	x_cord = data.x_cords
	y_cord = data.y_cords
	x_avg = data.x_avg
	y_avg = data.y_avg
	close_quarters = data.close_quarters
	if receivingData < 3: #these values are constant once the driving node is initalized, so it only updates them the first few times
		deadzone = data.deadzone
		tooCloseDis = data.tooCloseDis
		edgeDis = data.edgeDis
		close_quarters_edgefinding_lim = data.close_quarters_edgefinding_lim
		
def listener(): #listens for data from the driving node
	print ('Listening...')
	rospy.Subscriber("remyjr/motion_data", remyjr_graphs, motionDataListener, queue_size = 1)

def mainLoop():
	global close_quarters_queue
	rospy.init_node('remyjr_graphs')
	while not rospy.is_shutdown():
		if receivingData < 3: #only does these calculations the first few times it receives data from the drive node
			findRings()
		listener()
		close_quarters_queue = moveQueue(close_quarters_queue, close_quarters) 
		plotData()
		time.sleep(0.15)

if __name__ == "__main__":
	print('Setting up...')
        setup()
	print ('Setup Complete!')
        try:
		mainLoop()        
        except KeyboardInterrupt:
		pass



