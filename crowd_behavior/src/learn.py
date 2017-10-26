#!/usr/bin/env python
import rospy
import numpy as np
import matplotlib.pyplot as plt
from geometry_msgs.msg import PoseArray, Pose, Point, PoseStamped
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import OccupancyGrid
from semaforr.msg import CrowdModel
from visualization_msgs.msg import Marker
from visualization_msgs.msg import MarkerArray
from std_msgs.msg import String, Header
import itertools
import tf
from math import floor, sin, cos, atan2

class CrowdBehaviorModel:
    def __init__(self, width, height, division):
	# set up this python class as ros node and initialize publisher and subscriber
	rospy.init_node('crowd_model')
	rospy.Subscriber("crowd_pose", PoseArray, self.crowd_data)
	rospy.Subscriber("base_scan", LaserScan, self.laser_data)
	rospy.Subscriber("pose", PoseStamped, self.pose_data)
	self.pub_crowd_density = rospy.Publisher('crowd_density', OccupancyGrid, queue_size=1)
	self.pub_crowd_u = rospy.Publisher('crowd_u', MarkerArray, queue_size=1)
	self.pub_crowd_d = rospy.Publisher('crowd_d', MarkerArray, queue_size=1)
	self.pub_crowd_l = rospy.Publisher('crowd_l', MarkerArray, queue_size=1)
	self.pub_crowd_r = rospy.Publisher('crowd_r', MarkerArray, queue_size=1)
	self.pub_crowd_ur = rospy.Publisher('crowd_ur', MarkerArray, queue_size=1)
	self.pub_crowd_ul = rospy.Publisher('crowd_ul', MarkerArray, queue_size=1)
	self.pub_crowd_dr = rospy.Publisher('crowd_dr', MarkerArray, queue_size=1)
	self.pub_crowd_dl = rospy.Publisher('crowd_dl', MarkerArray, queue_size=1)
	self.pub_crowd_model = rospy.Publisher('crowd_model', CrowdModel, queue_size=1)
	self.pub_visibility_grid = rospy.Publisher('visibility_grid', OccupancyGrid, queue_size=1)
	self.pub_risk_grid = rospy.Publisher('risk_grid', OccupancyGrid, queue_size=1)

	# intialize time
        self.prev_message_time = rospy.Time.now()
	self.message_time = rospy.Time.now()
	self.init_time = rospy.Time.now()
	self.time = (self.message_time - self.init_time)

	# sensor data (laser scan, robot pose, crowd poses)
	self.crowd_poses = PoseArray()
	# robot current pose is x,y,theta
	self.robot_pose = [0,0,0]
	self.laser_scan_endpoints = []
	# indicates if the robot has access to the crowd information in that grid
	self.is_grid_active = [[False for x in range(division)] for y in range(division)]
	self.crowd_count = [[0 for x in range(division)] for y in range(division)]
	self.crowd_u = [[0 for x in range(division)] for y in range(division)]
	self.crowd_d = [[0 for x in range(division)] for y in range(division)]
	self.crowd_l = [[0 for x in range(division)] for y in range(division)]
	self.crowd_r = [[0 for x in range(division)] for y in range(division)]

	self.crowd_ul = [[0 for x in range(division)] for y in range(division)]
	self.crowd_ur = [[0 for x in range(division)] for y in range(division)]
	self.crowd_dl = [[0 for x in range(division)] for y in range(division)]
	self.crowd_dr = [[0 for x in range(division)] for y in range(division)]
	
	self.crowd_observations = [[1 for x in range(division)] for y in range(division)]
	self.crowd_experiences = [[1 for x in range(division)] for y in range(division)]
	self.crowd_risk_count = [[0 for x in range(division)] for y in range(division)]
	
	# map details	
	self.width = width # width of the map
	self.height = height # height of the map
	self.division = division # number of positions in the map where the model has to predict
        
    # calls the callback for each of the subscriber
    def listen(self):
        rospy.spin()
    
    # receive the laser data in standard format and save as a sequence of endpoints
    def laser_data(self, data):
	#print "receiveing laser_scan data message"
	self.laser_scan_endpoints = []
	angle = data.angle_min
	increment = data.angle_increment
        for laser_distance in data.ranges:
	    x = self.robot_pose[0] + laser_distance * cos(self.robot_pose[2] + angle)
	    y = self.robot_pose[1] + laser_distance * sin(self.robot_pose[2] + angle)
	    angle += increment
	    self.laser_scan_endpoints.append([x,y])
	

    # receive and save the robot pose data
    def pose_data(self, data):
	#print "receiveing robot pose data message"
	quaternion = (data.pose.orientation.x, data.pose.orientation.y, data.pose.orientation.z, data.pose.orientation.w)
	euler = tf.transformations.euler_from_quaternion(quaternion)
	roll = euler[0]
	pitch = euler[1]
	yaw = euler[2]
	self.robot_pose[0] = data.pose.position.x
	self.robot_pose[1] = data.pose.position.y
	self.robot_pose[2] = yaw
    
    # recieve and save the crowd data, update the dataset, publish the new crowd model
    def crowd_data(self, data):
	#print "receiveing crowd data message"
        self.message_time = data.header.stamp
	# elapsed time computes the time since the previous message
	elapsed_time = self.message_time - self.prev_message_time
	# accept crowd data if the last data received was more than 1 seconds earlier
        if elapsed_time.to_sec() > 1:
	    self.update_visibility()
	    # time in seconds since the begin of program, used as a feature in the GP
	    self.time = (self.message_time - self.init_time).to_sec()
	    self.crowd_pose_to_density_grid(data.poses)
	    self.crowd_risk_estimation(data.poses)
	    # save the current time as previous time	
	    self.prev_message_time = self.message_time
	    self.predict()

    # estimates the number of risky action < 0.5 in a given grid cell
    def crowd_risk_estimation(self, poses):
	rx = self.robot_pose[0]
	ry = self.robot_pose[1]
	count = 0
	cell_width = self.width/self.division
	cell_height = self.height/self.division
	for i in poses:
	    quaternion = (i.orientation.x, i.orientation.y, i.orientation.z, i.orientation.w)
	    euler = tf.transformations.euler_from_quaternion(quaternion)
	    theta = euler[2]
	    x = i.position.x
	    y = i.position.y
	    if self.distance(x,y,rx,ry) < 1:
	        count += 1
	rx_index = int(floor(rx / cell_width))
	ry_index = int(floor(ry / cell_height))
	self.crowd_experiences[rx_index][ry_index] += 1
	self.crowd_risk_count[rx_index][ry_index] += count


    # update the visiblity of grid map based on laser data from the robot
    def update_visibility(self):
	#print "in update visibility"
	# reset visibility to false by default
	self.is_grid_active = [[False for x in range(self.division)] for y in range(self.division)]
	# for every laser scan
	cell_width = self.width/self.division
	cell_height = self.height/self.division
        for endpoint in self.laser_scan_endpoints:
	    laser_distance = self.distance(endpoint[0],endpoint[1],self.robot_pose[0],self.robot_pose[1])
	    # walk along the laser scan and mark grid points as visible
	    test_x = self.robot_pose[0] 
	    test_y = self.robot_pose[1]
	    x_diff = (endpoint[0] - self.robot_pose[0])
	    y_diff = (endpoint[1] - self.robot_pose[1])
	    x_increment = cell_width * cos(atan2(y_diff,x_diff))
	    y_increment = cell_height * sin(atan2(y_diff,x_diff))
	    # as long as the walk does not do beyond the laser scan
	    #print "Robot position:", self.robot_pose[0], self.robot_pose[1]
	    #print "Laser endpoint:", endpoint[0],endpoint[1]
	    while self.distance(test_x,test_y,self.robot_pose[0],self.robot_pose[1]) < laser_distance:
		self.is_grid_active[int(floor(test_x/cell_width))][int(floor(test_y/cell_height))] = True
	        test_x += x_increment/2
	        test_y += y_increment/2
		#print test_x, test_y
        #print self.is_grid_active

    def crowd_pose_to_density_grid(self,poses):
	v_cells = self.division
	h_cells = self.division
	cell_width = self.width/self.division
	cell_height = self.height/self.division
	current_sample = [[0 for x in range(self.division)] for y in range(self.division)]
	#for every grid position compute the density
	for i in poses:
	    quaternion = (i.orientation.x, i.orientation.y, i.orientation.z, i.orientation.w)
	    euler = tf.transformations.euler_from_quaternion(quaternion)
	    theta = euler[2]
	    x = i.position.x
	    y = i.position.y
	    pi = 3.1416
            x_index = int(floor(x / cell_width))
	    y_index = int(floor(y / cell_height))
	    if(self.is_grid_active[x_index][y_index] == True):
	        self.crowd_count[x_index][y_index] += 1
		current_sample[x_index][y_index] += 1
		#print "->", x, y, theta
		if((theta < (pi/8)) and (theta >= (-pi/8))):
		    #print "right"
		    self.crowd_r[x_index][y_index] += 1
		if((theta < ((3*pi)/8)) and (theta >= (pi/8))):
		    #print "up-right"
		    self.crowd_ur[x_index][y_index] += 1
		if((theta < ((5*pi)/8)) and (theta >= ((3*pi)/8))):
		    #print "up"
		    self.crowd_u[x_index][y_index] += 1
		if((theta < ((7*pi)/8)) and (theta >= ((5*pi)/8))):
		    #print "up-left"
		    self.crowd_ul[x_index][y_index] += 1
		if( ((theta < (-(7*pi)/8)) and theta >= -pi) or (theta >= (7*pi/8) and theta <= pi)):
		    #print "left"
		    self.crowd_l[x_index][y_index] += 1
		if((theta < (-(5*pi)/8)) and (theta >= (-(7*pi)/8))):
		    #print "down-left"
		    self.crowd_dl[x_index][y_index] += 1
		if((theta < (-(3*pi)/8)) and (theta >= (-(5*pi)/8))):
		    #print "down"
		    self.crowd_d[x_index][y_index] += 1
		if((theta < (-pi/8)) and (theta >= (-(3*pi)/8))):
		    #print "down-right"
		    self.crowd_dr[x_index][y_index] += 1
	#print "~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~" 
	#print "finished counting crowd:" 
	#print self.crowd_count
	for x in range(h_cells):
	    for y in range(v_cells):
                if self.is_grid_active[x][y]:
	            self.crowd_observations[x][y] += 1    
	#print "finished counting observations:"
	#print self.crowd_observations
	#print "~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~" 


    def printState(self):
        for x in range(self.division):
	    for y in range(self.division):
		print x,y,self.is_grid_active[x][y], self.crowd_observations[x][y], self.crowd_count[x][y], self.change_detected[x][y]

    def reset_grid(self, x, y):
	self.cusum_up[x][y].reset()
	self.cusum_down[x][y].reset()
	self.crowd_count[x][y] = 0
	self.crowd_u[x][y] = 0
	self.crowd_d[x][y] = 0
	self.crowd_l[x][y] = 0
	self.crowd_r[x][y] = 0
	self.crowd_ul[x][y] = 0
	self.crowd_ur[x][y] = 0
	self.crowd_dl[x][y] = 0
	self.crowd_dr[x][y] = 0
	self.crowd_observations[x][y] = 1
		
    def predict(self):
        crowd_count_model = [[(self.crowd_count[x][y]/float(self.crowd_observations[x][y])) for x in range(self.division)] for y in range(self.division)]
	u_model = [[(self.crowd_u[x][y]/float(self.crowd_observations[x][y])) for x in range(self.division)] for y in range(self.division)]
	d_model = [[(self.crowd_d[x][y]/float(self.crowd_observations[x][y])) for x in range(self.division)] for y in range(self.division)]
	r_model = [[(self.crowd_r[x][y]/float(self.crowd_observations[x][y])) for x in range(self.division)] for y in range(self.division)]
	l_model = [[(self.crowd_l[x][y]/float(self.crowd_observations[x][y])) for x in range(self.division)] for y in range(self.division)]
	ul_model = [[(self.crowd_ul[x][y]/float(self.crowd_observations[x][y])) for x in range(self.division)] for y in range(self.division)]
	ur_model = [[(self.crowd_ur[x][y]/float(self.crowd_observations[x][y])) for x in range(self.division)] for y in range(self.division)]
	dl_model = [[(self.crowd_dl[x][y]/float(self.crowd_observations[x][y])) for x in range(self.division)] for y in range(self.division)]
	dr_model = [[(self.crowd_dr[x][y]/float(self.crowd_observations[x][y])) for x in range(self.division)] for y in range(self.division)]
	# this line converts the crowd count model into risk adjusted crowd count model
	crowd_count_model = [[((self.crowd_risk_count[x][y]+crowd_count_model[y][x])/float(self.crowd_experiences[x][y])) for x in range(self.division)] for y in range(self.division)]
	crowd_risk_model = [[(self.crowd_risk_count[x][y]/float(self.crowd_experiences[x][y])) for x in range(self.division)] for y in range(self.division)]
	self.publish_visibility_grid()
	self.publish_crowd_model(crowd_count_model, u_model, d_model, r_model, l_model, ul_model, ur_model, dl_model, dr_model)
	pi = 3.1416
	self.publish_crowd_flow(u_model, pi/2, self.pub_crowd_u)
	self.publish_crowd_flow(d_model, -pi/2, self.pub_crowd_d)
	self.publish_crowd_flow(r_model, 0, self.pub_crowd_r)
	self.publish_crowd_flow(l_model, pi, self.pub_crowd_l)
	self.publish_crowd_flow(ur_model, pi/4, self.pub_crowd_ur)
	self.publish_crowd_flow(ul_model, 3*pi/4, self.pub_crowd_ul)
	self.publish_crowd_flow(dr_model, -pi/4, self.pub_crowd_dr)
	self.publish_crowd_flow(dl_model, -3*pi/4, self.pub_crowd_dl)
	self.publish_crowd_risk_model(crowd_risk_model) 

    def publish_crowd_risk_model(self, crowd_risk_model):
	risk_grid = OccupancyGrid()
	risk_grid.header.stamp = rospy.Time.now()
	risk_grid.header.frame_id = "map"
	risk_grid.info.resolution = self.height/self.division
	risk_grid.info.width = self.division
	risk_grid.info.height = self.division
	risk_grid.info.origin.orientation.w = 1
	risk_list = [crowd_risk_model[x][y] for x in range(self.division) for y in range(self.division)]
	risk_grid.data = self.normalize(risk_list, 0, 100)  
	self.pub_risk_grid.publish(risk_grid)
    
    def publish_crowd_flow(self, u, angle, publisher):
	markerArray = MarkerArray();
	for x in range(self.division):
	    for y in range(self.division):
		offset = 1.5
	        marker = Marker()	
	        marker.header.frame_id = "map"
    	        marker.header.stamp = rospy.Time.now()
	        marker.ns = "basic_shapes"
    	        marker.id = (x * self.division) + y
	        marker.type = 0
    	        marker.pose.position.x = (x * (self.width/self.division)) + offset
    	        marker.pose.position.y = (y * (self.height/self.division)) + offset
    	        marker.pose.position.z = 0
    	        quaternion = tf.transformations.quaternion_from_euler(0, 0, angle)
                #type(pose) = geometry_msgs.msg.Pose
                marker.pose.orientation.x = quaternion[0]
                marker.pose.orientation.y = quaternion[1]
                marker.pose.orientation.z = quaternion[2]
                marker.pose.orientation.w = quaternion[3]

    	        # Set the scale of the marker -- 1x1x1 here means 1m on a side
    	        marker.scale.x = 0.2 + (u[y][x]*6)
		marker.scale.y = 0.2
		marker.scale.z = 1

    	        # Set the color -- be sure to set alpha to something non-zero!
    	        marker.color.r = 0.0
    	        marker.color.g = 1.0
    	        marker.color.b = 0.0
    	        marker.color.a = 0.5
		markerArray.markers.append(marker)		
	publisher.publish(markerArray);

    def publish_change_grid(self):
	change_grid = OccupancyGrid()
	change_grid.header.stamp = rospy.Time.now()
	change_grid.header.frame_id = "map"
	change_grid.info.resolution = self.height/self.division
	change_grid.info.width = self.division
	change_grid.info.height = self.division
	change_grid.info.origin.orientation.w = 1
	change_grid.data = [(self.change_detected[x][y])*25 for y in range(self.division) for x in range(self.division)]
	self.pub_change_grid.publish(change_grid)
        
    def publish_visibility_grid(self):
	visible_grid = OccupancyGrid()
	visible_grid.header.stamp = rospy.Time.now()
	visible_grid.header.frame_id = "map"
	visible_grid.info.resolution = self.height/self.division
	visible_grid.info.width = self.division
	visible_grid.info.height = self.division
	visible_grid.info.origin.orientation.w = 1
	visible_grid.data = [self.convert(self.is_grid_active[x][y]) for y in range(self.division) for x in range(self.division)]
	self.pub_visibility_grid.publish(visible_grid)

    def convert(self, isActive):
	if isActive:
	    return 100
	else:
	    return 0

    def publish_crowd_model(self, density, u, d, l, r, ul, ur, dl, dr):
	crowd_model = CrowdModel()
	crowd_model.header.stamp = rospy.Time.now()
	crowd_model.header.frame_id = "map"
	crowd_model.resolution = self.height/self.division
	crowd_model.width = self.division
	crowd_model.height = self.division

	density_list = [density[x][y] for x in range(self.division) for y in range(self.division)]
	#print density_list
	#density_list = self.normalize_float(density_list, 0, 1)
	#print density_list
	crowd_model.densities = density_list
	
	u_list = [u[x][y] for x in range(self.division) for y in range(self.division)]
	u_list = self.normalize_float(u_list, 0, 1)
	crowd_model.up = u_list

	d_list = [d[x][y] for x in range(self.division) for y in range(self.division)]
	d_list = self.normalize_float(d_list, 0, 1)
	crowd_model.down = d_list

	r_list = [r[x][y] for x in range(self.division) for y in range(self.division)]
	r_list = self.normalize_float(r_list, 0, 1)
	crowd_model.right = r_list

	l_list = [l[x][y] for x in range(self.division) for y in range(self.division)]
	l_list = self.normalize_float(l_list, 0, 1)
	crowd_model.left = l_list

	ul_list = [ul[x][y] for x in range(self.division) for y in range(self.division)]
	ul_list = self.normalize_float(ul_list, 0, 1)
	crowd_model.up_left = ul_list

	dl_list = [dl[x][y] for x in range(self.division) for y in range(self.division)]
	dl_list = self.normalize_float(dl_list, 0, 1)
	crowd_model.down_left = dl_list

	dr_list = [dr[x][y] for x in range(self.division) for y in range(self.division)]
	dr_list = self.normalize_float(dr_list, 0, 1)
	crowd_model.down_right = dr_list

	ur_list = [ur[x][y] for x in range(self.division) for y in range(self.division)]
	ur_list = self.normalize_float(ur_list, 0, 1)
	crowd_model.up_right = ur_list
	
	self.pub_crowd_model.publish(crowd_model)

        # send another message for rviz display
	crowd_rviz = OccupancyGrid()
	crowd_rviz.header.stamp = rospy.Time.now()
	crowd_rviz.header.frame_id = "map"
	crowd_rviz.info.resolution = self.height/self.division
	crowd_rviz.info.width = self.division
	crowd_rviz.info.height = self.division
	crowd_rviz.info.origin.orientation.w = 1
	density_rviz = [density[x][y] for x in range(self.division) for y in range(self.division)]
	crowd_rviz.data = self.normalize(density_rviz, 0, 100)  
	#print "Normalized Crowd model sent for rviz:"
	#print crowd_rviz.data 
	self.pub_crowd_density.publish(crowd_rviz)
	

    def normalize(self, list_of_floats, minimum, maximum):
	max_number = max(list_of_floats)
	#print max_number
	if max_number == 0:
	    return [int(x) for x in list_of_floats]
	else:
	    return [int((x/max_number)*maximum) for x in list_of_floats]

    
    def normalize_float(self, list_of_floats, minimum, maximum):
	#print "in normalization function" 
	max_number = max(list_of_floats)
	#print max_number
	if max_number == 0:
	    return [x for x in list_of_floats]
	else:
	    return [(x/max_number)*maximum for x in list_of_floats]


    def distance(self, x1,y1,x2,y2):
	return ((x1-x2)**2 + (y1-y2)**2) ** 0.5 


crowd_model = CrowdBehaviorModel(120,120,60)
crowd_model.listen()

