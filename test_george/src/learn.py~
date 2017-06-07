#!/usr/bin/env python
import rospy
import george
import numpy as np
import matplotlib.pyplot as plt
from geometry_msgs.msg import PoseArray, Pose, Point, PoseStamped
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import OccupancyGrid
from std_msgs.msg import String, Header
from george.kernels import ExpSquaredKernel, ExpSine2Kernel, CosineKernel
import matplotlib.pyplot as plt
import itertools
import tf
from math import floor, sin, cos, atan2

class GaussianCrowdModel:
    def __init__(self, width, height, division):
	# set up this python class as ros node and initialize publisher and subscriber
	rospy.init_node('crowd_model')
	rospy.Subscriber("crowd_pose", PoseArray, self.crowd_data)
	rospy.Subscriber("base_scan", LaserScan, self.laser_data)
	rospy.Subscriber("pose", PoseStamped, self.pose_data)
	self.pub_crowd_model = rospy.Publisher('crowd_model', OccupancyGrid, queue_size=1)
	self.pub_visibility_grid = rospy.Publisher('visibility_grid', OccupancyGrid, queue_size=1)

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
	self.density_grid = [[0 for x in range(division)] for y in range(division)]
	self.grid_mean_spotting = [[0 for x in range(division)] for y in range(division)]
	
	# map details	
	self.width = width # width of the map
	self.height = height # height of the map
	self.division = division # number of positions in the map where the model has to predict
        self.training_x = np.array([[0,0,0]])
        self.training_y = np.array([0])
	
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
	    # convert set of poses into density_data which is a list [[x1,y1,t1,d1],[x2,y2,t2,d2]]
	    density_data = self.crowd_pose_to_density_grid(data.poses)
	    # save the current time as previous time	
	    self.prev_message_time = self.message_time
	    # add the density_data to the dataset by updating self.training_y and self.training_x 
	    self.update_training_set(density_data)
	    #print self.training_x, self.training_y
	    #print len(self.training_x), len(self.training_y)
	    # predict along a map grid for the current time using the gaussian process model
	    self.predict()

    # update the visiblity of grid map based on laser data from the robot
    def update_visibility(self):
	print "in update visibility"
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

    def crowd_pose_to_density(self,poses):
        density_data = []
	# for every crowd pose compute the density 
        for i in poses:
            x = i.position.x
	    y = i.position.y
	    d = 1
	    # density computation using every other crowd pose and its distance from the current pose
            for j in poses:
	        if i != j:
	            d = d + (40.0/self.distance(x,y,j.position.x,j.position.y))
	    density_data.append([x,y,self.time,d])
        return density_data
    
    def crowd_pose_to_density_grid(self,poses):
	v_cells = self.division
	h_cells = self.division
	cell_width = self.width/self.division
	cell_height = self.height/self.division
	self.density_grid = [[0 for x in range(h_cells)] for y in range(v_cells)]
	#for every grid position compute the density
	for i in poses:
	    x = i.position.x
	    y = i.position.y
            x_index = int(floor(x / cell_width))
	    y_index = int(floor(y / cell_height))
	    self.density_grid[x_index][y_index] += 1
	print "finished filling up density grids" 
	density_data = [[x*cell_width,y*cell_height,int(floor(self.time)),self.density_grid[x][y]] for x in range(h_cells) for y in range(v_cells) if self.is_grid_active[x][y]]  
	print "returning density data"
        return density_data

    #def mean(self):
	
		
    def predict(self):
	# set up the kernel and the mean of the gaussian process
        k1 = ExpSquaredKernel(1.0,ndim=3)
	k2 = ExpSine2Kernel(1,15,ndim=3,dim=0)
	k3 = ExpSine2Kernel(1,15,ndim=3,dim=1)
	k4 = ExpSine2Kernel(1,1,ndim=3,dim=2)
        self.kernel = k1
        self.gp = george.GP(self.kernel, mean=np.mean(self.training_y),solver=george.HODLRSolver)
	# run the compute functions
	print "size of training input ",len(self.training_x)
	print "size of training output",len(self.training_y)
	#print "training features", self.training_x
	#print "training output", self.training_y
    	self.gp.compute(self.training_x,1e-5)
	#print "gp computation completed"
	# create a set of test set input to query for density estimate across the map (width * height)
	testing_x = np.array([[x,y,self.time] for y,x in itertools.product(np.linspace(0,self.width,self.division),np.linspace(0,self.height,self.division))])
	print "Size of test features", len(testing_x)
	#print "test features", testing_x
	# run the gp to predict for current state estimate
    	mu, cov = self.gp.predict(self.training_y, testing_x)
	self.publish_crowd_model(mu, cov)
	self.publish_visibility_grid()
        
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

    def publish_crowd_model(self,mu,cov):
	crowd_model = OccupancyGrid()
	crowd_model.header.stamp = rospy.Time.now()
	crowd_model.header.frame_id = "map"
	crowd_model.info.resolution = self.height/self.division
	crowd_model.info.width = self.division
	crowd_model.info.height = self.division
	crowd_model.info.origin.orientation.w = 1
	data = list(mu)
	print data
	max_density = max(data)
	clip = lambda x: int((x > 0) * x)

	if max_density == 0:
	    crowd_model.data = [clip(x) for x in data]
	else:
	    # function that forces negative values to zero
	    data_normed = [clip(x*100.0/max_density) for x in data]
	    crowd_model.data = data_normed 
	self.pub_crowd_model.publish(crowd_model)

    def distance(self, x1,y1,x2,y2):
	return ((x1-x2)**2 + (y1-y2)**2) ** 0.5 

    def update_training_set(self, density_data):
	#print "Density data" , density_data
        for data in density_data:
	    self.training_x = np.vstack([self.training_x,data[:-1]])
	    self.training_y = np.append(self.training_y,data[-1])


crowd_model = GaussianCrowdModel(40,40,10)
crowd_model.listen()

