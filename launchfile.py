# Author: Anoop Aroor
# This is a python launch file to control and launch ros nodes for Semaforr project
# Each ros node must has a launch file containing parameters that remain constant though different experiments
# Dynamic parameters that changes with different experiments are added here

import rospy
import time
import subprocess

def experiment():
    project_home = "/home/anoop/catkin_ws/src"
    menge_path = project_home+"/examples/core"
    semaforr_path = project_home+"/semaforr"

    map_folder = menge_path+"/"+map_name
    map_xml = menge_path+"/"+map_name+".xml"

    #menge files for semaforr
    map_config = map_folder+"/"+map_name+"S.xml"
    map_dimensions = map_folder+"/dimensions.conf"
    target_set = map_folder+"/" + target_file_name
    density = "on"
    flow = "on"
    risk = "off"
    cusum = "off"
    discount = "off"
    explore = "off"

    print target_set
    print map_config
    print map_xml
    print map_dimensions
    print log_name
    #start roscore
    roscore = subprocess.Popen(['roscore'])
    time.sleep(1)

    # start menge simulator
    menge_sim_process = subprocess.Popen(['rosrun','menge_sim','menge_sim','-p',map_xml])
    print "waiting,,"
    time.sleep(10)
    if mode == 1:
	print "Starting crowd model with CSA* "
        crowd_process = subprocess.Popen(['rosrun','crowd_count','learn.py'])
    if mode == 2:
	print "Starting crowd model with CUSUM-A* "
        crowd_process = subprocess.Popen(['rosrun','crowd_cusum','learn.py'])
    if mode == 3:
	print "Starting crowd model with Risk-A* "
        crowd_process = subprocess.Popen(['rosrun','crowd_behavior','learn.py'])
    if mode == 4:
	print "Starting crowd model with Thompson-A* "
        crowd_process = subprocess.Popen(['rosrun','crowd_count_thompson','learn.py'])
    if mode == 5:
        crowd_process = subprocess.Popen(['rosrun','crowd_learner','learn.py',density, flow, risk, cusum, discount, explore])

    log_file = open(log_name,"w")
    log_process = subprocess.Popen(['rostopic','echo','/decision_log'],stdout=log_file)

    # start semaforr
    semaforr_process = subprocess.Popen(['rosrun','semaforr','semaforr', semaforr_path, target_set, map_config, map_dimensions])
    print "waiting,,"


    # Wait till semaforr completes the process
    while semaforr_process.poll() is None:
        print "Semaforr process still running ..."
        time.sleep(1)

    print "Semaforr process has ended ..."
    print "Terminating the simulator"

    menge_sim_process.terminate()
    while menge_sim_process.poll() is None:
        print "Menge process still running ..."
    	time.sleep(1)

    print "Menge terminated!"
    if mode == 1 or mode == 2 or mode == 3 or mode == 4:
	print "Terminating crowd model"
        crowd_process.terminate()
    log_process.terminate()
    log_file.close()
    time.sleep(1)
    #why_process.terminate()
    #print "Why terminated!"

    roscore.terminate()
    time.sleep(10)
    print "roscore terminated!"

map_name = "gradcenter-4-static"
mode = 5

for i in range(10,20):
    target_file_name = "target.conf"
    log_name = map_name + "_" + "flow-csastar" + "_" + "orca" + "_" + "crowdsize60" + "_" + "trial" + str(i) + ".txt"
    experiment()
