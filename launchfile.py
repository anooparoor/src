# Author: Anoop Aroor
# This is a python launch file to control and launch ros nodes for Semaforr project
# Each ros node must has a launch file containing parameters that remain constant though different experiments
# Dynamic parameters that changes with different experiments are added here

import rospy
import time
import subprocess

def experiment(density, flow, risk, cusum, explore, discount):
    project_home = "/home/anoop/catkin_ws/src"
    menge_path = project_home+"/examples/core"
    semaforr_path = project_home+"/semaforr"

    scenario_folder = menge_path+"/"+scenario_name
    scenario_xml = menge_path+"/"+scenario_name+".xml"

    #menge files for semaforr
    scene_config = scenario_folder+"/"+scenario_name+"S.xml"
    map_dimensions = scenario_folder+"/dimensions.conf"
    target_set = scenario_folder+"/target.conf"
    
    print target_set
    print scene_config
    print scenario_xml
    print map_dimensions
    print log_name
    #start roscore
    roscore = subprocess.Popen(['roscore'])
    time.sleep(1)

    # start menge simulator
    menge_sim_process = subprocess.Popen(['rosrun','menge_sim','menge_sim','-p',scenario_xml])
    print "waiting,,"
    time.sleep(10)
    crowd_process = subprocess.Popen(['rosrun','crowd_learner','learn.py',density, flow, risk, cusum, explore, discount])

    log_file = open(log_name,"w")
    log_process = subprocess.Popen(['rostopic','echo','/decision_log'],stdout=log_file)

    # start semaforr
    semaforr_process = subprocess.Popen(['rosrun','semaforr','semaforr', semaforr_path, target_set, scene_config, map_dimensions])
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

    log_process.terminate()
    log_file.close()
    time.sleep(1)
    #why_process.terminate()
    #print "Why terminated!"

    roscore.terminate()
    time.sleep(10)
    print "roscore terminated!"


density = "on"
flow = "off"
risk = "off"
cusum = "off"
explore = "off"
# dont forget
discount = 1
scenario_name = "gradcenter-4-static"
trials = 1


for i in range(trials):
    log_name = scenario_name + "-" + "trial" + str(i) + ".txt"
    experiment(density, flow, risk, cusum, explore, discount)
