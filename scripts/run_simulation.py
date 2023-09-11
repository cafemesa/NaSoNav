import os
import signal
import subprocess
import rospy
import math
import csv
import time
from gazebo_msgs.msg import ModelStates
from rosgraph_msgs.msg import Clock

class SuccessRate():
    pedestrian_collisions = 0
    environment_collisions = 0
    timeout = 0

# Modify the following variables
sceneIds = [0, 1, 2, 3, 4, 5, 6, 7, 8, 9] # Scenes IDs to be runned
speedsValues = [0.3, 0.4, 0.5, 0.7, 1.0, 1.5] # Speed values to be runned
agentsIds = [7, 48, 77, 124, 150, 159, 189, 226, 273, 303] # Agents IDs to store metrics
personal_area_radius = 1.2 # To store Time in personal area
distance_collition = 0.45 # Distance to another person to count as collision
agent_collition_ids = [] # Arrau with the agents ID that the agent selected collide 
success_results = [[SuccessRate() for i in range(len(speedsValues))] for j in range(len(sceneIds))]
current_scene = 0 
current_speed = 0

# Define global variables
model_names = []
model_poses = []
pro = None  # Define the process variable globally to terminate it later

# Model Callback to get data Gazebo actors information
def model_states_callback(msg):
    global model_names
    global model_poses
    model_names = msg.name
    model_poses = msg.pose

def distance_between_points(x1, y1, x2, y2):
    return math.sqrt((x2 - x1)**2 + (y2 - y1)**2)

# Clock callback and kill process when actor reach goal
def clock_callback(msg):
    global pro, model_names, model_poses, current_scene, current_speed 
    current_time = msg.clock.to_sec()
    model_name = "actor" + str(agentsIds[current_scene])
    model_position = None

    # Find the z-dimension position of the current model in the ModelStates message
    for name, pose in zip(model_names, model_poses):
        if name == model_name:
            model_position = pose.position
            break
    
    # Check for collisions
    if current_time > 140 and pro:
        for name, pose in zip(model_names, model_poses):
            if name != model_name and 'actor' in name  and pose.position.z is not None and pose.position.z > 1:
                current_distance = distance_between_points(pose.position.x,pose.position.y, model_position.x, model_position.y)
                if current_distance < distance_collition and not (name in agent_collition_ids):
                    agent_collition_ids.append(name)
                    success_results[current_scene][current_speed].pedestrian_collisions = success_results[x][y].pedestrian_collisions+1

    # Check if the clock is more than 140 seconds and the z-dimension of the model is less than 1 meter
    if current_time > 143 and model_position.z is not None and model_position.z < 1:
        if pro:
            time.sleep(15)
            os.killpg(os.getpgid(pro.pid), signal.SIGTERM)
            pro = None
    elif current_time > 320:
        if pro:
            success_results[current_scene][current_speed].timeout = 1
            time.sleep(15)
            os.killpg(os.getpgid(pro.pid), signal.SIGTERM)
            pro = None
    

# Method to run the roslaunch file
def run_process(scene, agent_id):
    global pro, personal_area_radius
    roslaunch_cmd = f"roslaunch social_navigation_testbed store_metrics_model.launch world:={scene} actorId:={agent_id} personal_area_radius:={personal_area_radius} distance_collition:={distance_collition}"
    pro = subprocess.Popen(
        roslaunch_cmd, stdout=subprocess.PIPE, shell=True, preexec_fn=os.setsid)
    out, err = pro.communicate()  # Capture the process output

# Main function. ros node init, subscribers and array processing to launch the simulation
if __name__ == "__main__":
    rospy.init_node('socialNavSimulator', anonymous=True)
    rospy.Subscriber("/gazebo/model_states",
                     ModelStates, model_states_callback)
    rospy.Subscriber("/clock", Clock, clock_callback)

    for x in range(len(sceneIds)):
        for y in range(len(speedsValues)):
            agent_collition_ids = []
            current_scene = x
            current_speed = y
            scene = "scene" + str(sceneIds[x]) + \
                "_" + str(speedsValues[y]) + "_0"
            agent_id = agentsIds[x]
            run_process(scene, agent_id)

    csv_filename = "output_model.csv"
    with open(csv_filename, mode='w', newline='') as csv_file:
        writer = csv.writer(csv_file)

        # Write header row
        writer.writerow(["Scenario", "Pedestrian Collisions", "Environment Collisions", "Timeout"])

        for row_index in range(len(success_results)):
            for element_index in range(len(success_results[row_index])):
                element = success_results[row_index][element_index]

                # Get the values
                pedestrian_collisions = element.pedestrian_collisions
                environment_collisions = element.environment_collisions
                timeout = element.timeout
                
                # Write the values to the CSV file
                writer.writerow(["Scene"+str(sceneIds[row_index])+"_"+str(speedsValues[element_index])+"_0", pedestrian_collisions, environment_collisions, timeout])


    rospy.spin()
