import os
import signal
import subprocess
import rospy
import time
import math
import csv
from gazebo_msgs.msg import ModelStates
from rosgraph_msgs.msg import Clock
from geometry_msgs.msg import PoseWithCovarianceStamped
from geometry_msgs.msg import Pose
from kobuki_msgs.msg import BumperEvent

# Array Navigation Methods for simulation
# sceneIds = ["unaware", "sf", "rvo" , "sacadrl"]
navigationMethods = ["unaware", "sf", "rvo" , "sacadrl"]

# Array Scene IDs for simulation
# sceneIds = [0,1,2,3,4,5,6,7,8,9]
sceneIds = [0,1,2,3,4,5,6,7,8,9]

# Array with Speeds to be evaluated
speedsValuesAll = [0.3, 0.4, 0.5, 0.7, 1.0, 1.5]
speedsValues = [0.3, 0.4, 0.5, 0.7, 1.0, 1.5]

class SuccessRate():
    pedestrian_collisions = 0
    environment_collisions = 0
    minimum_distance_exceeded = 0
    timeout = 0

class AgentPose():
    def __init__(self, x, y, a):
        self.x = x
        self.y = y
        self.a = a


model_names = []
model_poses = []
simulation_process = None 
currentRobotPose = None
initial_robot_gazebo_position_x = 12.091963
initial_robot_gazebo_position_y = 5.868032
distance_collition = 0.45 # Distance to another person to count as collision
current_min_distance_person = 0.45

success_results = [[[SuccessRate() for i in range(len(navigationMethods))] for j in range(len(speedsValues))] for k in range(len(sceneIds))]
current_scene = 0 
current_speed = 0
current_navigation_method = 0
current_clock = None



def distance_between_points(x1, y1, x2, y2):
    return math.sqrt((x2 - x1)**2 + (y2 - y1)**2)

def amcl_pose_callback(msg):
    global currentRobotPose
    # currentRobotPose = msg

def model_states_callback(msg):
    global model_names
    global model_poses
    model_names = msg.name
    model_poses = msg.pose
    global currentRobotPose
    for xy in range(len(msg.name)):
        if msg.name[xy] == "mobile_base":
            currentRobotPose = msg.pose[xy]

def clock_callback(msg):
    global simulation_process, currentRobotPose, current_clock
    current_clock = msg.clock.to_sec()
     # Check for collisions
    if msg.clock.to_sec() > 140 and simulation_process:
        for name, pose in zip(model_names, model_poses):
            if name != "mobile_base" and 'actor' in name  and pose.position.z is not None and pose.position.z > 0.5:
                current_distance = distance_between_points(pose.position.x,pose.position.y, currentRobotPose.position.x, currentRobotPose.position.y)
                if current_distance < distance_collition and not (name in agent_collition_ids):
                    agent_collition_ids.append(name)
                    success_results[current_scene][current_speed][current_navigation_method].pedestrian_collisions = success_results[current_scene][current_speed][current_navigation_method].pedestrian_collisions+1
                
                print (current_min_distance_person)

                if current_distance < current_min_distance_person and not (name in agent_min_distance_ids):
                    agent_min_distance_ids.append(name)
                    success_results[current_scene][current_speed][current_navigation_method].minimum_distance_exceeded = success_results[current_scene][current_speed][current_navigation_method].minimum_distance_exceeded+1

    # Check if the clock is more than 140 seconds and the z-dimension of the model is less than 1 meter
    if msg.clock.to_sec() > 320:
        if simulation_process:
            success_results[current_scene][current_speed][current_navigation_method].timeout = 1
            time.sleep(15)
            os.killpg(os.getpgid(simulation_process.pid), signal.SIGTERM)
            simulation_process = None
    elif msg.clock.to_sec() > 143:
        deltaDistance = distance_between_points(
            goalPoses[sceneIds[x]].x, goalPoses[sceneIds[x]].y, currentRobotPose.position.x, currentRobotPose.position.y)
        if simulation_process and deltaDistance < 0.2:
            time.sleep(15)
            os.killpg(os.getpgid(simulation_process.pid), signal.SIGTERM)
            simulation_process = None

def bumper_callback(data):
    global current_clock
    if data.state == BumperEvent.PRESSED and current_clock > 140:
        success_results[current_scene][current_speed][current_navigation_method].environment_collisions = success_results[x][y][current_navigation_method].environment_collisions+1

def run_process(scene, gazebo_origin_x, gazebo_origin_y, robot_yaw, navMethod, gazebo_goal_x, gazebo_goal_y, linear_velocity, distance2person):
    global simulation_process, initial_robot_gazebo_position_x, initial_robot_gazebo_position_y
    
    amcl_origin_x = gazebo_origin_x-initial_robot_gazebo_position_x
    amcl_origin_y = gazebo_origin_y-initial_robot_gazebo_position_y

    roslaunch_cmd = f"roslaunch social_navigation_testbed turtlebot2_navigation_lidar.launch world:={scene} gazebo_origin_x:={gazebo_origin_x} gazebo_origin_y:={gazebo_origin_y} robot_yaw:={robot_yaw} amcl_origin_x:={amcl_origin_x} amcl_origin_y:={amcl_origin_y} method:={navMethod} gazebo_goal_x:={gazebo_goal_x} gazebo_goal_y:={gazebo_goal_y} linear_velocity:={linear_velocity} personal_area_radius:={personal_area_radius} distance_collition:={distance_collition} distance_collition:={distance_collition} min_distance_person:={distance2person}"
    simulation_process = subprocess.Popen(
        roslaunch_cmd, stdout=subprocess.PIPE, shell=True, preexec_fn=os.setsid)
    out, err = simulation_process.communicate()  # Capture the process output


if __name__ == "__main__":
    rospy.init_node('socialNavSimulator', anonymous=True)
    rospy.Subscriber("/gazebo/model_states",
                     ModelStates, model_states_callback)
    rospy.Subscriber("/clock", Clock, clock_callback)
    rospy.Subscriber("/amcl_pose", PoseWithCovarianceStamped,
                     amcl_pose_callback)
    rospy.Subscriber('/mobile_base/events/bumper', BumperEvent, bumper_callback)


    personal_area_radius = 1.2

    # Set origin point for the different scenes
    originScene0 = AgentPose(12.091963, 5.868032, 2.945059)
    originScene1 = AgentPose(4.639745, 4.544470, 0.359245)
    originScene2 = AgentPose(12.375877, 4.341165, 3.047600)
    originScene3 = AgentPose(9.788466, 6.584937, 3.082292)
    originScene4 = AgentPose(13.099268, 6.045603, 2.882637)
    originScene5 = AgentPose(-0.992511, 5.333773, 0.042148)
    originScene6 = AgentPose(12.700387, 7.289635, 3.069573)
    originScene7 = AgentPose(13.471352, 6.328443, 2.806855)
    originScene8 = AgentPose(12.927797, 6.398554, 2.608999)
    originScene9 = AgentPose(12.920242, 6.169975, -2.713411)
    originPoses = [originScene0, originScene1, originScene2, originScene3, originScene4,
                   originScene5, originScene6, originScene7, originScene8, originScene9]

    # Set goal point for the different scenes
    goalScene0 = AgentPose(0.700344, 4.908462, -2.973111)
    goalScene1 = AgentPose(11.702465, 5.575852, 0.017477)
    goalScene2 = AgentPose(-3.042949, 2.632171, 3.138053)
    goalScene3 = AgentPose(-2.223126, 3.676843, -2.436460)
    goalScene4 = AgentPose(-1.288819, 6.873778, -3.082558)
    goalScene5 = AgentPose(12.794776, 5.717078, 0.098503)
    goalScene6 = AgentPose(-3.563498, -0.545546, -2.472420)
    goalScene7 = AgentPose(-0.949896, 7.084822, -2.941277)
    goalScene8 = AgentPose(-1.477435, 4.697356, -3.015326)
    goalScene9 = AgentPose(-1.527224, 8.453801, -3.120914)
    goalPoses = [goalScene0, goalScene1, goalScene2, goalScene3, goalScene4,
                 goalScene5, goalScene6, goalScene7, goalScene8, goalScene9]

    

    avgRobotSpeed = [
        [0.252418, 0.335901, 0.420353, 0.589711, 0.841496, 1.24919],
        [0.168348, 0.224625, 0.280525, 0.392572, 0.560025, 0.841078],
        [0.229876, 0.305746, 0.383433, 0.536322, 0.765904, 1.12792],
        [0.250255, 0.33404, 0.417882, 0.585169, 0.834604, 1.24107],
        [0.2062, 0.275238, 0.343853, 0.480257, 0.684365, 1.01884],
        [0.166737, 0.222228, 0.277707, 0.389534, 0.555412, 0.831549],
        [0.257034, 0.342406, 0.427361, 0.59711, 0.849698, 1.24569],
        [0.213046, 0.283892, 0.354849, 0.49699, 0.710696, 1.05498],
        [0.137487, 0.182988, 0.22863, 0.320372, 0.457903, 0.685867],
        [0.120487, 0.160485, 0.200379, 0.280829, 0.400815, 0.601463]
    ]



    min_distance_person = [
        [1.01043, 1.01159, 1.01326, 1.02296, 1.06341, 1.05536],
        [0.894782, 0.893153, 0.89169, 0.889673, 0.887245, 0.884284],
        [1.6251, 1.6264, 1.62836, 1.63414, 1.65993, 1.65341],
        [0.907469, 0.909528, 0.91174, 0.914935, 0.917378, 0.921891],
        [0.82882, 0.828745, 0.828729, 0.828482, 0.828193, 0.828417],
        [0.870108, 0.873239, 0.875437, 0.875946, 0.877578, 0.879687],
        [0.742159, 0.739274, 0.736484, 0.731293, 0.727378, 0.735278],
        [1.03912, 1.03995, 1.04126, 1.04659, 1.0679, 1.09104],
        [0.639062, 0.648555, 0.671361, 0.679895, 0.686208, 0.672892],
        [0.313038, 0.31278, 0.312635, 0.31246, 0.312187, 0.311991]
    ]


    for x in range(len(sceneIds)):
        for y in range(len(speedsValues)):
            for z in range(len(navigationMethods)):
                agent_collition_ids = []
                agent_min_distance_ids = []
                current_scene = x
                current_speed = y
                current_navigation_method = z
                current_min_distance_person = min_distance_person[sceneIds[x]][speedsValuesAll.index(speedsValues[y])]
                scene = "scene" + \
                    str(sceneIds[x]) + "_" + str(speedsValues[y]) + "_1"
                run_process(scene, originPoses[sceneIds[x]].x, originPoses[sceneIds[x]].y, originPoses[sceneIds[x]].a,
                            navigationMethods[z], goalPoses[sceneIds[x]].x, goalPoses[sceneIds[x]].y, avgRobotSpeed[sceneIds[x]][speedsValuesAll.index(speedsValues[y])], min_distance_person[sceneIds[x]][speedsValuesAll.index(speedsValues[y])])

    csv_filename = "output_experiment.csv"
    with open(csv_filename, mode='w', newline='') as csv_file:
        writer = csv.writer(csv_file)

        # Write header row
        writer.writerow(["Scenario", "Pedestrian Collisions", "Environment Collisions", "Timeout", "Minimum Distance Exceeded"])

        for row_index in range(len(success_results)):
            for element_index in range(len(success_results[row_index])):
                for element_index2 in range(len(success_results[row_index][element_index])):

                    element = success_results[row_index][element_index][element_index2]

                    # Get the values
                    pedestrian_collisions = element.pedestrian_collisions
                    environment_collisions = element.environment_collisions
                    timeout = element.timeout
                    minimum_distance_exceeded = element.minimum_distance_exceeded

                    # Write the values to the CSV file
                    writer.writerow(["Scene"+str(sceneIds[row_index])+"_"+str(speedsValues[element_index])+"_1_"+navigationMethods[element_index2], pedestrian_collisions, environment_collisions, timeout, minimum_distance_exceeded])

    rospy.spin()
