#include "ros/ros.h"
#include "include/lightsfm/sfm.hpp"
#include "rosgraph_msgs/Clock.h"
#include "geometry_msgs/Twist.h"
#include "include/rvo/RVO.h"
#include "geometry_msgs/Pose.h"
#include <tf2/LinearMath/Matrix3x3.h>
#include "gazebo_msgs/ModelStates.h"
#include "sensor_msgs/PointCloud2.h"
#include <cmath>
#include <iostream>
#include <fstream>
#include <algorithm>
#include <regex>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

// Global variables to store data
std::string method;
ros::Publisher cmd_vel_pub_;
bool goal_reached_ = false;
double goal_x, goal_y, linear_velocity, angular_velocity;
geometry_msgs::Pose current_robot_pose;
gazebo_msgs::ModelStates modelState;
sensor_msgs::PointCloud2 currentObstaclePointCloud;
double initial_robot_gazebo_position_x = 12.091963;
double initial_robot_gazebo_position_y = 5.868032;
double last_processed_time = 0.0;

struct DatasetLine
{
    double frameId, personId, positionX, positionY, positionZ, speedX, speedY, speedZ;

    DatasetLine(double col1, double col2, double col3, double col4, double col5, double col6, double col7, double col8)
        : frameId(col1), personId(col2), positionX(col3), positionY(col5), positionZ(col4), speedX(col6), speedY(col8), speedZ(col7) {}
};

std::vector<DatasetLine> datasetLineVector;
bool annotationProcessed = false;

struct AgentPositionStored
{
    std::string name;
    double x;
    double y;
    double time;

    AgentPositionStored(std::string nameIn, double xIn, double yIn, double timeIn)
        : name(nameIn), x(xIn), y(yIn), time(timeIn) {}
};
std::vector<AgentPositionStored> agentPositionStored;

struct ApaAgent
{
    int personId;
    double x;
    double y;
    double a;
    double speed;
    double time;

    ApaAgent(int personIdIn, double xIn, double yIn, double aIn, double timeIn, double speedIn)
        : personId(personIdIn), x(xIn), y(yIn), a(aIn), time(timeIn), speed(speedIn) {}
};

// Function declarations
void clockCallback(const rosgraph_msgs::Clock &msg);
void stopRobot();
void rotateRobot(double angle_error);
void moveForward(double distance_to_goal);
void getAnnotationData();
void rotateAndMoveRobot(double angle_error, double distance_to_goal);
double normalizeAngle(double angle);
void modelCallback(const gazebo_msgs::ModelStates msg);
void obstaclesPointCloudCallback(const sensor_msgs::PointCloud2 msg);
int getIndexByName(const std::vector<AgentPositionStored> vec, const std::string targetName);

// Motion Controllers
void socialForceMotionController(double currentTime);
void rvo2MotionController(double currentTime);
void sacadrl2MotionController(double currentTime);
void apa2MotionController(double currentTime);

// APA variables
double v_d = 0.3;
double omega_distance = 0.74;
double omega_heading = 0.16;
double omega_speed = 0.1;
double tau = 0.78;
double apa_d = 1.2;
double prediction_window = 4.0;
double sigma_apa = 1.0;

int main(int argc, char **argv)
{
    ros::init(argc, argv, "social_navigation_testbed_robot_controller");
    ros::NodeHandle n;
    ros::NodeHandle nh("~");
    if (!annotationProcessed)
    {
        getAnnotationData();
        annotationProcessed = !annotationProcessed;
    }

    // Read parameters from the parameter server
    nh.param<std::string>("method", method, "apa");
    nh.param<double>("goal_x", goal_x, 0.700344);
    nh.param<double>("goal_y", goal_y, 4.908462);
    nh.param<double>("linear_velocity", linear_velocity, 0.3);
    nh.param<double>("angular_velocity", angular_velocity, 0.5);

    // Subscribers
    ros::Subscriber subClock = n.subscribe("/clock", 1000, clockCallback);
    ros::Subscriber subModelStates = n.subscribe("/gazebo/model_states", 1000, modelCallback);
    ros::Subscriber subObstaclesPointCloud = n.subscribe("/pointcloud_traversable", 1000, obstaclesPointCloudCallback);

    // Publisher for MoveBase simple goal
    cmd_vel_pub_ = n.advertise<geometry_msgs::Twist>("/cmd_vel_mux/input/teleop", 10);

    // Enter the ROS event loop
    ros::spin();
}

// Callback for the clock topic
void clockCallback(const rosgraph_msgs::Clock &msg)
{
    if (!goal_reached_)
    {
        if (msg.clock.toSec() > 140 && method == "unaware" && msg.clock.toSec() > last_processed_time + 0.1)
        {
            double distance_to_goal = std::sqrt(std::pow(goal_x - current_robot_pose.position.x, 2) + std::pow(goal_y - current_robot_pose.position.y, 2));

            if (distance_to_goal < 0.1)
            {
                goal_reached_ = true;
                stopRobot();
            }
            else
            {

                // Get robot angle in radians
                tf2::Quaternion quaternion(current_robot_pose.orientation.x, current_robot_pose.orientation.y, current_robot_pose.orientation.z, current_robot_pose.orientation.w);
                tf2::Matrix3x3 matrix(quaternion);
                double roll_robot, pitch_robot, yaw_robot;
                matrix.getRPY(roll_robot, pitch_robot, yaw_robot);

                double target_angle = std::atan2(goal_y - current_robot_pose.position.y, goal_x - current_robot_pose.position.x);
                double angle_error = normalizeAngle(target_angle - yaw_robot);

                if (std::fabs(angle_error) > 0.5)
                {
                    rotateRobot(angle_error);
                }
                else if (std::fabs(angle_error) > 0.1)
                {
                    rotateAndMoveRobot(angle_error, distance_to_goal);
                }
                else
                {
                    moveForward(distance_to_goal);
                }
            }
        }
        else if (msg.clock.toSec() > 140 && method == "sf" && msg.clock.toSec() > last_processed_time + 0.1)
        {
            socialForceMotionController(msg.clock.toSec());
            last_processed_time = msg.clock.toSec();
        }
        else if (msg.clock.toSec() > 140 && method == "rvo" && msg.clock.toSec() > last_processed_time + 0.1)
        {
            rvo2MotionController(msg.clock.toSec());
            last_processed_time = msg.clock.toSec();
        }
        else if (msg.clock.toSec() > 140 && method == "sacadrl" && msg.clock.toSec() > last_processed_time + 0.1)
        {
            sacadrl2MotionController(msg.clock.toSec());
            last_processed_time = msg.clock.toSec();
        }
        else if (msg.clock.toSec() > 140 && method == "apa" && msg.clock.toSec() > last_processed_time + 0.1)
        {

            double distance_to_goal = std::sqrt(std::pow(goal_x - current_robot_pose.position.x, 2) + std::pow(goal_y - current_robot_pose.position.y, 2));

            if (distance_to_goal < 0.1)
            {
                goal_reached_ = true;
                stopRobot();
            }
            else
            {
                apa2MotionController(msg.clock.toSec());
                last_processed_time = msg.clock.toSec();
            }
        }
    }
}

void stopRobot()
{
    geometry_msgs::Twist cmd_vel;
    cmd_vel.linear.x = 0.0;
    cmd_vel.angular.z = 0.0;
    cmd_vel_pub_.publish(cmd_vel);
}

void rotateRobot(double angle_error)
{
    geometry_msgs::Twist cmd_vel;
    cmd_vel.linear.x = 0.0;
    cmd_vel.angular.z = angle_error;
    cmd_vel_pub_.publish(cmd_vel);
}

void moveForward(double distance_to_goal)
{
    geometry_msgs::Twist cmd_vel;
    cmd_vel.linear.x = linear_velocity;
    cmd_vel.angular.z = 0.0;
    cmd_vel_pub_.publish(cmd_vel);
}

void rotateAndMoveRobot(double angle_error, double distance_to_goal)
{
    geometry_msgs::Twist cmd_vel;
    cmd_vel.linear.x = linear_velocity;
    cmd_vel.angular.z = angle_error*3;
    cmd_vel_pub_.publish(cmd_vel);
}

double normalizeAngle(double angle)
{
    while (angle <= -M_PI)
    {
        angle += 2 * M_PI;
    }
    while (angle > M_PI)
    {
        angle -= 2 * M_PI;
    }
    return angle;
}

// Callback for the model states topic
void modelCallback(const gazebo_msgs::ModelStates msg)
{
    modelState = msg;
    for (int i = 0; i < msg.name.size(); i++)
    {
        if (msg.name[i] == "mobile_base")
        {
            current_robot_pose = msg.pose[i];
        }
    }
}

void socialForceMotionController(double currentTime)
{
    // Define agent vector
    std::vector<sfm::Agent> myAgents;

    // Extract agents position from model
    for (int i = 0; i < modelState.name.size(); i++)
    {
        std::regex pattern("actor(\\d+)");
        std::smatch matches;

        if (modelState.name[i].find("actor") != std::string::npos && std::regex_search(modelState.name[i], matches, pattern) && modelState.pose[i].position.z > 0)
        {
            // Extract position
            utils::Vector2d agentPosition = utils::Vector2d(modelState.pose[i].position.x, modelState.pose[i].position.y);

            // Extract orientation
            tf2::Quaternion quaternion(modelState.pose[i].orientation.x, modelState.pose[i].orientation.y, modelState.pose[i].orientation.z, modelState.pose[i].orientation.w);
            tf2::Matrix3x3 matrix(quaternion);
            double roll, pitch, yaw;
            matrix.getRPY(roll, pitch, yaw);
            utils::Angle agentYaw = utils::Angle::fromRadian(yaw);

            int objIndex = getIndexByName(agentPositionStored, modelState.name[i]);
            double agentSpeed = 0.0;
            if (objIndex >= 0)
            {
                double deltaDistance = (double)sqrt(pow(modelState.pose[i].position.x - agentPositionStored[objIndex].x, 2) + pow(modelState.pose[i].position.y - agentPositionStored[objIndex].y, 2));
                double deltaTime = currentTime - agentPositionStored[objIndex].time;
                agentSpeed = deltaDistance / deltaTime;
                agentPositionStored[objIndex].x = modelState.pose[i].position.x;
                agentPositionStored[objIndex].y = modelState.pose[i].position.y;
                agentPositionStored[objIndex].time = currentTime;
            }
            else
            {
                agentPositionStored.push_back(AgentPositionStored(modelState.name[i], modelState.pose[i].position.x, modelState.pose[i].position.y, currentTime));
            }

            // Create Agent OBJ
            sfm::Agent myAgent = sfm::Agent(agentPosition, agentYaw, agentSpeed, 0.0);
            myAgent.desiredVelocity = agentSpeed;
            myAgent.radius = 1.2;
            // myAgent.params.forceFactorObstacle = 5.0;
            // myAgent.obstacles1 = laserObstaclesTmp;

            // Set Agent Goal
            sfm::Goal goal;
            goal.radius = 0.3;
            goal.center.set(0.0, 0.0);
            myAgent.goals.push_back(goal);

            myAgents.push_back(myAgent);
        }
    }

    pcl::PointCloud<pcl::PointXYZI> currentObstaclePCL;
    pcl::fromROSMsg(currentObstaclePointCloud, currentObstaclePCL);

    for (int i = 0; i < myAgents.size(); i++)
    {
        for (int j = 0; j < currentObstaclePCL.size(); j++)
        {
            double distance2obstacle = (double)sqrt(pow(initial_robot_gazebo_position_x - currentObstaclePCL[j].x - myAgents[i].position.getX(), 2) + pow(initial_robot_gazebo_position_y - currentObstaclePCL[j].y - myAgents[i].position.getY(), 2));
            if (distance2obstacle > 0.45 && distance2obstacle < 3.0)
            {
                myAgents[i].obstacles1.push_back(utils::Vector2d(initial_robot_gazebo_position_x - currentObstaclePCL[j].x, initial_robot_gazebo_position_y - currentObstaclePCL[j].y));
            }
        }
    }

    // Create Robot Agent
    // Extract position
    utils::Vector2d sfmRobotPosition = utils::Vector2d(current_robot_pose.position.x, current_robot_pose.position.y);

    // Extract orientation
    tf2::Quaternion quaternion(current_robot_pose.orientation.x, current_robot_pose.orientation.y, current_robot_pose.orientation.z, current_robot_pose.orientation.w);
    tf2::Matrix3x3 matrix(quaternion);
    double roll, pitch, yaw;
    matrix.getRPY(roll, pitch, yaw);
    utils::Angle sfmRobotYaw = utils::Angle::fromRadian(yaw);

    sfm::Agent myRobotAgent = sfm::Agent(sfmRobotPosition, sfmRobotYaw, linear_velocity, 0.0);
    myRobotAgent.radius = 1.2;

    sfm::Goal robotGoal;
    robotGoal.center.set(goal_x, goal_y);
    myRobotAgent.goals.push_back(robotGoal);
    myAgents.push_back(myRobotAgent);

    myAgents = sfm::SFM.computeForces(myAgents);

    double robotForceAngle = atan2(myAgents.back().forces.globalForce.getY(), myAgents.back().forces.globalForce.getX());
    // double robotForceAngle = myAgents.back().forces.globalForce.angle().toRadian();
    double force = myAgents.back().forces.socialForce.angle().toRadian();

    double distance_to_goal = std::sqrt(std::pow(goal_x - current_robot_pose.position.x, 2) + std::pow(goal_y - current_robot_pose.position.y, 2));

    if (distance_to_goal < 0.1)
    {
        goal_reached_ = true;
        stopRobot();
    }
    else
    {

        double angle_error = normalizeAngle(robotForceAngle - yaw);

        if (std::fabs(angle_error) > 0.5)
        {
            rotateRobot(angle_error);
        }
        else if (std::fabs(angle_error) > 0.1)
        {
            rotateAndMoveRobot(angle_error, distance_to_goal);
        }
        else
        {
            moveForward(distance_to_goal);
        }
    }
}

void obstaclesPointCloudCallback(const sensor_msgs::PointCloud2 msg)
{
    currentObstaclePointCloud = msg;
}

int getIndexByName(const std::vector<AgentPositionStored> vec, const std::string targetName)
{
    for (size_t i = 0; i < vec.size(); ++i)
    {
        if (vec[i].name == targetName)
        {
            return static_cast<int>(i);
        }
    }
    return -1;
}

void rvo2MotionController(double currentTime)
{
    // Create a new simulator instance.
    RVO::RVOSimulator *sim = new RVO::RVOSimulator();

    // Specify default parameters for agents that are subsequently added.
    sim->setAgentDefaults(1.2f, 10, 3.0f, 5.0f, 0.1f, linear_velocity);

    // Set up the scenario.
    // Specify global time step of the simulation.
    sim->setTimeStep(0.1f);

    // Specify default parameters for agents that are subsequently added.
    // sim->setAgentDefaults(15.0f, 10, 10.0f, 5.0f, 2.0f, 2.0f); // TODO Check defaults

    // Add agents, specifying their start position.
    for (int i = 0; i < modelState.name.size(); i++)
    {
        std::regex pattern("actor(\\d+)");
        std::smatch matches;

        if (modelState.name[i].find("actor") != std::string::npos && std::regex_search(modelState.name[i], matches, pattern) && modelState.pose[i].position.z > 0)
        {
            sim->addAgent(RVO::Vector2(modelState.pose[i].position.x, modelState.pose[i].position.y));

            for (int j = datasetLineVector.size() - 1; j >= 0; j--)
            {
                if (datasetLineVector[j].personId == stoi(matches[1].str()))
                {
                    sim->setAgentPrefVelocity(sim->getNumAgents() - 1, normalize(RVO::Vector2(datasetLineVector[j].positionX, datasetLineVector[j].positionY) - sim->getAgentPosition(sim->getNumAgents() - 1)));
                    break;
                }
            }
        }
    }

    // Add (polygonal) obstacle(s), specifying vertices in counterclockwise order.// TODO Check
    pcl::PointCloud<pcl::PointXYZI> currentObstaclePCL;
    pcl::fromROSMsg(currentObstaclePointCloud, currentObstaclePCL);
    for (int j = 0; j < currentObstaclePCL.size(); j++)
    {
        bool isPerson = false;

        for (int i = 0; i < sim->getNumAgents(); i++)
        {
            double distance2obstacle = (double)sqrt(pow(initial_robot_gazebo_position_x - currentObstaclePCL[j].x - sim->getAgentPosition(i).x(), 2) + pow(initial_robot_gazebo_position_y - currentObstaclePCL[j].y - sim->getAgentPosition(i).y(), 2));
            if (distance2obstacle < 0.4)
            {
                isPerson = true;
                break;
            }
        }
        if (!isPerson)
        {

            std::vector<RVO::Vector2> vertices;
            vertices.push_back(RVO::Vector2(initial_robot_gazebo_position_x - currentObstaclePCL[j].x, initial_robot_gazebo_position_y - currentObstaclePCL[j].y));
            vertices.push_back(RVO::Vector2(initial_robot_gazebo_position_x - currentObstaclePCL[j].x + 0.05, initial_robot_gazebo_position_y - currentObstaclePCL[j].y));
            vertices.push_back(RVO::Vector2(initial_robot_gazebo_position_x - currentObstaclePCL[j].x + 0.05, initial_robot_gazebo_position_y - currentObstaclePCL[j].y + 0.05));
            vertices.push_back(RVO::Vector2(initial_robot_gazebo_position_x - currentObstaclePCL[j].x, initial_robot_gazebo_position_y - currentObstaclePCL[j].y + 0.05));
            sim->addObstacle(vertices);
        }
    }
    sim->processObstacles();

    // Add robot position and goal
    sim->addAgent(RVO::Vector2(current_robot_pose.position.x, current_robot_pose.position.y));
    sim->setAgentPrefVelocity(sim->getNumAgents() - 1, normalize(RVO::Vector2(goal_x, goal_y) - sim->getAgentPosition(sim->getNumAgents() - 1)));

    // Perform (and manipulate) the simulation.
    sim->doStep();

    // Extract Current Robot orientation
    tf2::Quaternion quaternion(current_robot_pose.orientation.x, current_robot_pose.orientation.y, current_robot_pose.orientation.z, current_robot_pose.orientation.w);
    tf2::Matrix3x3 matrix(quaternion);
    double roll, pitch, yaw;
    matrix.getRPY(roll, pitch, yaw);
    utils::Angle sfmRobotYaw = utils::Angle::fromRadian(yaw);

    // TODO GET NEW ROBOT POSITION AND MOVE
    double relativeAngle = atan2(sim->getAgentPosition(sim->getNumAgents() - 1).y() - current_robot_pose.position.y, sim->getAgentPosition(sim->getNumAgents() - 1).x() - current_robot_pose.position.x);

    std::cout << sim->getAgentPosition(sim->getNumAgents() - 1) << std::endl;

    double distance_to_goal = std::sqrt(std::pow(goal_x - current_robot_pose.position.x, 2) + std::pow(goal_y - current_robot_pose.position.y, 2));

    std::cout << "distance_to_goal = " << distance_to_goal << std::endl;

    if (distance_to_goal < 0.1)
    {
        goal_reached_ = true;
        stopRobot();
    }
    else
    {

        double angle_error = normalizeAngle(relativeAngle - yaw);

        if (std::fabs(angle_error) > 0.5)
        {
            rotateRobot(angle_error);
        }
        else if (std::fabs(angle_error) > 0.1)
        {
            rotateAndMoveRobot(angle_error, distance_to_goal);
        }
        else
        {
            moveForward(distance_to_goal);
        }
    }

    delete sim;
}

void getAnnotationData()
{
    std::string inputfile = "/home/carlos/catkin_ws/src/social_navigation_testbed/dataset/seq_eth/obsmat.txt";

    std::ifstream infile(inputfile);
    if (infile.fail())
    {
        std::cout << "Failed to open file: " << inputfile << std::endl;
        return;
    }

    // Store all dataset lines in datasetLineVector and store the max speed in the scene
    double col1, col2, col3, col4, col5, col6, col7, col8;
    while (infile >> col1 >> col2 >> col3 >> col4 >> col5 >> col6 >> col7 >> col8)
    {
        // Get dataset line
        DatasetLine datasetLine(col1, col2, col3, col4, col5, col6, col7, col8);
        // Store dataset line in datasetLineVector
        datasetLineVector.push_back(datasetLine);
    }
    infile.close();
}

void sacadrl2MotionController(double currentTime)
{
}

void apa2MotionController(double currentTime)
{

    std::vector<ApaAgent> apaAgentStored;

    // Path Predictor
    for (int i = 0; i < modelState.name.size(); i++)
    {
        std::regex pattern("actor(\\d+)");
        std::smatch matches;

        if (modelState.name[i].find("actor") != std::string::npos && std::regex_search(modelState.name[i], matches, pattern) && modelState.pose[i].position.z > 0)
        {
            bool isInArray = false;
            for (int j = 0; j < apaAgentStored.size(); j++)
            {
                if (stoi(matches[1].str()) == apaAgentStored[j].personId)
                {
                    // Update Agent
                    apaAgentStored[j].speed = sqrt(pow(modelState.pose[i].position.y - apaAgentStored[j].y, 2) + pow(modelState.pose[i].position.x - apaAgentStored[j].x, 2)) / (currentTime - apaAgentStored[j].time);
                    apaAgentStored[j].a = atan2(modelState.pose[i].position.y - apaAgentStored[j].y, modelState.pose[i].position.x - apaAgentStored[j].x);
                    apaAgentStored[j].x = modelState.pose[i].position.x;
                    apaAgentStored[j].y = modelState.pose[i].position.y;
                    apaAgentStored[j].time = currentTime;
                    isInArray = true;
                    break;
                }
            }
            if (!isInArray)
            {
                // Add new Agent
                // Extract Current Robot orientation
                tf2::Quaternion quaternion(modelState.pose[i].orientation.x, modelState.pose[i].orientation.y, modelState.pose[i].orientation.z, modelState.pose[i].orientation.w);
                tf2::Matrix3x3 matrix(quaternion);
                double roll, pitch, yaw;
                matrix.getRPY(roll, pitch, yaw);

                apaAgentStored.push_back(ApaAgent(stoi(matches[1].str()), modelState.pose[i].position.x, modelState.pose[i].position.y, yaw, currentTime, linear_velocity));
            }
        }
    }
    // Group detection

    // Convert Agents in single groups
    std::cout << "Agents = " << apaAgentStored.size();
    std::vector<std::vector<ApaAgent>> apaAgentGroupsStored;
    apaAgentGroupsStored.resize(apaAgentStored.size());

    for (int j = 0; j < apaAgentStored.size(); j++)
    {
        apaAgentGroupsStored[j].push_back(apaAgentStored[j]);
    }

    // Create groups
    for (int j = 0; j < apaAgentGroupsStored.size() - 1; j++)
    {
        bool shouldBreak = false;

        for (int k = 0; k < apaAgentGroupsStored[j].size(); k++)
        {
            for (int l = 0; l < apaAgentGroupsStored.size(); l++)
            {
                if (j != l)
                {
                    double delta_distance = sqrt(pow(apaAgentGroupsStored[j][k].x - apaAgentGroupsStored[l][0].x, 2) + pow(apaAgentGroupsStored[j][k].y - apaAgentGroupsStored[l][0].y, 2));
                    double sigma_d = exp(-v_d * delta_distance);
                    double sigma_distance = omega_distance * sigma_d;

                    double sigma_h = 1 - (fabs(fabs(apaAgentGroupsStored[j][k].a) - fabs(apaAgentGroupsStored[l][0].a)) / M_PI);
                    double sigma_heading = omega_heading * sigma_h;

                    double sigma_s = 1 - (fabs(apaAgentGroupsStored[j][k].speed - apaAgentGroupsStored[l][0].speed) / std::max(fabs(apaAgentGroupsStored[j][k].speed), fabs(apaAgentGroupsStored[l][0].speed)));
                    double sigma_speed = omega_speed * sigma_s;

                    double sigma_total = sigma_distance + sigma_heading + sigma_speed;

                    if (sigma_total > tau)
                    {
                        apaAgentGroupsStored[j].push_back(apaAgentGroupsStored[l][0]);
                        apaAgentGroupsStored.erase(apaAgentGroupsStored.begin() + l);
                        j = -1;
                        shouldBreak = true;
                        break;
                    }
                    // Compare agent in [j][k] with [j+1][0]
                }
            }
            if (shouldBreak)
            {
                shouldBreak = false;
                break; // Breaks out of the middle loop
            }
        }
    }

    // Generaate APA and check for current and future intrussions
    // Create groups

    // TODO add Prediction window
    bool collision_detected = false;
    std::vector<geometry_msgs::Point> way_point_vector;
    geometry_msgs::Point way_point_selected;
    for (double p_time = 0.0; p_time < prediction_window; p_time += 0.1)
    {
        for (int j = 0; j < apaAgentGroupsStored.size() - 1; j++)
        {
            for (int k = 0; k < apaAgentGroupsStored[j].size(); k++)
            {
                // TODO GET AGENT AND ROBOT POSITION IN CURREN PREDICTED TIME
                double agent_delta_distance = (double)apaAgentGroupsStored[j][k].speed * p_time;
                double new_agent_x = (double)apaAgentGroupsStored[j][k].x + agent_delta_distance * cos(apaAgentGroupsStored[j][k].a);
                double new_agent_y = (double)apaAgentGroupsStored[j][k].y + agent_delta_distance * sin(apaAgentGroupsStored[j][k].a);

                // Extract Current Robot orientation
                tf2::Quaternion quaternion(current_robot_pose.orientation.x, current_robot_pose.orientation.y, current_robot_pose.orientation.z, current_robot_pose.orientation.w);
                tf2::Matrix3x3 matrix(quaternion);
                double robot_roll, robot_pitch, robot_yaw;
                matrix.getRPY(robot_roll, robot_pitch, robot_yaw);

                double angle_to_goal_tmp = atan2(goal_y - current_robot_pose.position.y, goal_x - current_robot_pose.position.x);

                double robot_delta_distance = (double)linear_velocity * p_time;
                double new_robot_x = (double)current_robot_pose.position.x + robot_delta_distance * cos(angle_to_goal_tmp);
                double new_robot_y = (double)current_robot_pose.position.y + robot_delta_distance * sin(angle_to_goal_tmp);

                double angle_to_robot = atan2(new_robot_y - new_agent_y, new_robot_x - new_agent_x);

                double distance_to_robot = sqrt(pow(new_robot_y - new_agent_y, 2) + pow(new_robot_x - new_agent_x, 2));

                double delta_angle_orientation = fabs(normalizeAngle(angle_to_robot - apaAgentGroupsStored[j][k].a));
                if (delta_angle_orientation < M_PI / 2)
                {

                    double cp1 = ((apaAgentGroupsStored[j][k].speed / (sigma_apa * sqrt(2 * M_PI))) * exp(-0.5 * pow(delta_angle_orientation / sigma_apa, 2))) + apa_d;
                    std::cout << "cp1 = " << cp1 << std::endl;
                    std::cout << "distance_to_robot = " << distance_to_robot << std::endl;
                    if (cp1 > distance_to_robot)
                    {
                        std::cout << "frente" << std::endl;
                        // TODO out area evasion
                        double tmp_angle_to_cp = 0.0;
                        for (double angle_iterator = delta_angle_orientation; angle_iterator < M_PI / 2; M_PI / 100)
                        {
                            double cp_iterator = ((apaAgentGroupsStored[j][k].speed / (sigma_apa * sqrt(2 * M_PI))) * exp(-0.5 * pow(angle_iterator / sigma_apa, 2))) + apa_d;
                            double relative_robot_position_x = distance_to_robot * cos(delta_angle_orientation);
                            double relative_robot_position_y = distance_to_robot * sin(delta_angle_orientation);
                            double relative_cp_position_x = cp_iterator * cos(angle_iterator);
                            double relative_cp_position_y = cp_iterator * sin(angle_iterator);
                            double angle_to_agent = atan2(-relative_robot_position_y, -relative_robot_position_x);
                            double angle_to_cp = atan2(relative_cp_position_y - relative_robot_position_y, relative_cp_position_x - relative_robot_position_x);
                            double delta_cp_robot_angle = fabs(normalizeAngle(angle_to_agent - angle_to_cp));
                            if (tmp_angle_to_cp < delta_cp_robot_angle)
                            {
                                tmp_angle_to_cp = delta_cp_robot_angle;
                            }
                            else
                            {
                                geometry_msgs::Point tmp_point;
                                tmp_point.x = new_agent_x + cp_iterator * cos(angle_to_cp);
                                tmp_point.y = new_agent_y + cp_iterator * sin(angle_to_cp);
                                tmp_point.z = (double)apaAgentGroupsStored[j][k].speed /linear_velocity;
                                way_point_vector.push_back(tmp_point);
                                break;
                            }
                        }

                        tmp_angle_to_cp = 0.0;
                        for (double angle_iterator = delta_angle_orientation; angle_iterator > -M_PI / 2; -M_PI / 100)
                        {
                            double cp_iterator = ((apaAgentGroupsStored[j][k].speed / (sigma_apa * sqrt(2 * M_PI))) * exp(-0.5 * pow(angle_iterator / sigma_apa, 2))) + apa_d;
                            double relative_robot_position_x = distance_to_robot * cos(delta_angle_orientation);
                            double relative_robot_position_y = distance_to_robot * sin(delta_angle_orientation);
                            double relative_cp_position_x = cp_iterator * cos(angle_iterator);
                            double relative_cp_position_y = cp_iterator * sin(angle_iterator);
                            double angle_to_agent = atan2(-relative_robot_position_y, -relative_robot_position_x);
                            double angle_to_cp = atan2(relative_cp_position_y - relative_robot_position_y, relative_cp_position_x - relative_robot_position_x);
                            double delta_cp_robot_angle = fabs(normalizeAngle(angle_to_agent - angle_to_cp));
                            if (tmp_angle_to_cp < delta_cp_robot_angle)
                            {
                                tmp_angle_to_cp = delta_cp_robot_angle;
                            }
                            else
                            {
                                geometry_msgs::Point tmp_point;
                                tmp_point.x = new_agent_x + cp_iterator * cos(angle_to_cp);
                                tmp_point.y = new_agent_y + cp_iterator * sin(angle_to_cp);
                                tmp_point.z = (double)apaAgentGroupsStored[j][k].speed /linear_velocity;
                                way_point_vector.push_back(tmp_point);
                                break;
                            }
                        }

                        collision_detected = true;
                    }
                    else
                    {
                        // TODO in area evasion
                        collision_detected = false;
                    }
                }
                else
                {

                    double cp2 = ((apaAgentGroupsStored[j][k].speed / (sigma_apa * sqrt(2 * M_PI))) * exp(-0.5 * pow(((2 * M_PI) - delta_angle_orientation) / sigma_apa, 2))) + apa_d;

                    std::cout << "cp2 = " << cp2 << std::endl;
                    std::cout << "distance_to_robot = " << distance_to_robot << std::endl;
                    if (cp2 > distance_to_robot)
                    {
                        std::cout << "dettras" << std::endl;
                        double tmp_angle_to_cp = 0.0;
                        for (double angle_iterator = delta_angle_orientation; angle_iterator < M_PI / 2; M_PI / 100)
                        {
                            double cp_iterator = ((apaAgentGroupsStored[j][k].speed / (sigma_apa * sqrt(2 * M_PI))) * exp(-0.5 * pow(((2 * M_PI) - angle_iterator) / sigma_apa, 2))) + apa_d;

                            double relative_robot_position_x = distance_to_robot * cos(delta_angle_orientation);
                            double relative_robot_position_y = distance_to_robot * sin(delta_angle_orientation);
                            double relative_cp_position_x = cp_iterator * cos(angle_iterator);
                            double relative_cp_position_y = cp_iterator * sin(angle_iterator);
                            double angle_to_agent = atan2(-relative_robot_position_y, -relative_robot_position_x);
                            double angle_to_cp = atan2(relative_cp_position_y - relative_robot_position_y, relative_cp_position_x - relative_robot_position_x);
                            double delta_cp_robot_angle = fabs(normalizeAngle(angle_to_agent - angle_to_cp));
                            if (tmp_angle_to_cp < delta_cp_robot_angle)
                            {
                                tmp_angle_to_cp = delta_cp_robot_angle;
                            }
                            else
                            {
                                geometry_msgs::Point tmp_point;
                                tmp_point.x = new_agent_x + cp_iterator * cos(angle_to_cp);
                                tmp_point.y = new_agent_y + cp_iterator * sin(angle_to_cp);
                                tmp_point.z = (double)apaAgentGroupsStored[j][k].speed /linear_velocity;
                                way_point_vector.push_back(tmp_point);
                                break;
                            }
                        }

                        tmp_angle_to_cp = 0.0;
                        for (double angle_iterator = delta_angle_orientation; angle_iterator > -M_PI / 2; -M_PI / 100)
                        {
                            double cp_iterator = ((apaAgentGroupsStored[j][k].speed / (sigma_apa * sqrt(2 * M_PI))) * exp(-0.5 * pow(((2 * M_PI) - angle_iterator) / sigma_apa, 2))) + apa_d;
                            double relative_robot_position_x = distance_to_robot * cos(delta_angle_orientation);
                            double relative_robot_position_y = distance_to_robot * sin(delta_angle_orientation);
                            double relative_cp_position_x = cp_iterator * cos(angle_iterator);
                            double relative_cp_position_y = cp_iterator * sin(angle_iterator);
                            double angle_to_agent = atan2(-relative_robot_position_y, -relative_robot_position_x);
                            double angle_to_cp = atan2(relative_cp_position_y - relative_robot_position_y, relative_cp_position_x - relative_robot_position_x);
                            double delta_cp_robot_angle = fabs(normalizeAngle(angle_to_agent - angle_to_cp));
                            if (tmp_angle_to_cp < delta_cp_robot_angle)
                            {
                                tmp_angle_to_cp = delta_cp_robot_angle;
                            }
                            else
                            {
                                geometry_msgs::Point tmp_point;
                                tmp_point.x = new_agent_x + cp_iterator * cos(angle_to_cp);
                                tmp_point.y = new_agent_y + cp_iterator * sin(angle_to_cp);
                                tmp_point.z = (double)apaAgentGroupsStored[j][k].speed /linear_velocity;
                                way_point_vector.push_back(tmp_point);
                                break;
                            }
                        }
                        collision_detected = true;
                    }
                    else
                    {
                        // TODO in area evasion
                        collision_detected = false;
                    }
                }
            }
        }

        if (way_point_vector.size() > 0)
        {
            break;
        }
    }

    // Get robot angle in radians
    tf2::Quaternion quaternion(current_robot_pose.orientation.x, current_robot_pose.orientation.y, current_robot_pose.orientation.z, current_robot_pose.orientation.w);
    tf2::Matrix3x3 matrix(quaternion);
    double roll_robot, pitch_robot, yaw_robot;
    matrix.getRPY(roll_robot, pitch_robot, yaw_robot);

    for (int way_points_index = 0; way_points_index < way_point_vector.size(); way_points_index++)
    {
        double target_angle = std::atan2(way_point_vector[way_points_index].y- current_robot_pose.position.y, way_point_vector[way_points_index].x - current_robot_pose.position.x);
        double delta_theta = fabs(normalizeAngle(target_angle-yaw_robot));
        double sigma_theta = 1 -(delta_theta/M_PI);
        double omega_theta = 0;
        if(way_point_vector[way_points_index].z<1){
            omega_theta = 1 - (1/way_point_vector[way_points_index].z);
        }

        


    }

    if (!collision_detected)
    {
        std::cout << "Sending goal" << std::endl;

        double target_angle = std::atan2(goal_y - current_robot_pose.position.y, goal_x - current_robot_pose.position.x);
        double angle_error = normalizeAngle(target_angle - yaw_robot);

        double distance_to_goal = std::sqrt(std::pow(goal_x - current_robot_pose.position.x, 2) + std::pow(goal_y - current_robot_pose.position.y, 2));

        if (std::fabs(angle_error) > 0.5)
        {
            rotateRobot(angle_error);
        }
        else if (std::fabs(angle_error) > 0.1)
        {
            rotateAndMoveRobot(angle_error, distance_to_goal);
        }
        else
        {
            moveForward(distance_to_goal);
        }
    }
    else
    {
        std::cout << "Sending waypoint" << std::endl;
        // Get robot angle in radians
        tf2::Quaternion quaternion(current_robot_pose.orientation.x, current_robot_pose.orientation.y, current_robot_pose.orientation.z, current_robot_pose.orientation.w);
        tf2::Matrix3x3 matrix(quaternion);
        double roll_robot, pitch_robot, yaw_robot;
        matrix.getRPY(roll_robot, pitch_robot, yaw_robot);

        double target_angle = std::atan2(way_point_selected.y - current_robot_pose.position.y, way_point_selected.x - current_robot_pose.position.x);
        double angle_error = normalizeAngle(target_angle - yaw_robot);

        double distance_to_goal = std::sqrt(std::pow(way_point_selected.x - current_robot_pose.position.x, 2) + std::pow(way_point_selected.y - current_robot_pose.position.y, 2));

        if (std::fabs(angle_error) > 0.5)
        {
            rotateRobot(angle_error);
        }
        else if (std::fabs(angle_error) > 0.1)
        {
            rotateAndMoveRobot(angle_error, distance_to_goal);
        }
        else
        {
            moveForward(distance_to_goal);
        }
    }
}
