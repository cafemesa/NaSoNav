#include "ros/ros.h"
#include "gazebo_msgs/ModelStates.h"
#include "rosgraph_msgs/Clock.h"
#include <iostream>
#include <fstream>
#include <vector>
#include <algorithm>
#include <regex>

// Output file streams
std::ofstream distanceFileName, timeGoalFileName, timeInPersonalAreaFileName, robotPositionsFileName;

// Global variables
rosgraph_msgs::Clock currentClock, lastClock;
std::vector<double> timeVector, speedVector;
bool goalReached = false;
double startAgentSimTime = 140.0;
geometry_msgs::Pose initRobotPosition, lastRobotPosition;
bool initPoseStored = false;
double pathLenght = 0.0;
double minDistance2Person = 10000;
double personalAreaRadius = 1.2;
double distance_collition = 0.45;
double min_distance_person = 0.45;
std::vector<std::string> pedestrian_collitions_id;
std::vector<std::string> pedestrian_min_distance_id;
int pedestrian_collistions = 0;
int pedestrian_min_distance = 0;
double goalX = 0.0;
double goalY = 0.0;

struct TimeInPersonalZone
{
    double initTime;
    double endTime;
    int agentId;

    TimeInPersonalZone(double initTimeIn, double endTimeIn, int agentIdIn)
        : initTime(initTimeIn), endTime(endTimeIn), agentId(agentIdIn) {}

    double elapsedTime()
    {
        return endTime - initTime;
    }
};

std::vector<TimeInPersonalZone> timeInPersonalZoneVector;

// Callback functions
void modelCallback(const gazebo_msgs::ModelStates msg);
void clockCallback(const rosgraph_msgs::Clock msg);
double calculateAverage(const std::vector<double> &vec);

int main(int argc, char **argv)
{
    ros::init(argc, argv, "store_test_annotations");
    ros::NodeHandle n;
    ros::NodeHandle nh("~");

    // File names
    std::string distance2personsFile, time2goalFile, timeInPersonalAreaFile, robotPositionsFile;
    nh.param<std::string>("distance2personsFileName", distance2personsFile, "/home/carlos/catkin_ws/src/social_navigation_testbed/annotations/tests/1.csv");
    nh.param<std::string>("time2goalFilename", time2goalFile, "/home/carlos/catkin_ws/src/social_navigation_testbed/annotations/tests/2.csv");
    nh.param<std::string>("timeInPersonAreaFileName", timeInPersonalAreaFile, "/home/carlos/catkin_ws/src/social_navigation_testbed/annotations/tests/3.csv");
    nh.param<std::string>("robotPositionsFileName", robotPositionsFile, "/home/carlos/catkin_ws/src/social_navigation_testbed/annotations/tests/4.csv");

    // Variables
    nh.param<double>("personalAreaRadius", personalAreaRadius, 1.2);
    nh.param<double>("distance_collition", distance_collition, 0.45);
    nh.param<double>("minDistance2Person", minDistance2Person, 0.45);
    nh.param<double>("goal_x", goalX, 0.700344);
    nh.param<double>("goal_y", goalY, 4.908462);

    // Open files
    distanceFileName.open(distance2personsFile);
    timeGoalFileName.open(time2goalFile);
    timeInPersonalAreaFileName.open(timeInPersonalAreaFile);
    robotPositionsFileName.open(robotPositionsFile);

    // Write header to distance file
    distanceFileName << "Time (s),Agent ID,Distance\n";
    robotPositionsFileName << "Time (s),x,y\n";

    // Subscribers
    ros::Subscriber subClock = n.subscribe("/clock", 1000, clockCallback);
    ros::Subscriber subModelStates = n.subscribe("/gazebo/model_states", 1000, modelCallback);

    // Enter the ROS event loop
    ros::spin();

    // Close files
    distanceFileName.close();
    timeGoalFileName.close();
    timeInPersonalAreaFileName.close();

    return 0;
}

// Callback for the model states topic
void modelCallback(const gazebo_msgs::ModelStates msg)
{
    rosgraph_msgs::Clock currentClockLocal = currentClock;

    if (std::find(timeVector.begin(), timeVector.end(), currentClockLocal.clock.toSec()) != timeVector.end())
    {
        return;
    }

    geometry_msgs::Pose robotPosition;

    if (std::find(timeVector.begin(), timeVector.end(), currentClock.clock.toSec()) == timeVector.end())
    {

        for (int i = 0; i < msg.name.size(); i++)
        {

            if (msg.name[i] != "mobile_base")
            {
                continue;
            }

            if (currentClockLocal.clock.toSec() > startAgentSimTime)
            {
                robotPosition.position.x = msg.pose[i].position.x;
                robotPosition.position.y = msg.pose[i].position.y;
                robotPosition.position.z = msg.pose[i].position.z;
                robotPosition.orientation.x = msg.pose[i].orientation.x;
                robotPosition.orientation.y = msg.pose[i].orientation.y;
                robotPosition.orientation.z = msg.pose[i].orientation.z;
                robotPosition.orientation.w = msg.pose[i].orientation.w;

                robotPositionsFileName << currentClockLocal.clock.toSec() << "," << robotPosition.position.x << "," << robotPosition.position.y << "\n";

                if (!initPoseStored)
                {
                    initRobotPosition = robotPosition;
                    lastClock = currentClockLocal;
                    lastRobotPosition = robotPosition;
                    initPoseStored = !initPoseStored;
                }
                else
                {
                    double deltaPathLength = sqrt(pow(msg.pose[i].position.x - lastRobotPosition.position.x, 2) + pow(msg.pose[i].position.y - lastRobotPosition.position.y, 2));

                    const double epsilon = 1e-6; // Example epsilon value, adjust as needed
                    if (std::fabs(ros::Time::now().toSec() - lastClock.clock.toSec()) > 0.1f + epsilon && deltaPathLength > 0.1f + epsilon)
                    {
                        pathLenght += deltaPathLength;

                        double currentAgentSpeed = deltaPathLength / (ros::Time::now().toSec() - lastClock.clock.toSec());
                        speedVector.push_back(currentAgentSpeed);
                        lastClock = currentClockLocal;
                        lastRobotPosition = robotPosition;
                    }
                }
            }

            double distance2goal = (double)std::sqrt(std::pow(robotPosition.position.x - goalX, 2) + std::pow(robotPosition.position.y - goalY, 2));

            if (!goalReached && distance2goal < 0.2)
            {
                // Store Time In Personal Area Metrics
                double maxTimeInPersonalZone = 0.0;
                int maxTimeInPersonalZoneAgent = 0;
                timeInPersonalAreaFileName << "Agent Id"
                                           << ","
                                           << "Init Time (s)"
                                           << ","
                                           << "End Time (s)"
                                           << ","
                                           << "Time In Personal Area"
                                           << "\n";
                for (const TimeInPersonalZone &timeInZone : timeInPersonalZoneVector)
                {
                    double totalTimeTmp = timeInZone.endTime - timeInZone.initTime;
                    if (maxTimeInPersonalZone < totalTimeTmp)
                    {
                        maxTimeInPersonalZone = totalTimeTmp;
                        maxTimeInPersonalZoneAgent = timeInZone.agentId;
                    }
                    timeInPersonalAreaFileName << timeInZone.agentId
                                               << ","
                                               << timeInZone.initTime
                                               << ","
                                               << timeInZone.endTime
                                               << ","
                                               << totalTimeTmp
                                               << "\n";
                }

                double elapsedTime = ros::Time::now().toSec() - startAgentSimTime;

                // Store Main Metrics
                timeGoalFileName << "Init Robot Position X"
                                 << ","
                                 << "Init Robot Position Y"
                                 << ","
                                 << "Goal Robot Position X"
                                 << ","
                                 << "Goal Robot Position Y"
                                 << ","
                                 << "Time to goal (s)"
                                 << ","
                                 << "Path Lenght to goal (m)"
                                 << ","
                                 << "Avg Robot Speed (m/s)"
                                 << ","
                                 << "Min Ditance to Person (m)"
                                 << ","
                                 << "Max Time in Personal Area (s)"
                                 << ","
                                 << "Agent Id"
                                 << ","
                                 << "Ped. Collitions"
                                 << ","
                                 << "Ped. lower than min distance"
                                 << "\n";

                timeGoalFileName << initRobotPosition.position.x
                                 << ","
                                 << initRobotPosition.position.y
                                 << ","
                                 << msg.pose[i].position.x
                                 << ","
                                 << msg.pose[i].position.y
                                 << ","
                                 << elapsedTime
                                 << ","
                                 << pathLenght
                                 << ","
                                 << calculateAverage(speedVector)
                                 << ","
                                 << minDistance2Person
                                 << ","
                                 << maxTimeInPersonalZone
                                 << ","
                                 << maxTimeInPersonalZoneAgent
                                 << ","
                                 << pedestrian_collistions
                                 << ","
                                 << pedestrian_min_distance
                                 << "\n";

                goalReached = !goalReached; // Update the goalReached flag.
            }
        }

        for (int i = 0; i < msg.name.size(); i++)
        {

            std::regex pattern("actor(\\d+)");
            std::smatch matches;

            if (msg.name[i].find("actor") != std::string::npos && std::regex_search(msg.name[i], matches, pattern) && currentClockLocal.clock.toSec() > startAgentSimTime && msg.pose[i].position.z > 0)
            {
                double distance = std::sqrt(std::pow(robotPosition.position.x - msg.pose[i].position.x, 2) + std::pow(robotPosition.position.y - msg.pose[i].position.y, 2));

                if (distance < distance_collition)
                {
                    auto it = std::find(pedestrian_collitions_id.begin(), pedestrian_collitions_id.end(), matches[1].str());

                    if (it != pedestrian_collitions_id.end())
                    {
                        // std::cout << "Element found: " << *it << std::endl;
                    }
                    else
                    {
                        pedestrian_collitions_id.push_back(matches[1].str());
                        pedestrian_collistions++;
                    }
                }

                if (distance < min_distance_person)
                {
                    auto it = std::find(pedestrian_min_distance_id.begin(), pedestrian_min_distance_id.end(), matches[1].str());

                    if (it != pedestrian_min_distance_id.end())
                    {
                        // std::cout << "Element found: " << *it << std::endl;
                    }
                    else
                    {
                        pedestrian_min_distance_id.push_back(matches[1].str());
                        pedestrian_min_distance++;
                    }
                }

                distanceFileName << currentClockLocal.clock.toSec()
                                 << ","
                                 << matches[1].str()
                                 << ","
                                 << distance
                                 << "\n";

                if (minDistance2Person > distance)
                {
                    minDistance2Person = distance;
                }

                int currentAgentId = std::stoi(matches[1].str());

                auto agentInZone = std::find_if(timeInPersonalZoneVector.begin(), timeInPersonalZoneVector.end(),
                                                [currentAgentId](const TimeInPersonalZone &timeInZone)
                                                {
                                                    return timeInZone.agentId == currentAgentId;
                                                });

                if (distance < personalAreaRadius && agentInZone == timeInPersonalZoneVector.end())
                {
                    timeInPersonalZoneVector.push_back(TimeInPersonalZone(currentClockLocal.clock.toSec(), currentClockLocal.clock.toSec(), currentAgentId));
                }
                else if (distance > personalAreaRadius && agentInZone != timeInPersonalZoneVector.end())
                {

                    size_t agentIndex = std::distance(timeInPersonalZoneVector.begin(), agentInZone);
                    if (timeInPersonalZoneVector[agentIndex].endTime == timeInPersonalZoneVector[agentIndex].initTime)
                    {
                        timeInPersonalZoneVector[agentIndex].endTime = currentClockLocal.clock.toSec();
                    }
                }
            }
        }

        timeVector.push_back(currentClockLocal.clock.toSec());
    }
}

// Callback for the clock topic
void clockCallback(const rosgraph_msgs::Clock msg)
{
    currentClock = msg;
}

double calculateAverage(const std::vector<double> &vec)
{
    double sum = 0.0;
    for (const double &value : vec)
    {
        sum += value;
    }

    // Check if the vector is not empty to avoid division by zero
    if (!vec.empty())
    {
        return sum / static_cast<double>(vec.size());
    }
    else
    {
        // Return some default value, or you can handle this case differently based on your requirements.
        return 0.0;
    }
}
