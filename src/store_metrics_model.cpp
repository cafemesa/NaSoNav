#include "ros/ros.h"
#include "std_msgs/String.h"
#include "gazebo_msgs/ModelStates.h"
#include "rosgraph_msgs/Clock.h"
#include "geometry_msgs/PoseWithCovarianceStamped.h"
#include "geometry_msgs/PoseStamped.h"
#include <iostream>
#include <fstream>
#include <regex>

std::ofstream distanceFileName, timeGoalFileName, robotPositionsFileName, timeInPersonalAreaFileName;
rosgraph_msgs::Clock currentClock, lastClock;
double initTime = 0;
double endTime = 0;
double personalAreaRadius = 1.2;
double distance_collition = 0.45;
double pathLenght = 0.0;
double minDistance2Person = 10000;
std::vector<std::string> pedestrian_collitions_id;
int pedestrian_collistions = 0;
std::vector<double> timeVector, speedVector;
int actorId;
bool goalReached = false;
geometry_msgs::Pose initAgentPosition, lastAgentPosition;
bool initPoseStored = false;
double startAgentSimTime = 140.0;

// Function prototypes
void modelCallback(const gazebo_msgs::ModelStates &msg);
void clockCallback(const rosgraph_msgs::Clock &msg);
void robotPoseCallback(const geometry_msgs::PoseWithCovarianceStamped &msg);
void goalPoseCallback(const geometry_msgs::PoseStamped &msg);
double calculateAverage(const std::vector<double> &vec);

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

int main(int argc, char **argv)
{
    ros::init(argc, argv, "store_metrics_model");
    ros::NodeHandle n;
    ros::NodeHandle nh("~");

    // File names
    std::string distance2personsFile, time2goalFile, pathLenghtFile, timeInPersonalAreaFile, robotPositionsFile;
    nh.param<std::string>("distance2personsFileName", distance2personsFile, "");
    nh.param<std::string>("time2goalFilename", time2goalFile, "");
    nh.param<std::string>("timeInPersonAreaFileName", timeInPersonalAreaFile, "");
    nh.param<std::string>("robotPositionsFileName", robotPositionsFile, "");

    // Open files
    distanceFileName.open(distance2personsFile);
    timeGoalFileName.open(time2goalFile);
    timeInPersonalAreaFileName.open(timeInPersonalAreaFile);
    robotPositionsFileName.open(robotPositionsFile);

    distanceFileName << "Time (s)"
                     << ","
                     << "Agent ID"
                     << ","
                     << "Distance"
                     << "\n";
    robotPositionsFileName << "Time (s),x,y\n";

    // Variables
    nh.param<double>("personalAreaRadius", personalAreaRadius, 1.2);
    nh.param<double>("distance_collition", distance_collition, 0.45);

    // Subscribers
    ros::Subscriber subClock = n.subscribe("/clock", 1000, clockCallback);
    ros::Subscriber subModelStates = n.subscribe("/gazebo/model_states", 1000, modelCallback);

    nh.param<int>("actorId", actorId, 0);

    ros::spin();

    // Close files
    distanceFileName.close();
    timeGoalFileName.close();
    timeInPersonalAreaFileName.close();

    return 0;
}

void modelCallback(const gazebo_msgs::ModelStates &msg)
{

    rosgraph_msgs::Clock currentClockLocal = currentClock;

    if (std::find(timeVector.begin(), timeVector.end(), currentClockLocal.clock.toSec()) != timeVector.end())
    {
        return;
    }

    geometry_msgs::Pose agentPosition;

    for (int i = 0; i < msg.name.size(); i++)
    {

        if (msg.name[i] == "actor" + std::to_string(actorId) && currentClockLocal.clock.toSec() > startAgentSimTime && msg.pose[i].position.z > 0)
        {

            agentPosition.position.x = msg.pose[i].position.x;
            agentPosition.position.y = msg.pose[i].position.y;
            agentPosition.position.z = msg.pose[i].position.z;
            agentPosition.orientation.x = msg.pose[i].orientation.x;
            agentPosition.orientation.y = msg.pose[i].orientation.y;
            agentPosition.orientation.z = msg.pose[i].orientation.z;
            agentPosition.orientation.w = msg.pose[i].orientation.w;

            robotPositionsFileName << currentClockLocal.clock.toSec() << "," << agentPosition.position.x << "," << agentPosition.position.y << "\n";


            if (!initPoseStored)
            {
                initAgentPosition = agentPosition;
                lastClock = currentClockLocal;
                lastAgentPosition = agentPosition;
                initPoseStored = !initPoseStored;
            }
            else
            {
                double deltaPathLength = sqrt(pow(msg.pose[i].position.x - lastAgentPosition.position.x, 2) + pow(msg.pose[i].position.y - lastAgentPosition.position.y, 2));

                const double epsilon = 1e-6; // Example epsilon value, adjust as needed
                if (std::fabs(ros::Time::now().toSec() - lastClock.clock.toSec()) > 0.1f + epsilon && deltaPathLength > 0.1f + epsilon)
                {
                    pathLenght += deltaPathLength;

                    double currentAgentSpeed = deltaPathLength / (ros::Time::now().toSec() - lastClock.clock.toSec());
                    speedVector.push_back(currentAgentSpeed);
                    lastClock = currentClockLocal;
                    lastAgentPosition = agentPosition;
                }
            }
        }

        if (!goalReached && msg.pose[i].position.z < 0 && msg.name[i] == "actor" + std::to_string(actorId) && currentClockLocal.clock.toSec() > (double)startAgentSimTime + 0.5)
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
            timeGoalFileName << "Init Agent Position X"
                             << ","
                             << "Init Agent Position Y"
                             << ","
                             << "Goal Agent Position X"
                             << ","
                             << "Goal Agent Position Y"
                             << ","
                             << "Time to goal (s)"
                             << ","
                             << "Path Lenght to goal (m)"
                             << ","
                             << "Avg Agent Speed (m/s)"
                             << ","
                             << "Min Ditance to Person (m)"
                             << ","
                             << "Max Time in Personal Area (s)"
                             << ","
                             << "Agent Id"
                             << ","
                             << "Ped. Collitions"
                             << "\n";

            timeGoalFileName << initAgentPosition.position.x
                             << ","
                             << initAgentPosition.position.y
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
                             << "\n";

            goalReached = !goalReached; // Update the goalReached flag.
        }
    }

    for (int i = 0; i < msg.name.size(); i++)
    {

        if (msg.name[i] == "actor" + std::to_string(actorId))
        {
            continue;
        }

        std::regex pattern("actor(\\d+)");
        std::smatch matches;

        if (msg.name[i].find("actor") != std::string::npos && std::regex_search(msg.name[i], matches, pattern) && currentClockLocal.clock.toSec() > startAgentSimTime && msg.pose[i].position.z > 0 && agentPosition.position.z > 0)
        {
            double distance = std::sqrt(std::pow(agentPosition.position.x - msg.pose[i].position.x, 2) + std::pow(agentPosition.position.y - msg.pose[i].position.y, 2));

            if (distance < distance_collition)
            {
                auto it = std::find(pedestrian_collitions_id.begin(), pedestrian_collitions_id.end(), matches[1].str());

                if (it != pedestrian_collitions_id.end())
                {
                    //std::cout << "Element found: " << *it << std::endl;
                }
                else
                {
                    pedestrian_collitions_id.push_back(matches[1].str());
                    pedestrian_collistions++;
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

void clockCallback(const rosgraph_msgs::Clock &msg)
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
