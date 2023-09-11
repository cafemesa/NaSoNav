#include <iostream>
#include <fstream>
#include <vector>
#include <cmath>
#include <algorithm>
#include <ros/ros.h>

#include "geometry_msgs/Point.h"

struct DatasetLine
{
  double frameId, personId, positionX, positionY, positionZ, speedX, speedY, speedZ;

  DatasetLine(double col1, double col2, double col3, double col4, double col5, double col6, double col7, double col8)
      : frameId(col1), personId(col2), positionX(col3), positionY(col5), positionZ(col4), speedX(col6), speedY(col8), speedZ(col7) {}
};

void logger(const std::string &message, const std::string &filename)
{
  std::ofstream outfile(filename, std::ios_base::app);
  if (outfile.is_open())
  {
    outfile << message << '\n';
    outfile.close();
  }
  else
  {
    std::cout << "Failed to open file: " << filename << std::endl;
  }
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "create_simulated_environments");
  ros::NodeHandle n;
  ros::NodeHandle nh("~");

  std::string inputfile;
  std::string logfolder;

  // Retrieve file paths from ROS parameters
  if (!nh.getParam("input_file", inputfile))
  {
    ROS_ERROR("Failed to get input_file parameter");
    return 1;
  }

  if (!nh.getParam("output_worlds_folder", logfolder))
  {
    ROS_ERROR("Failed to get output_log parameter");
    return 1;
  }

  // Set Frame ranges
  int initFrames[] = {824, 2199, 4249, 6299, 7299, 7749, 8349, 9624, 10324, 10625};
  int endFrames[] = {1026, 2399, 4449, 6474, 7449, 7899, 8499, 9724, 10499, 10950};
  int hiddenPersonsIds[] = {7, 48, 77, 124, 150, 159, 189, 226, 273, 303};

  double startTime = 140.0;

  for (int j = 0; j < sizeof(initFrames) / sizeof(*initFrames); ++j)
  {
    double maxAgentSpeedInScene = 0;
    int minFrameId = 10000000;

    std::vector<DatasetLine> datasetLineVector;
    std::vector<int> userIdVectorScene;

    // Open dataset annotations file and handle errors
    std::ifstream infile(inputfile);
    if (infile.fail())
    {
      std::cout << "Failed to open file: " << inputfile << std::endl;
      return 1;
    }

    // Store all dataset lines in datasetLineVector and store the max speed in the scene
    double col1, col2, col3, col4, col5, col6, col7, col8;
    while (infile >> col1 >> col2 >> col3 >> col4 >> col5 >> col6 >> col7 >> col8)
    {
      // Get dataset line
      DatasetLine datasetLine(col1, col2, col3, col4, col5, col6, col7, col8);
      // Store dataset line in datasetLineVector
      datasetLineVector.push_back(datasetLine);

      // If frame current frame ID is part of the scene store person Id and max speed
      if (datasetLine.frameId >= initFrames[j] && datasetLine.frameId <= endFrames[j])
      {
        userIdVectorScene.push_back(datasetLine.personId);
        // maxAgentSpeedInScene = std::max(maxAgentSpeedInScene, std::hypot(datasetLine.speedX, datasetLine.speedY));
      }

      if (minFrameId == 10000000 && datasetLine.personId == hiddenPersonsIds[j])
      {
        minFrameId = datasetLine.frameId;
      }
    }
    infile.close();

    // Sort and clean userIdVectorScene
    std::sort(userIdVectorScene.begin(), userIdVectorScene.end());
    userIdVectorScene.erase(std::unique(userIdVectorScene.begin(), userIdVectorScene.end()), userIdVectorScene.end());

    for (int aaa = 0; aaa != userIdVectorScene.size(); ++aaa)
    {
      bool isFirsPosition = true;
      geometry_msgs::Point lastPose;
      int lastFrameIdTmp = 0;
      for (int bbb = 0; bbb != datasetLineVector.size(); ++bbb)
      {
        if (datasetLineVector[bbb].personId == userIdVectorScene[aaa])
        {
          if (isFirsPosition)
          {
            isFirsPosition = !isFirsPosition;
          }
          else
          {
            double currentDistanceTmp = (double)sqrt(pow(datasetLineVector[bbb].positionX-lastPose.x,2)+pow(datasetLineVector[bbb].positionY-lastPose.y,2));
            double currentSpeedTmp = (double)currentDistanceTmp/((datasetLineVector[bbb].frameId-lastFrameIdTmp)*0.04);
            if(currentSpeedTmp>maxAgentSpeedInScene){
              maxAgentSpeedInScene = currentSpeedTmp;
            }
          }
          lastPose.x = datasetLineVector[bbb].positionX;
          lastPose.y = datasetLineVector[bbb].positionY;
          lastFrameIdTmp = datasetLineVector[bbb].frameId;
        }
      }
    }

    // Define speed factos for simulations
    double speedLimits[] = {0.3, 0.4, 0.5, 0.7, 1.0, 1.5, maxAgentSpeedInScene};
    for (int k = 0; k < sizeof(speedLimits) / sizeof(*speedLimits); k++)
    {
      // Get speed factor
      double speedFactor = maxAgentSpeedInScene / speedLimits[k];

      for (int kkk = 0; kkk < 2; kkk++)
      {
        // Set output file name based on the scene and max speed
        std::string logfile = logfolder + "scene" + std::to_string(j) + "_" + std::to_string(speedLimits[k]).substr(0, 3) + "_" + std::to_string(kkk) + ".world";

        logger("<sdf version='1.7'><world name='default'>", logfile);

        for (int i = 0; i < userIdVectorScene.size(); i++)
        {

          if ((hiddenPersonsIds[j] == userIdVectorScene[i] && kkk == 1) || (304 == userIdVectorScene[i] && kkk == 1 && j == 9))
          {
            logger("<!--actor name='actor" + std::to_string(userIdVectorScene[i]) + "'><skin><filename>walk.dae</filename></skin><animation name='walking'><filename>walk.dae</filename><interpolate_x>true</interpolate_x></animation><script><trajectory id='0' type='walking'>", logfile);
          }
          else
          {
            logger("<actor name='actor" + std::to_string(userIdVectorScene[i]) + "'><skin><filename>walk.dae</filename></skin><animation name='walking'><filename>walk.dae</filename><interpolate_x>true</interpolate_x></animation><script><trajectory id='0' type='walking'>", logfile);
          }

          int pathCounter = 0;
          double lastX = 0.0;
          double lastY = 0.0;
          double lastA = 0.0;
          int lastFrame = 0;
          int lastArrayPosition = 0;
          for (int l = 0; l < datasetLineVector.size(); l++)
          {
            if (datasetLineVector[l].personId == userIdVectorScene[i])
            {

              lastA = (double)atan2(datasetLineVector[l].speedY, datasetLineVector[l].speedX);
              lastX = datasetLineVector[l].positionX;
              lastY = datasetLineVector[l].positionY;
              lastFrame = datasetLineVector[l].frameId;

              if (pathCounter < 1)
              {
                logger("<waypoint><time>0</time><pose>" + std::to_string(lastX) + " " + std::to_string(lastY) + " -2 0 0 " + std::to_string(lastA) + "</pose></waypoint>", logfile);

                logger("<waypoint><time>" + std::to_string((double)((datasetLineVector[l].frameId - minFrameId) * 0.04 * speedFactor) + startTime - 0.0001) + "</time><pose>" + std::to_string(lastX) + " " + std::to_string(lastY) + " -2 0 0 " + std::to_string(lastA) + "</pose></waypoint>", logfile);
                pathCounter++;
              }

              // TODO Improve angle of persons
              // else {
              //   lastA = (double)atan2(datasetLineVector[l].positionY - datasetLineVector[lastArrayPosition].positionY, datasetLineVector[l].positionX-datasetLineVector[lastArrayPosition].positionX);
              // }

              // lastArrayPosition = l;

              logger("<waypoint><time>" + std::to_string((double)((lastFrame - minFrameId) * 0.04 * speedFactor) + startTime) + "</time><pose>" + std::to_string(lastX) + " " + std::to_string(lastY) + " 0 0 0 " + std::to_string(lastA) + "</pose></waypoint>", logfile);
            }
          }

          logger("<waypoint><time>" + std::to_string((double)((lastFrame - minFrameId) * 0.04 * speedFactor) + startTime + 0.0001) + "</time><pose>" + std::to_string(lastX) + " " + std::to_string(lastY) + " -2 0 0 " + std::to_string(lastA) + "</pose></waypoint>", logfile);

          logger("<waypoint><time>500</time><pose>" + std::to_string(lastX) + " " + std::to_string(lastY) + " -2 0 0 " + std::to_string(lastA) + "</pose></waypoint>", logfile);

          if ((hiddenPersonsIds[j] == userIdVectorScene[i] && kkk == 1) || (304 == userIdVectorScene[i] && kkk == 1 && j == 9))
          {
            logger("</trajectory></script></actor-->", logfile);
          }
          else
          {
            logger("</trajectory></script></actor>", logfile);
          }
        }

        logger("<light name='sun' type='directional'><cast_shadows>1</cast_shadows><pose>0 0 10 0 -0 0</pose><diffuse>0.8 0.8 0.8 1</diffuse><specular>0.2 0.2 0.2 1</specular><attenuation><range>1000</range><constant>0.9</constant><linear>0.01</linear><quadratic>0.001</quadratic></attenuation><direction>-0.5 0.1 -0.9</direction><spot><inner_angle>0</inner_angle><outer_angle>0</outer_angle><falloff>0</falloff></spot></light><model name='ground_plane'><static>1</static><link name='link'><collision name='collision'><geometry><plane><normal>0 0 1</normal><size>100 100</size></plane></geometry><surface><contact><collide_bitmask>65535</collide_bitmask><ode /></contact><friction><ode><mu>100</mu><mu2>50</mu2></ode><torsional><ode /></torsional></friction><bounce /></surface><max_contacts>10</max_contacts></collision><visual name='visual'><cast_shadows>0</cast_shadows><geometry><plane><normal>0 0 1</normal><size>100 100</size></plane></geometry><material><script><uri>file://media/materials/scripts/gazebo.material</uri><name>Gazebo/Grey</name></script></material></visual><self_collide>0</self_collide><enable_wind>0</enable_wind><kinematic>0</kinematic></link></model><gravity>0 0 -9.8</gravity><magnetic_field>6e-06 2.3e-05 -4.2e-05</magnetic_field><atmosphere type='adiabatic' /><physics type='ode'><max_step_size>0.001</max_step_size><real_time_factor>1</real_time_factor><real_time_update_rate>1000</real_time_update_rate></physics><scene><ambient>0.4 0.4 0.4 1</ambient><background>0.7 0.7 0.7 1</background><shadows>1</shadows></scene><wind /><spherical_coordinates><surface_model>EARTH_WGS84</surface_model><latitude_deg>0</latitude_deg><longitude_deg>0</longitude_deg><elevation>0</elevation><heading_deg>0</heading_deg></spherical_coordinates><state world_name='default'><sim_time>130 0</sim_time><real_time>0 0</real_time><wall_time>1682666033 172342014</wall_time><iterations>0</iterations><model name='ground_plane'><pose>0 0 0 0 -0 0</pose><scale>1 1 1</scale><link name='link'><pose>0 0 0 0 -0 0</pose><velocity>0 0 0 0 -0 0</velocity><acceleration>0 0 0 0 -0 0</acceleration><wrench>0 0 0 0 -0 0</wrench></link></model><model name='mywall1'><pose>28.849 -8.5 0 1.57 -0 0</pose><scale>1 1 1</scale><link name='link'><pose>28.849 -8.5 0 1.57 -0 0</pose><velocity>0 0 0 0 -0 0</velocity><acceleration>0 0 0 0 -0 0</acceleration><wrench>0 0 0 0 -0 0</wrench></link></model><model name='mywall2'><pose>1.5 10.0302 -0.000422 1.57 -0 0</pose><scale>1 1 1</scale><link name='link'><pose>1.5 10.0302 -0.000422 1.57 -0 0</pose><velocity>0 0 0 0 -0 0</velocity><acceleration>0 0 0 0 -0 0</acceleration><wrench>0 0 0 0 -0 0</wrench></link></model><model name='wall3'><pose>16 4.75 0 0 -0 0</pose><scale>1 1 1</scale><link name='link'><pose>16 4.75 0 0 -0 0</pose><velocity>0 0 0 0 -0 0</velocity><acceleration>0 0 0 0 -0 0</acceleration><wrench>0 0 0 0 -0 0</wrench></link></model><model name='wall4'><pose>16 7.25 0 0 -0 0</pose><scale>1 1 1</scale><link name='link'><pose>16 7.25 0 0 -0 0</pose><velocity>0 0 0 0 -0 0</velocity><acceleration>0 0 0 0 -0 0</acceleration><wrench>0 0 0 0 -0 0</wrench></link></model><model name='wall5'><pose>16 9.75 0 0 -0 0</pose><scale>1 1 1</scale><link name='link'><pose>16 9.75 0 0 -0 0</pose><velocity>0 0 0 0 -0 0</velocity><acceleration>0 0 0 0 -0 0</acceleration><wrench>0 0 0 0 -0 0</wrench></link></model><model name='column1'><pose>-3.5 1.97 0 0 -0 0</pose><scale>1 1 1</scale><link name='Wall_4'><pose>-3.5 1.97 0 0 -0 0</pose><velocity>0 0 0 0 -0 0</velocity><acceleration>0 0 0 0 -0 0</acceleration><wrench>0 0 0 0 -0 0</wrench></link></model><model name='column2'><pose>-3.5 3.97 0 0 -0 0</pose><scale>1 1 1</scale><link name='Wall_4'><pose>-3.5 3.97 0 0 -0 0</pose><velocity>0 0 0 0 -0 0</velocity><acceleration>0 0 0 0 -0 0</acceleration><wrench>0 0 0 0 -0 0</wrench></link></model><model name='column3'><pose>-3.5 5.97 0 0 -0 0</pose><scale>1 1 1</scale><link name='Wall_4'><pose>-3.5 5.97 0 0 -0 0</pose><velocity>0 0 0 0 -0 0</velocity><acceleration>0 0 0 0 -0 0</acceleration><wrench>0 0 0 0 -0 0</wrench></link></model><model name='column4'><pose>-3.5 7.97 0 0 -0 0</pose><scale>1 1 1</scale><link name='Wall_4'><pose>-3.5 7.97 0 0 -0 0</pose><velocity>0 0 0 0 -0 0</velocity><acceleration>0 0 0 0 -0 0</acceleration><wrench>0 0 0 0 -0 0</wrench></link></model><model name='column5'><pose>-3.5 9.97 0 0 -0 0</pose><scale>1 1 1</scale><link name='Wall_4'><pose>-3.5 9.97 0 0 -0 0</pose><velocity>0 0 0 0 -0 0</velocity><acceleration>0 0 0 0 -0 0</acceleration><wrench>0 0 0 0 -0 0</wrench></link></model><light name='sun'><pose>0 0 10 0 -0 0</pose></light></state><gui fullscreen='0'><camera name='user_camera'><pose>12.4751 -9.84208 4.18899 0 0.275643 2.35619</pose><view_controller>orbit</view_controller><projection_type>perspective</projection_type></camera></gui><model name='mywall1'><static>1</static><link name='link'><collision name='collision'><geometry><mesh><scale>0.01 0.01 0.01</scale><uri>model://mywall1/meshes/Pieza3.obj</uri></mesh></geometry><max_contacts>10</max_contacts><surface><contact><ode/></contact><bounce/><friction><torsional><ode/></torsional><ode/></friction></surface></collision><visual name='visual'><geometry><mesh><scale>0.01 0.01 0.01</scale><uri>model://mywall1/meshes/Pieza3.obj</uri></mesh></geometry></visual><self_collide>0</self_collide><enable_wind>0</enable_wind><kinematic>0</kinematic></link><pose>28.5 -8.5 0 1.57 -0 0</pose></model><model name='mywall2'><static>1</static><link name='link'><collision name='collision'><geometry><mesh><scale>0.01 0.01 0.01</scale><uri>model://mywall2/meshes/Pieza2.obj</uri></mesh></geometry><max_contacts>10</max_contacts><surface><contact><ode/></contact><bounce/><friction><torsional><ode/></torsional><ode/></friction></surface></collision><visual name='visual'><geometry><mesh><scale>0.01 0.01 0.01</scale><uri>model://mywall2/meshes/Pieza2.obj</uri></mesh></geometry></visual><self_collide>0</self_collide><enable_wind>0</enable_wind><kinematic>0</kinematic></link><pose>1.5 10.0302 -0.000422 1.57 -0 0</pose></model><model name='wall3'><pose>16 4.75 0 0 -0 0</pose><link name='Wall_0'><collision name='Wall_0_Collision'><geometry><box><size>1 0.15 2.5</size></box></geometry><pose>0 0 1.25 0 -0 0</pose><max_contacts>10</max_contacts><surface><contact><ode/></contact><bounce/><friction><torsional><ode/></torsional><ode/></friction></surface></collision><visual name='Wall_0_Visual'><pose>0 0 1.25 0 -0 0</pose><geometry><box><size>1 0.15 2.5</size></box></geometry><material><script><uri>file://media/materials/scripts/gazebo.material</uri><name>Gazebo/Grey</name></script><ambient>1 1 1 1</ambient></material><meta><layer>0</layer></meta></visual><pose>-0.1975 -1.0975 0 0 -0 0</pose><self_collide>0</self_collide><enable_wind>0</enable_wind><kinematic>0</kinematic></link><link name='Wall_1'><collision name='Wall_1_Collision'><geometry><box><size>1 0.15 2.5</size></box></geometry><pose>0 0 1.25 0 -0 0</pose><max_contacts>10</max_contacts><surface><contact><ode/></contact><bounce/><friction><torsional><ode/></torsional><ode/></friction></surface></collision><visual name='Wall_1_Visual'><pose>0 0 1.25 0 -0 0</pose><geometry><box><size>1 0.15 2.5</size></box></geometry><material><script><uri>file://media/materials/scripts/gazebo.material</uri><name>Gazebo/Grey</name></script><ambient>1 1 1 1</ambient></material><meta><layer>0</layer></meta></visual><pose>0.2275 -1.5225 0 0 -0 -1.5708</pose><self_collide>0</self_collide><enable_wind>0</enable_wind><kinematic>0</kinematic></link><link name='Wall_2'><collision name='Wall_2_Collision'><geometry><box><size>1 0.15 2.5</size></box></geometry><pose>0 0 1.25 0 -0 0</pose><max_contacts>10</max_contacts><surface><contact><ode/></contact><bounce/><friction><torsional><ode/></torsional><ode/></friction></surface></collision><visual name='Wall_2_Visual'><pose>0 0 1.25 0 -0 0</pose><geometry><box><size>1 0.15 2.5</size></box></geometry><material><script><uri>file://media/materials/scripts/gazebo.material</uri><name>Gazebo/Grey</name></script><ambient>1 1 1 1</ambient></material><meta><layer>0</layer></meta></visual><pose>-0.1975 -1.9475 0 0 -0 3.14159</pose><self_collide>0</self_collide><enable_wind>0</enable_wind><kinematic>0</kinematic></link><link name='Wall_3'><collision name='Wall_3_Collision'><geometry><box><size>1 0.15 2.5</size></box></geometry><pose>0 0 1.25 0 -0 0</pose><max_contacts>10</max_contacts><surface><contact><ode/></contact><bounce/><friction><torsional><ode/></torsional><ode/></friction></surface></collision><visual name='Wall_3_Visual'><pose>0 0 1.25 0 -0 0</pose><geometry><box><size>1 0.15 2.5</size></box></geometry><material><script><uri>file://media/materials/scripts/gazebo.material</uri><name>Gazebo/Grey</name></script><ambient>1 1 1 1</ambient></material><meta><layer>0</layer></meta></visual><pose>-0.6225 -1.5225 0 0 -0 1.5708</pose><self_collide>0</self_collide><enable_wind>0</enable_wind><kinematic>0</kinematic></link><static>1</static></model><model name='wall4'><pose>16 7.25 0 0 -0 0</pose><link name='Wall_0'><collision name='Wall_0_Collision'><geometry><box><size>1 0.15 2.5</size></box></geometry><pose>0 0 1.25 0 -0 0</pose><max_contacts>10</max_contacts><surface><contact><ode/></contact><bounce/><friction><torsional><ode/></torsional><ode/></friction></surface></collision><visual name='Wall_0_Visual'><pose>0 0 1.25 0 -0 0</pose><geometry><box><size>1 0.15 2.5</size></box></geometry><material><script><uri>file://media/materials/scripts/gazebo.material</uri><name>Gazebo/Grey</name></script><ambient>1 1 1 1</ambient></material><meta><layer>0</layer></meta></visual><pose>-0.1975 -1.0975 0 0 -0 0</pose><self_collide>0</self_collide><enable_wind>0</enable_wind><kinematic>0</kinematic></link><link name='Wall_1'><collision name='Wall_1_Collision'><geometry><box><size>1 0.15 2.5</size></box></geometry><pose>0 0 1.25 0 -0 0</pose><max_contacts>10</max_contacts><surface><contact><ode/></contact><bounce/><friction><torsional><ode/></torsional><ode/></friction></surface></collision><visual name='Wall_1_Visual'><pose>0 0 1.25 0 -0 0</pose><geometry><box><size>1 0.15 2.5</size></box></geometry><material><script><uri>file://media/materials/scripts/gazebo.material</uri><name>Gazebo/Grey</name></script><ambient>1 1 1 1</ambient></material><meta><layer>0</layer></meta></visual><pose>0.2275 -1.5225 0 0 -0 -1.5708</pose><self_collide>0</self_collide><enable_wind>0</enable_wind><kinematic>0</kinematic></link><link name='Wall_2'><collision name='Wall_2_Collision'><geometry><box><size>1 0.15 2.5</size></box></geometry><pose>0 0 1.25 0 -0 0</pose><max_contacts>10</max_contacts><surface><contact><ode/></contact><bounce/><friction><torsional><ode/></torsional><ode/></friction></surface></collision><visual name='Wall_2_Visual'><pose>0 0 1.25 0 -0 0</pose><geometry><box><size>1 0.15 2.5</size></box></geometry><material><script><uri>file://media/materials/scripts/gazebo.material</uri><name>Gazebo/Grey</name></script><ambient>1 1 1 1</ambient></material><meta><layer>0</layer></meta></visual><pose>-0.1975 -1.9475 0 0 -0 3.14159</pose><self_collide>0</self_collide><enable_wind>0</enable_wind><kinematic>0</kinematic></link><link name='Wall_3'><collision name='Wall_3_Collision'><geometry><box><size>1 0.15 2.5</size></box></geometry><pose>0 0 1.25 0 -0 0</pose><max_contacts>10</max_contacts><surface><contact><ode/></contact><bounce/><friction><torsional><ode/></torsional><ode/></friction></surface></collision><visual name='Wall_3_Visual'><pose>0 0 1.25 0 -0 0</pose><geometry><box><size>1 0.15 2.5</size></box></geometry><material><script><uri>file://media/materials/scripts/gazebo.material</uri><name>Gazebo/Grey</name></script><ambient>1 1 1 1</ambient></material><meta><layer>0</layer></meta></visual><pose>-0.6225 -1.5225 0 0 -0 1.5708</pose><self_collide>0</self_collide><enable_wind>0</enable_wind><kinematic>0</kinematic></link><static>1</static></model><model name='wall5'><pose>16 9.75 0 0 -0 0</pose><link name='Wall_0'><collision name='Wall_0_Collision'><geometry><box><size>1 0.15 2.5</size></box></geometry><pose>0 0 1.25 0 -0 0</pose><max_contacts>10</max_contacts><surface><contact><ode/></contact><bounce/><friction><torsional><ode/></torsional><ode/></friction></surface></collision><visual name='Wall_0_Visual'><pose>0 0 1.25 0 -0 0</pose><geometry><box><size>1 0.15 2.5</size></box></geometry><material><script><uri>file://media/materials/scripts/gazebo.material</uri><name>Gazebo/Grey</name></script><ambient>1 1 1 1</ambient></material><meta><layer>0</layer></meta></visual><pose>-0.1975 -1.0975 0 0 -0 0</pose><self_collide>0</self_collide><enable_wind>0</enable_wind><kinematic>0</kinematic></link><link name='Wall_1'><collision name='Wall_1_Collision'><geometry><box><size>1 0.15 2.5</size></box></geometry><pose>0 0 1.25 0 -0 0</pose><max_contacts>10</max_contacts><surface><contact><ode/></contact><bounce/><friction><torsional><ode/></torsional><ode/></friction></surface></collision><visual name='Wall_1_Visual'><pose>0 0 1.25 0 -0 0</pose><geometry><box><size>1 0.15 2.5</size></box></geometry><material><script><uri>file://media/materials/scripts/gazebo.material</uri><name>Gazebo/Grey</name></script><ambient>1 1 1 1</ambient></material><meta><layer>0</layer></meta></visual><pose>0.2275 -1.5225 0 0 -0 -1.5708</pose><self_collide>0</self_collide><enable_wind>0</enable_wind><kinematic>0</kinematic></link><link name='Wall_2'><collision name='Wall_2_Collision'><geometry><box><size>1 0.15 2.5</size></box></geometry><pose>0 0 1.25 0 -0 0</pose><max_contacts>10</max_contacts><surface><contact><ode/></contact><bounce/><friction><torsional><ode/></torsional><ode/></friction></surface></collision><visual name='Wall_2_Visual'><pose>0 0 1.25 0 -0 0</pose><geometry><box><size>1 0.15 2.5</size></box></geometry><material><script><uri>file://media/materials/scripts/gazebo.material</uri><name>Gazebo/Grey</name></script><ambient>1 1 1 1</ambient></material><meta><layer>0</layer></meta></visual><pose>-0.1975 -1.9475 0 0 -0 3.14159</pose><self_collide>0</self_collide><enable_wind>0</enable_wind><kinematic>0</kinematic></link><link name='Wall_3'><collision name='Wall_3_Collision'><geometry><box><size>1 0.15 2.5</size></box></geometry><pose>0 0 1.25 0 -0 0</pose><max_contacts>10</max_contacts><surface><contact><ode/></contact><bounce/><friction><torsional><ode/></torsional><ode/></friction></surface></collision><visual name='Wall_3_Visual'><pose>0 0 1.25 0 -0 0</pose><geometry><box><size>1 0.15 2.5</size></box></geometry><material><script><uri>file://media/materials/scripts/gazebo.material</uri><name>Gazebo/Grey</name></script><ambient>1 1 1 1</ambient></material><meta><layer>0</layer></meta></visual><pose>-0.6225 -1.5225 0 0 -0 1.5708</pose><self_collide>0</self_collide><enable_wind>0</enable_wind><kinematic>0</kinematic></link><static>1</static></model><model name='column1'><pose>-3.5 1.97 0 0 -0 0</pose><link name='Wall_4'><collision name='Wall_4_Collision'><geometry><box><size>0.2 0.2 2.5</size></box></geometry><pose>0 0 0 0 -0 0</pose><max_contacts>10</max_contacts><surface><contact><ode/></contact><bounce/><friction><torsional><ode/></torsional><ode/></friction></surface></collision><visual name='Wall_4_Visual'><pose>0 0  0 0 -0 0</pose><geometry><box><size>0.2 0.2 2.5</size></box></geometry><material><script><uri>file://media/materials/scripts/gazebo.material</uri><name>Gazebo/Grey</name></script><ambient>1 1 1 1</ambient></material><meta><layer>0</layer></meta></visual><pose>0 0 0 0 -0 0</pose><self_collide>0</self_collide><enable_wind>0</enable_wind><kinematic>0</kinematic></link><static>1</static></model><model name='column2'><pose>-3.5 3.97 0 0 -0 0</pose><link name='Wall_4'><collision name='Wall_4_Collision'><geometry><box><size>0.2 0.2 2.5</size></box></geometry><pose>0 0 0 0 -0 0</pose><max_contacts>10</max_contacts><surface><contact><ode/></contact><bounce/><friction><torsional><ode/></torsional><ode/></friction></surface></collision><visual name='Wall_4_Visual'><pose>0 0  0 0 -0 0</pose><geometry><box><size>0.2 0.2 2.5</size></box></geometry><material><script><uri>file://media/materials/scripts/gazebo.material</uri><name>Gazebo/Grey</name></script><ambient>1 1 1 1</ambient></material><meta><layer>0</layer></meta></visual><pose>0 0 0 0 -0 0</pose><self_collide>0</self_collide><enable_wind>0</enable_wind><kinematic>0</kinematic></link><static>1</static></model><model name='column3'><pose>-3.5 5.97 0 0 -0 0</pose><link name='Wall_4'><collision name='Wall_4_Collision'><geometry><box><size>0.2 0.2 2.5</size></box></geometry><pose>0 0 0 0 -0 0</pose><max_contacts>10</max_contacts><surface><contact><ode/></contact><bounce/><friction><torsional><ode/></torsional><ode/></friction></surface></collision><visual name='Wall_4_Visual'><pose>0 0  0 0 -0 0</pose><geometry><box><size>0.2 0.2 2.5</size></box></geometry><material><script><uri>file://media/materials/scripts/gazebo.material</uri><name>Gazebo/Grey</name></script><ambient>1 1 1 1</ambient></material><meta><layer>0</layer></meta></visual><pose>0 0 0 0 -0 0</pose><self_collide>0</self_collide><enable_wind>0</enable_wind><kinematic>0</kinematic></link><static>1</static></model><model name='column4'><pose>-3.5 7.97 0 0 -0 0</pose><link name='Wall_4'><collision name='Wall_4_Collision'><geometry><box><size>0.2 0.2 2.5</size></box></geometry><pose>0 0 0 0 -0 0</pose><max_contacts>10</max_contacts><surface><contact><ode/></contact><bounce/><friction><torsional><ode/></torsional><ode/></friction></surface></collision><visual name='Wall_4_Visual'><pose>0 0  0 0 -0 0</pose><geometry><box><size>0.2 0.2 2.5</size></box></geometry><material><script><uri>file://media/materials/scripts/gazebo.material</uri><name>Gazebo/Grey</name></script><ambient>1 1 1 1</ambient></material><meta><layer>0</layer></meta></visual><pose>0 0 0 0 -0 0</pose><self_collide>0</self_collide><enable_wind>0</enable_wind><kinematic>0</kinematic></link><static>1</static></model><model name='column5'><pose>-3.5 9.97 0 0 -0 0</pose><link name='Wall_4'><collision name='Wall_4_Collision'><geometry><box><size>0.2 0.2 2.5</size></box></geometry><pose>0 0 0 0 -0 0</pose><max_contacts>10</max_contacts><surface><contact><ode/></contact><bounce/><friction><torsional><ode/></torsional><ode/></friction></surface></collision><visual name='Wall_4_Visual'><pose>0 0  0 0 -0 0</pose><geometry><box><size>0.2 0.2 2.5</size></box></geometry><material><script><uri>file://media/materials/scripts/gazebo.material</uri><name>Gazebo/Grey</name></script><ambient>1 1 1 1</ambient></material><meta><layer>0</layer></meta></visual><pose>0 0 0 0 -0 0</pose><self_collide>0</self_collide><enable_wind>0</enable_wind><kinematic>0</kinematic></link><static>1</static></model></world></sdf>", logfile);
      }
    }
  }

  return 0;
}
