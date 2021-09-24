/*********************************************************************************************************
*                                       Copyright Notice
*
*********************************************************************************************************/

/*********************************************************************************************************
*                                  COMMAND & RESPONSE OPERATIONS
* Filename      : mps_driver.cpp
* Version       : V1.0.1
* Programmers(s): Xiang He

**********************************************************************************************************
* Notes         : (1) n/a
*/
/* Includes ---------------------------------------------------------------------------------------------*/
#include "mpsFunc.h"
#include "uart.h"
#include <string>
#include <iostream>
#include <ros/ros.h>
#include <ros/console.h>
#include <geometry_msgs/PoseStamped.h>

using namespace std;

//Global Variables 

geometry_msgs::PoseStamped robot_pose;
bool pose_cb_flag = true;

sensor_msgs::NavSatFix c_gps;
nav_msgs::Odometry c_local_position;
ros::Publisher MPS_pub;
bool mps_init=false;

bool platformSim;

//#define DEBUG

/* Functions --------------------------------------------------------------------------------------------*/
void gps_callback(const sensor_msgs::NavSatFix &new_message)
{
    c_gps = new_message;
}

void local_position_callback(const nav_msgs::Odometry &new_message)
{
    c_local_position = new_message;
}

void init_MPS()
{
    // Turn on MPS through power control
    cout<<"Starting MPS."<<endl;
}

void mpsDataCallback(const ros::TimerEvent& event){
  if (mps_init){

    answer_t result;
    if(run_MPS(0x01, 0, &result) != 0){
      ROS_ERROR("MPS reading error!");
      exit(-2);
    }    
    printf("Normal\n");

#ifdef DEBUG
    
    printf("Analyte ID, Concentration [%%LEL], Temp [C], Pressure [kPa], Humidity [%%RH], Relative Humidity [%%H]\n");
    printf("%s,%f, %f, %f, %f, %f\n",result->flamID, result->concentration, result->temp, result->pressure, result->relHumidity, result->absHumidity);
    
    /*
    printf("Cycle Count, Methane Gas Concentration (ppm), GasID, Temp[C], Pressure [kPa], Rel_humidity[%%RH, Absolute Humidity [g/m^3]\n");
    printf("%s, %f, %f, %f, %f, %f, %f\n", result->cycleCount, resulult->concentration, result->ID, result->temp, result->pressure,result->relHumidity, result->absHumidity);
    */
#endif
    
    mps_driver::MPS mps_data;
    
    mps_data.gasID           = result.flamID;
    mps_data.percentLEL      = result.concentration;
    mps_data.temperature     = result.temp;
    mps_data.pressure        = result.pressure;
    mps_data.humidity        = result.relHumidity;
    mps_data.absHumidity     = result.absHumidity;
    mps_data.humidAirDensity = 0;
    
    /*
    mps_data.cycle           = result.cycleCount;
    mps_data.ppm             = result.concentration;
    mps_data.gasID           = result.flamID;
    mps_data.temperature     = result.temp;
    mps_data.pressure        = result.pressure;
    mps_data.humidity        = result.relHumidity;
    mps_data.absHumidity     = result.absHumidity;
    */
    mps_data.local_x         = c_local_position.pose.pose.position.x;
    mps_data.local_y         = c_local_position.pose.pose.position.y;
    mps_data.local_z         = c_local_position.pose.pose.position.z;
    mps_data.GPS_latitude    = c_gps.latitude;
    mps_data.GPS_longitude   = c_gps.longitude;
    mps_data.GPS_altitude    = c_gps.altitude;
    MPS_pub.publish(mps_data);

  }
}

void mpsEngDataCallback(const ros::TimerEvent& event){
  if (mps_init){

    flam_engdata_t engData;
    if(run_MPS(0x99, 0, &engData) != 0){
      ROS_ERROR("MPS reading error!");
      exit(-2);
    }    

    //printf("engData: %s, %d, %f, %f, %f, %f, %f, %f, %f, %f, %d, %f, %f, %f, %f, %f, %f, %f, %f, %f, %f, %f\n",
    //    timestamp,
    //      SWAP32(engData.answer.cycleCount), engData.answer.temp, engData.answer.pressure, engData.answer.relHumidity,
    //      engData.answer.absHumidity, 0.0, 0.0, engData.dDSC1MidCompNorm, engData.dDSC1MaxCompNorm,
    //      SWAP32(engData.answer.flamID), engData.answer.concentration, engData.midKelvinPower, engData.maxKelvinPower,
    //      engData.midTotalResistance, engData.maxTotalResistance,
    //	      engData.midTotalPower, engData.maxTotalPower,
    //	      engData.ambientResistance_t, engData.ambientResistance_k,
    //	      engData.midKelvinResistance, engData.maxKelvinResistance);
     //printf("%s, %d, %f, %f, %f, %f, %f, %f\n",
     //      timestamp,
     //      SWAP32(engData.answer.cycleCount), engData.answer.temp, engData.answer.pressure, engData.answer.relHumidity,
     //      engData.answer.absHumidity, 0.0, 0.0);

    //printf("Eng\n");
    ///*
    mps_driver::MPS mps_data;
    //mps_data.gasID           = engData.flamID;
    mps_data.percentLEL      = engData.answer.concentration;
    mps_data.temperature     = engData.dDSC1MidCompNorm;
    mps_data.pressure        = engData.dDSC1MaxCompNorm;
    mps_data.humidity        = engData.midKelvinPower;
    mps_data.absHumidity     = engData.maxKelvinPower;
    mps_data.humidAirDensity = engData.midTotalPower;//0;
    if (platformSim){
      mps_data.local_x         = robot_pose.pose.position.x;  
      mps_data.local_y         = robot_pose.pose.position.y;
      mps_data.local_z         = robot_pose.pose.position.z;

    }
    else{
      mps_data.local_x         = engData.maxTotalPower;//c_local_position.pose.pose.position.x;   
      mps_data.local_y         = c_local_position.pose.pose.position.y;
      mps_data.local_z         = c_local_position.pose.pose.position.z;

    }

    mps_data.GPS_latitude    = c_gps.latitude;
    mps_data.GPS_longitude   = c_gps.longitude;
    mps_data.GPS_altitude    = c_gps.altitude;
    MPS_pub.publish(mps_data); 
    //*/

  }
}

void pose_cb(const geometry_msgs::PoseStamped::ConstPtr& robot_pose_tmp){
  robot_pose = *robot_pose_tmp;
  pose_cb_flag = false;
  //  printf("x:%f,y:%f,z:%f",robot_pose.pose.position.x,robot_pose.pose.position.y,robot_pose.pose.position.z);
}

int main(int argc, char *argv[]){

  ros::init(argc, argv, "MPS_Driver");
  ros::NodeHandle nh;
    
  // Publish MPS message
  MPS_pub = nh.advertise<mps_driver::MPS>("mps_data", 1);
  ros::Subscriber subscribe_GPS = nh.subscribe("/mavros/global_position/global",1,gps_callback);
  ros::Subscriber subscribe_local_position = nh.subscribe("/mavros/global_position/local",1,local_position_callback);

  // Local variable
  int ii, freq = 2, portNumber = 0, count = 0, RobotID=0;
  string device_s = "/dev/ttySAC";
  
  
  nh.getParam("frequency", freq);
  nh.getParam("device", device_s);
  nh.getParam("port", portNumber);
  nh.getParam("Robot_ID", RobotID);
  nh.getParam("platformSim", platformSim);
  //ros::Timer mpsData_timer_     = nh.createTimer(ros::Duration(1.0/freq), &mpsDataCallback);
  ros::Timer mpsEngData_timer_  = nh.createTimer(ros::Duration(1.0/freq), &mpsEngDataCallback);

  //ros::Rate loop_rate(freq);

  // Opening port and get file handler 
  if ((uartFP = openSerialPort((char *)device_s.c_str(), portNumber)) == -1) {
    printf("Failed to open %s%d: %s (%d)\n", device_s, portNumber, strerror(errno), errno);
    exit(1);
  }else{
    ROS_INFO("MPS openned successfully.");
    //mps_init=true;
  }

  answer_t result;

  
  // Initialize MPS
  if(run_MPS(0x61, 2, &result) != 0){
    ROS_ERROR("Initializing MPS reading failed.");
    exit(-1);
  }else{
    ROS_INFO("Initializing MPS successfully.");
    sleep(3);
    mps_init=true;
  }
   
  //Get Position Data for MPS
  std::string firstString="/mocap_node/Robot_";
  std::string secondString="/pose";
  std::string str = std::to_string(RobotID);
  std::string fullString = firstString + str + secondString;
  cout<<RobotID<<endl;
  ros::Subscriber controller_sub = nh.subscribe<geometry_msgs::PoseStamped>(fullString,1,pose_cb);
  
  ROS_INFO("getting version");
  uint32_t temp;
  uint8_t data;
  temp=ReadVersion(CMD_VERSION, &data ,8);

  /*
  // Main loop
  while(ros::ok()){
  // send command packet 
    if(run_MPS(0x01, 0, &result) != 0){
      ROS_ERROR("MPS reading error!");
      exit(-2);
    }

#ifdef DEBUG
    printf("Analyte ID, Concentration [%%LEL], Temp [C], Pressure [kPa], Humidity [%%RH], Relative Humidity [%%H]\n");
    printf("%s, %f, %f, %f, %f, %f\n", result->flamID, result->concentration, result->temp, result->pressure, result->relHumidity, result->absHumidity);
#endif
    
    mps_driver::MPS mps_data;
    mps_data.gasID           = result.flamID;
    mps_data.percentLEL      = result.concentration;
    mps_data.temperature     = result.temp;
    mps_data.pressure        = result.pressure;
    mps_data.humidity        = result.relHumidity;
    mps_data.absHumidity     = result.absHumidity;
    mps_data.humidAirDensity = 0;
    mps_data.local_x         = c_local_position.pose.pose.position.x;
    mps_data.local_y         = c_local_position.pose.pose.position.y;
    mps_data.local_z         = c_local_position.pose.pose.position.z;
    mps_data.GPS_latitude    = c_gps.latitude;
    mps_data.GPS_longitude   = c_gps.longitude;
    mps_data.GPS_altitude    = c_gps.altitude;
    MPS_pub.publish(mps_data);

    ros::spinOnce();
    loop_rate.sleep();
    ++count;
  }
  */
  while(ros::ok()){
    ros::spinOnce();
    while (pose_cb_flag) {
      ros::spinOnce();
      if (!pose_cb_flag) {
	break;
      }
    }
  }
  
  ros::spin();  
  closeSerialPort(uartFP);
}
