#include "../../dynamixel-workbench/dynamixel_workbench_toolbox/include/dynamixel_workbench_toolbox/dynamixel_workbench.h"
#include <ros/ros.h>
#include <ros/console.h>
#include <geometry_msgs/Twist.h>
#include <math.h> //  for atan2 and sqrt

bool velcmd_cb_flag = false;
geometry_msgs::Twist velcmd;

void velcmd_cb(const geometry_msgs::Twist::ConstPtr& velcmdMsg){
  velcmd = *velcmdMsg;
  velcmd_cb_flag = true;
}

int main(int argc, char *argv[])
{
  ros::init(argc, argv, "velocityControlDynamixel");
  ros::NodeHandle nh;

  ros::Subscriber velcmd_sub = nh.subscribe<geometry_msgs::Twist>("totalVelocityCMD",100, velcmd_cb);

  ros::Rate rate(50.0);

  float desiredXVel = 0;
  float desiredYVel = 0;

  const char* port_name = "/dev/dyna";
  int baud_rate = 1000000;

  int dxl_idM1 = 1; // default to motorID 1
  int dxl_idM2 = 2; // default to motorID 1
  int dxl_idM3 = 3; // default to motorID 1
  int dxl_idM4 = 4; // default to motorID 1

  DynamixelWorkbench dxl_wbM1;
  DynamixelWorkbench dxl_wbM2;
  DynamixelWorkbench dxl_wbM3;
  DynamixelWorkbench dxl_wbM4;

  const char *log;
  bool result = false;

  uint16_t model_number = 0;

  result = dxl_wbM1.init(port_name, baud_rate, &log);
  if (result == false)
  {
    printf("%s\n", log);
    printf("Failed to init M1\n");

    return 0;
  }
  else{
    printf("Succeed to init M1 (%d)\n", baud_rate);
  }

  result = dxl_wbM2.init(port_name, baud_rate, &log);
  if (result == false)
  {
    printf("%s\n", log);
    printf("Failed to init M2\n");

    return 0;
  }
  else{
    printf("Succeed to init M2 (%d)\n", baud_rate);
  }

  result = dxl_wbM3.init(port_name, baud_rate, &log);
  if (result == false)
  {
    printf("%s\n", log);
    printf("Failed to init M3\n");

    return 0;
  }
  else{
    printf("Succeed to init M3 (%d)\n", baud_rate);
  }

  result = dxl_wbM4.init(port_name, baud_rate, &log);
  if (result == false)
  {
    printf("%s\n", log);
    printf("Failed to init M4\n");

    return 0;
  }
  else{
    printf("Succeed to init M4 (%d)\n", baud_rate);
  }


  result = dxl_wbM1.ping(dxl_idM1, &model_number, &log);
  if (result == false)
  {
    printf("%s\n", log);
    printf("Failed to ping M1\n");
  }
  else
  {
    printf("Succeed to ping M1\n");
    printf("id : %d, model_number : %d\n", dxl_idM1, model_number);
  }

  result = dxl_wbM2.ping(dxl_idM2, &model_number, &log);
  if (result == false)
  {
    printf("%s\n", log);
    printf("Failed to ping M2\n");
  }
  else
  {
    printf("Succeed to ping M2\n");
    printf("id : %d, model_number : %d\n", dxl_idM2, model_number);
  }

  result = dxl_wbM3.ping(dxl_idM3, &model_number, &log);
  if (result == false)
  {
    printf("%s\n", log);
    printf("Failed to ping M3\n");
  }
  else
  {
    printf("Succeed to ping M3\n");
    printf("id : %d, model_number : %d\n", dxl_idM3, model_number);
  }

  result = dxl_wbM4.ping(dxl_idM4, &model_number, &log);
  if (result == false)
  {
    printf("%s\n", log);
    printf("Failed to ping M4\n");
  }
  else
  {
    printf("Succeed to ping M4\n");
    printf("id : %d, model_number : %d\n", dxl_idM4, model_number);
  }


  result = dxl_wbM1.wheelMode(dxl_idM1, 0, &log) && dxl_wbM2.wheelMode(dxl_idM2, 0, &log) && dxl_wbM3.wheelMode(dxl_idM3, 0, &log) && dxl_wbM4.wheelMode(dxl_idM4, 0, &log);
  if (result == false)
  {
    printf("%s\n", log);
    printf("Failed to change wheel mode\n");
  }
  else
  {
    printf("Succeed to change wheel mode\n");
    printf("Dynamixel is moving...\n");

    while(velcmd_cb_flag == false && ros::ok()){
      ros::spinOnce();
      rate.sleep();
      if (velcmd_cb_flag){
        break;
      }
    }

    while(ros::ok()){

      float theta = atan2(velcmd.linear.y,velcmd.linear.x); // radians
      float velocity = sqrt(pow(velcmd.linear.y,2) + pow(velcmd.linear.x,2)); // m/s
      float angularVelocity = velcmd.angular.z;


      // From top down
      // M1 is the front of the robot
      // M2 is the rigth of the robot
      // M3 is the back of the robot
      // M4 is the left of the robot

      //      Front
      //       M1 +y
      // -x M4    M2 +x
      //    -y M3
      //      Back

      float M1Vel = velocity  * -sin(theta - M_PI/2) - angularVelocity;
      float M2Vel = velocity  * -cos(theta - M_PI/2) - angularVelocity;
      float M3Vel = velocity  *  sin(theta - M_PI/2) - angularVelocity;
      float M4Vel = velocity  *  cos(theta - M_PI/2) - angularVelocity;
      if (M1Vel >460) {
	M1Vel=460;
	  }
      else if (M1Vel<-460){
	M1Vel = -460;
      }

      if (M2Vel >460) {
        M2Vel=460;
          }
      else if (M2Vel<-460){
        M2Vel =	-460;
      }
      if (M3Vel >460) {
        M3Vel=460;
          }
      else if (M3Vel<-460){
        M3Vel =	-460;
      }
      if (M4Vel >460) {
        M4Vel=460;
          }
      else if (M4Vel<-460){
        M4Vel =	-460;
      }


      // Takes value in from -100 to 100
      dxl_wbM1.goalVelocity(dxl_idM1, (int32_t)M1Vel);
      dxl_wbM2.goalVelocity(dxl_idM2, (int32_t)M2Vel);
      dxl_wbM3.goalVelocity(dxl_idM3, (int32_t)M3Vel);
      dxl_wbM4.goalVelocity(dxl_idM4, (int32_t)M4Vel);

      ros::spinOnce();
      rate.sleep();
    }

    dxl_wbM1.goalVelocity(dxl_idM1, (int32_t)0);
    dxl_wbM2.goalVelocity(dxl_idM2, (int32_t)0);
    dxl_wbM3.goalVelocity(dxl_idM3, (int32_t)0);
    dxl_wbM4.goalVelocity(dxl_idM4, (int32_t)0);

    // for (int count = 0; count < 3; count++)
    // {
    //   dxl_wbM1.goalVelocity(dxl_idM1, (int32_t)-100);
    //   dxl_wbM2.goalVelocity(dxl_idM2, (int32_t)-100);
    //   sleep(3);
    //
    //   dxl_wbM1.goalVelocity(dxl_idM1, (int32_t)100);
    //   dxl_wbM2.goalVelocity(dxl_idM2, (int32_t)100);
    //   sleep(3);
    // }
    //
    // dxl_wbM1.goalVelocity(dxl_idM1, (int32_t)0);
    // dxl_wbM2.goalVelocity(dxl_idM2, (int32_t)0);
  }

  return 0;
}
