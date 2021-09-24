/*********************************************************************************************************
*                                       Copyright Notice
*
*********************************************************************************************************/

/*********************************************************************************************************
*                                  COMMAND & RESPONSE OPERATIONS
* Filename      : MOX_driver.cpp
* Version       : V1.0.1
* Programmers(s): Xiang He kyle joey 

**********************************************************************************************************
* Notes         : (1) n/a
*/
/* Includes ---------------------------------------------------------------------------------------------*/
#include <commandResponse.h>
#include "sensor_msgs/NavSatFix.h"
//#include <ros/ros.h>
#define PORT_ADC1   1 
//#define DEBUG


/* Variables --------------------------------------------------------------------------------------------*/

/* Callback functions --------------------------------------------------------------------------------------------*/

sensor_msgs::NavSatFix c_gps;

//#define DEBUG

/* Functions --------------------------------------------------------------------------------------------*/
void gps_callback(const sensor_msgs::NavSatFix &new_message)
{
    c_gps = new_message;
}


int main(int argc, char *argv[]){
  
  ros::init(argc, argv, "MOX_Driver");
  ros::NodeHandle nh;
  double a, b;
  
  nh.getParam("a_exp", a);
  nh.getParam("b_exp", b);

  ros::Subscriber subscribe_GPS = nh.subscribe("/mavros/global_position/global",1,gps_callback);

  // Publish MPS message
  ros::Publisher mox_pub = nh.advertise<mps_driver::MPS>("mox_data", 1);
  ros::Rate loop_rate(50);
  int count = 0;
  
  float sensor_volt;
  float mox_lel;
  float RS_air; // Get value of RS in clean air
  float R0; // Get value of R0 in H2  
  float sensorValue;

  ros::Time starttime=ros::Time::now();

  //wiringPiSetup();
  FILE* fd = NULL;
  // mox driver code

  while(ros::ok()){
    
    //sensorValue = analogRead(PORT_ADC1);
    fd = fopen("/sys/bus/iio/devices/iio\:device0/in_voltage3_raw", "rb");
    char buf[10] = {0};
    fread(buf, sizeof(char), 10, fd);
    fclose(fd);
    sensorValue = atof(buf);
      
    /*--- Get a average data by testing 100 times ---*/
    /*for(int x = 0 ; x < 100 ; x++)
      {
      sensorValue = sensorValue + analogRead(PORT_ADC1);
      }*/
    //sensorValue = sensorValue/100.0;
    /*-----------------------------------------------*/
      
    sensor_volt = sensorValue/4096*1.8;
    //mox_lel = 51.27*powr((sensor_volt),5.446);
    //mox_lel = 51.59*pow((sensor_volt),5.372) - 0.3466;
    //mox_lel = 0.1362*exp(5.5874*sensor_volt);//0.1051*exp(6.199*sensor_volt); //keeper for MOX
    mox_lel = a*exp(b*sensor_volt);

	//ros::Time start_time(0.0001)	
	ros::Time time=ros::Time::now();//.toSec();
	ros::Duration deltatime = time - starttime;
	double timesec = deltatime.toSec();
	//cout << "ROS Delta time: "<< deltatime << endl;
	//cout << "Time sec "<< timesec << endl;
    //mox_lel= 62.5*(sensor_volt-0.43); // Attempt for NDIR
    //RS_air = (1.-sensor_volt)/sensor_volt; // omit *RL
    // The ratio of RS/R0 is 9.8 in a clear air from Graph (Found using WebPlotDigitizer)
    //R0 = RS_air/9.8; 
    
    #ifdef DEBUG
    printf("sensor_volt = %f\n", sensor_volt);
    #endif
    
    mps_driver::MPS mox_data;
    mox_data.gasID           = "MOX";
    mox_data.percentLEL      = mox_lel;
    mox_data.temperature     = 0;
    mox_data.pressure        = 0;
    mox_data.humidity        = 0;
    mox_data.absHumidity     = sensor_volt;
    mox_data.humidAirDensity = 0;
    mox_data.GPS_latitude    = c_gps.latitude;
    mox_data.GPS_longitude   = c_gps.longitude;
    mox_data.GPS_altitude    = c_gps.altitude;


    mox_pub.publish(mox_data);
      
    ros::spinOnce();
    loop_rate.sleep();
    ++count;
  }
}
