#include <ros/ros.h>
#include <rosgraph_msgs/Clock.h>
#include "boost/date_time/posix_time/posix_time.hpp"
#include <dynamic_reconfigure/server.h>
#include <cam_trigger/cam_triggerConfig.h>
#include <unistd.h>
#include <pigpiod_if2.h>

//#include <time.h>
//#include <signal.h>

//-----------Ros Stuff----------------------------
//Publisher
ros::Publisher clock_trigger_pub;
//Clock Message
rosgraph_msgs::Clock trigger_time;
//Ros Timer for Interrupt
ros::Timer timer;


//-----------Variable Definition------------------
//Raspberry Pi ID
int pi;
//Set Timer Freuqency
double freq = 30;
//Set Impulse width in %
double width = 5;
//Impulse width in s
ros::Duration width_d(0.0);
//Enabled/Disabled Cameras
bool cam[4] = {true, true, true, true};
//System Disable/Enable
bool system_stat = false;


//-----------GPIO Definition-----------------------
//GPIO 26 --> Pin 37
int output_pin_1 = 26;
//GPIO 19 --> Pin 35
int output_pin_2 = 19;
//GPIO 13 --> Pin 33
int output_pin_3 = 13;
//GPIO 6 --> Pin 31
int output_pin_4 = 6;

//Bitmask for GPIO write
uint32_t BitMask = (1 << output_pin_1) | (1 << output_pin_2) | (1 << output_pin_3) | (1 << output_pin_4);


//-----------Timer Interrupt Routine---------------
void alarmWakeup(const ros::TimerEvent&) {
	//Set Bitmask to set the GPIOs to high
	set_bank_1(pi, BitMask);
	
	//Publish the trigger time
	const ros::Time t1 = ros::Time::now();
	trigger_time.clock = t1;
	clock_trigger_pub.publish(trigger_time);
	
	//Wait to generate a pulsewidth (substract the time to publish the clock message)
	const ros::Time t2 = ros::Time::now();
	const ros::Duration publish_d(t2-t1);
	if (publish_d >= width_d) {
		ROS_ERROR("Pulsewidth cannot be accomplished: publish_d >= width_d");
	} else {
		ros::Duration(width_d - publish_d).sleep();
	}
	
	//Set Bitmask to set the GPIOs to low
	clear_bank_1(pi, BitMask);
}


//-----------Reconfigure Callback-------------------
void callback(cam_trigger::cam_triggerConfig &config, uint32_t level) {

	//cancel the actual alarm and gpios
	timer.stop();
	clear_bank_1(pi, UINT32_MAX);
	
	//Display new Parameter
	ROS_INFO("Parameter: Frequenz: %f Hz  -  Pulsweite: %f %%  -  System AN: %s  - Cam1 AN: %s  - Cam2 AN: %s  - Cam3 AN: %s  - Cam4 AN: %s", 
			config.Frequenz, config.Pulsweite, 
			config.System?"True":"False", 
			config.cam1?"True":"False", 
			config.cam2?"True":"False", 
			config.cam3?"True":"False",
			config.cam3?"True":"False");
	
	//set new parameters
	freq = config.Frequenz;
	width = config.Pulsweite;
	system_stat = config.System;
	cam[0] = config.cam1;
	cam[1] = config.cam2;
	cam[2] = config.cam3;
	cam[3] = config.cam4;
	
	//Set the new Bitmask for gpio trigger
	BitMask = (cam[0] << output_pin_1) | (cam[1] << output_pin_2) | (cam[2] << output_pin_3) | (cam[3] << output_pin_4);
	
	//Start alarm new
	if(system_stat == true) {
		double timer_periode = 1.0 / freq;
		width_d = ros::Duration(timer_periode * (width/100.0));
		ROS_INFO("Timer Periode: %f s -  Pulswidth of signal: %f s\n", timer_periode, width_d.toSec());
		
		//Set new Timer
		timer.setPeriod(ros::Duration(timer_periode));
		timer.start();
	}
}

//-----------MAIN----------------------------------
int main(int argc, char **argv) {
	//Init ROS
	ros::init(argc, argv, "cam_trigger");
	
	//Init Node
	ros::NodeHandle Cam_Trigger;
	
	//Init Clock Publisher
	clock_trigger_pub = Cam_Trigger.advertise<rosgraph_msgs::Clock>("clock_trigger_pub", 1000);
	
	//Init dynamic reconfigure
	dynamic_reconfigure::Server<cam_trigger::cam_triggerConfig> server;
	dynamic_reconfigure::Server<cam_trigger::cam_triggerConfig>::CallbackType f;
	f = boost::bind(&callback, _1, _2);
	server.setCallback(f);
	
	//Init Timer with 30Hz
	timer = Cam_Trigger.createTimer(ros::Duration(1/30), alarmWakeup); 
	timer.stop();

	//Initialize GPIOs
	int pi = pigpio_start(NULL, "8888");
	ROS_INFO("PI Init: %d\n", pi);

	if(pi==0) {
	  ROS_INFO("PI Initialisation succesfull\n");
	} else {
	  ROS_INFO("PI Initialisation failed\nTry to start GPIOs with following command: sudo pigpiod\n\n");
	  return 1;
	}

	//Set Mode of the GPIOs
	set_mode(pi,output_pin_1, PI_OUTPUT);
	set_mode(pi,output_pin_2, PI_OUTPUT);
	set_mode(pi,output_pin_3, PI_OUTPUT);
	set_mode(pi,output_pin_4, PI_OUTPUT);
	//Set all GPIOs to Zero
	clear_bank_1(pi, UINT32_MAX);

	//Start Trigger
	ROS_INFO("Start Trigger");
	ros::spin();
	return 0;
}
