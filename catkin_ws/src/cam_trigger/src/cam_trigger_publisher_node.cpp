#include <ros/ros.h>
#include <dynamic_reconfigure/server.h>
#include <cam_trigger/cam_triggerConfig.h>
#include <signal.h>
#include <unistd.h>
#include <pigpiod_if2.h>

//-----------Variable Definition------------------
//Raspberry Pi ID
int pi;
//Set Timer Freuqency
int freq = 30;
//Set Impulse width in %
double width = 5;
//Impulse width in us
double width_us; 

//-----------GPIO Definition-----------------------
//GPIO 26 --> Pin 37
int output_pin_1 = 26;
//GPIO 19 --> Pin 35
int output_pin_2 = 19;
//GPIO 13 --> Pin 33
int output_pin_3 = 13;
//GPIO 6 --> Pin 31
int output_pin_4 = 6;

//-----------Timer Interrupt Routine---------------
void alarmWakeup(int signum) {
	//Set Bitmask to set the GPIOs to high 
	set_bank_1(pi, (1 << output_pin_1) | (1 << output_pin_2) | (1 << output_pin_3) | (1 << output_pin_4));
	//Wait to generate a pulsewidth
	usleep(width_us);
	//Set Bitmask to set the GPIOs to low
	clear_bank_1(pi, (1 << output_pin_1) | (1 << output_pin_2) | (1 << output_pin_3) | (1 << output_pin_4));
	ROS_INFO("Timer abgelaufen");
}

//-----------Reconfigure Callback-------------------
void callback(cam_trigger::cam_triggerConfig &config, uint32_t level) {
	ROS_INFO("Reconfigure Request: %f %f %s %s %s %s %s", 
			config.Frequenz, config.Pulsweite, 
			config.System?"True":"False", 
			config.cam1?"True":"False", 
			config.cam2?"True":"False", 
			config.cam3?"True":"False",
			config.cam3?"True":"False");
}

//-----------MAIN----------------------------------
int main(int argc, char **argv) {
	ros::init(argc, argv, "cam_trigger");

	dynamic_reconfigure::Server<cam_trigger::cam_triggerConfig> server;
	dynamic_reconfigure::Server<cam_trigger::cam_triggerConfig>::CallbackType f;

	f = boost::bind(&callback, _1, _2);
	server.setCallback(f);

	//Initialize GPIO
	int pi = pigpio_start(NULL, "8888");
	printf("PI Init: %i\n", pi);

	if(pi==0) {
	  printf("PI Initialisation succesfull\n");
	} else {
	  printf("PI Initialisation failed\nTry to start GPIOs with following command: sudo pigpiod\n\n");
	  return 1;
	}

	//Set Mode of the GPIOs
	set_mode(pi,output_pin_1, PI_OUTPUT);
	set_mode(pi,output_pin_2, PI_OUTPUT);
	set_mode(pi,output_pin_3, PI_OUTPUT);
	set_mode(pi,output_pin_4, PI_OUTPUT);
	//Set all GPIOs to Zero
	clear_bank_1(pi, UINT32_MAX);

	double timer_periode = 1000000 / freq; 

	//Set pulse witdh in us
	width_us = timer_periode * (width/100);
	printf("Timer Periode: %f us -  Pulswidth of signal: %f us\n", timer_periode, width_us);

	//Set Timer/Alarm in us
	signal(SIGALRM, alarmWakeup);
	ualarm(timer_periode,timer_periode);

	ROS_INFO("Spinning node");
	ros::spin();
	return 0;
}
