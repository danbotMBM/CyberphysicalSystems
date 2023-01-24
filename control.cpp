// ~/catkin_ws/src/race/src/control.cpp
#include <ros/ros.h>
#include <race/drive_param.h>
#include <race/pid_input.h>
#include <ros/callback_queue.h>

#include <ctime>
#include <chrono>
#include <thread>

#include <string>
#include <vector>
#include <iostream>
#include <iterator>
#include <iomanip>
#include <sstream>
using namespace std;

static double kp = 250.0;
static double kd = 0.0;
static double servo_offset = 18.5;
static double prev_error = 0.0;
static double vel_input = 25.0;
static double start_time = 0.0;
static int choice = 0;	// choose spin mode

static auto end_time = std::chrono::high_resolution_clock::now();


void callback(const boost::shared_ptr<const race::pid_input> &data, ros::Publisher &pub){

	// TEST: show time elapsed between callbacks
	// auto new_start = std::chrono::high_resolution_clock::now();
	// std::chrono::duration<double> elapsed = new_start - end_time;
	// cout << "Elapsed time between callback: " << (double)elapsed.count() << "s"<<endl;
	double angle = data.get()->pid_error*kp;
	race::drive_param msg;

	string command;
	cout << "Enter command: ";
	//TODO parse the string
	string temp;
	char command_type;
	cin >> temp;
	command_type = temp.at(0);

	double value;
	cin >> temp;
	value = stod(temp);
	//TODO HAVE SAFETY MEASURES for weird inputs
	switch (command_type){
		case 's':
			vel_input = value;
			cout << "Velocity set to " << vel_input << endl;
		break;
		case 't':
			angle = value;
			cout << "Angle set to " << angle << endl;
		break;
		case 'c':
			cout << "c case";
		break;
		default:
			vel_input = 0;
			cout << "default case";
		break;
	}


	

	if (angle > 100){
		angle = 100;
	}	
	if (angle < -100){
		angle = -100;
	}
	
	if(data.get()->pid_vel == 0){
		msg.velocity = -8;
	}
	else{
		msg.velocity = vel_input;
	}	
	msg.angle = angle;
	cout << "Velocity: " << msg.velocity << " Angle: " << msg.angle << endl;

	pub.publish(msg);

	// TEST:
	//end_time = std::chrono::high_resolution_clock::now();
}


int main(int argc, char **argv){
	int pub_queue_size = 1;
	int sub_queue_size = 1;
	//int choice = 0;
	int given_time = 100;
	int hz = 10;
	string command;
	string command_s1;
	string command_s2;
	
	// Get user input
	cout << "Listening to error for PID" << "\n";
	cout << "Enter Velocity: ";
	cin >> vel_input;
	/*
	// Chose different types of spin
	cout << "Choose spinning method(0--Robust Spin, 1--Dynamic Spin, 2--Static Spin): ";
	cin >> choice;
	if(choice == 1){
		cout << "Enter Execution Time in milliseconds: ";
		cin >> given_time;
	}
	if(choice == 2){
		cout << "Enter Frequency in HZ: ";
		cin >> hz;
	}
	*/

	// Initiate Node
	ros::init(argc, argv, "pid_controller");
	// Create Node Handeler for publisher
	ros::NodeHandle p;
	ros::Publisher pub = p.advertise<race::drive_param>("drive_parameters", pub_queue_size);	
	// Create Node Handeler for subscriber
	ros::NodeHandle s;
	ros::Subscriber sub = s.subscribe<race::pid_input>("error", sub_queue_size, boost::bind(callback, _1, pub));


	// Robust Spin
	if(choice == 0){
		ros::spin();
	}

	// Dynamic Spin
	else if(choice == 1){
		while(ros::ok()){
			// record the time in millisecond that used for the callback function
			auto start = std::chrono::high_resolution_clock::now();

			ros::getGlobalCallbackQueue()->callAvailable(ros::WallDuration(0.01));

			auto end = std::chrono::high_resolution_clock::now();
			std::chrono::duration<double> elapsed = end - start;
			int sleep_time = (int)(given_time - (double)elapsed.count()*1000);

			// Dynamically sleep
			std::this_thread::sleep_for(std::chrono::milliseconds(sleep_time));
		}
	}

	// Static Spin
	else if(choice == 2){
		ros::Rate r(hz);
		while (ros::ok()){
			ros::spinOnce();
			r.sleep();
		}
	}
	
	

	return 0;
}
