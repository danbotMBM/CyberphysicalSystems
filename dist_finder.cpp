// ~/catkin_ws/src/race/src/dist_finder.cpp
#include <ros/ros.h>
#include <sensor_msgs/LaserScan.h>
#include <race/pid_input.h>
#include <cmath>
#include <iostream>
#include <ros/callback_queue.h>

#include <ctime>
#include <chrono>
#include <thread>
using namespace std;

static int choice = 0;	// choose spin mode

static int vel = 30;

// 20.5 30 12
// 20.3 28 15.2
// extarct dsiatnce data from LaserScan data set
static double getRange(const sensor_msgs::LaserScan *data, int theta){
	double distance = data->ranges[theta];
/*
	if(!distance || distance > 1.5){
		distance = 50;
	}
*/
	if(!distance){
		cout << "distance";
	}


	return distance;
}

// helper function: transfer degree to radiant
static double getRadians(double degree){
	const double PI = 3.1415926;
	return PI * degree / 180;
}

double p_distance = 0;
static void callBack(const boost::shared_ptr<const sensor_msgs::LaserScan> &data, ros::Publisher &pub){
	double total = 0.0;
	double error = 0.0;
	double vel = 0.0;
	double c_distance;

	const sensor_msgs::LaserScan *raw_data = data.get();
	for(int i = 180; i < 500; i += 10){
		total += (getRange(raw_data,i)* cos(getRadians((i-180)/4))-.75) / getRange(raw_data,i) * sin(getRadians((i-180.0)/4));
	}
	error = total/36.0;

	c_distance = getRange(raw_data,540);

	p_distance = c_distance;

    cout << p_distance << endl;

	if(getRange(raw_data,540) < 0.75){
		vel = 0;
	}
	else{
		vel = 8;
	}

	
	race::pid_input msg;
	msg.pid_error = error;
	msg.pid_vel = vel;
	pub.publish(msg);
}

int main(int argc, char **argv){
	int pub_queue_size = 1;
	int sub_queue_size = 1;
	//int choice = 0;
	int given_time = 100;
	int hz = 10;
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

	// Initiate the Node
	cout<< "Laser node started" << endl; 
	ros::init(argc, argv, "dist_finder");
	ros::NodeHandle p;
	ros::Publisher pub = p.advertise<race::pid_input>("error", pub_queue_size);

	ros::NodeHandle s;
	ros::Subscriber sub = s.subscribe<sensor_msgs::LaserScan>("scan", sub_queue_size, boost::bind(callBack, _1, pub));

	
	// Robust Spin
	if(choice == 0){
		ros::spin();
	}

	// Dynamic Spin
	else if(choice == 1){
		while(ros::ok()){
			// record the time in millisecond that used for the callback function
			auto start = std::chrono::high_resolution_clock::now();

			//ros::getGlobalCallbackQueue()->callAvailable(ros::WallDuration(0));
			ros::getGlobalCallbackQueue()->callOne();

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