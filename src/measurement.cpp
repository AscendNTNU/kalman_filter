#include <ros/ros.h>
#include <math.h>
#include <geometry_msgs/PoseStamped.h>
#include <random>


int main(int argc, char **argv){
	ros::init(argc, argv, "user_setpoint_repeater");
	ros::NodeHandle n;

	ros::Publisher pub = n.advertise<geometry_msgs::PoseStamped>("/measurement/pose", 100);

	ros::Rate loop_rate(30);
	geometry_msgs::PoseStamped measurement;

	std::default_random_engine generator;
    std::normal_distribution<double> dist(0.0, 0.1);

	while(ros::ok()){
		measurement.header.stamp=ros::Time::now(); 
		measurement.pose.position.x = cos(ros::Time::now().toSec()) + dist(generator);
		pub.publish(measurement); 
		loop_rate.sleep();
	}
	return 0;
}