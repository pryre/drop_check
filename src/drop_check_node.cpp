#include "ros/ros.h"
#include "geometry_msgs/TransformStamped.h"

#define TIMEOUT 0.2
#define AV_MSG 100

int msg_count = 0;
double running_dt = 0.0;
double running_lat = 0.0;
ros::Time last_msg;

void msg_cb( const geometry_msgs::TransformStamped::ConstPtr& msg ) {
	//Average out the last X amount of messages
	if( msg_count >= AV_MSG) {
		ROS_INFO( "Average delta: %0.2f", running_dt/msg_count );
		ROS_INFO( "Average latency: %0.2f", running_lat/msg_count );

		msg_count = 0;
		running_dt = 0;
	}

	//Get the difference in time from the last message
	double dt = ( msg->header.stamp - last_msg ).toSec();
	double lat = ( ros::Time::now() - msg->header.stamp ).toSec();


	//Show a warning if necessary
	if( dt > TIMEOUT )
		ROS_ERROR( "There was a large gap in timestamps: +%0.2fs", dt );

	if( lat > TIMEOUT )
		ROS_ERROR( "There was a large latency for this message: +%0.2fs", lat );

	//Save the messages till next loop
	last_msg = msg->header.stamp;

	//Update counters
	running_dt += dt;
	running_lat += lat;
	msg_count++;
}

int main( int argc, char **argv ) {
	ros::init( argc, argv, "drop_check" );

	ros::NodeHandle nh;
	ros::Subscriber sub = nh.subscribe( "/vicon/cicada/cicada", 1000, msg_cb );

	ros::spin();

	return 0;
}
