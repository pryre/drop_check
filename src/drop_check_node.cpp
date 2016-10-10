#include "ros/ros.h"
#include "geometry_msgs/TransformStamped.h"

#define TIMEOUT 0.2
#define AV_MSG 100

int msg_count = 0;
double running_dt = 0.0;
ros::Time last_msg;

void chatterCallback( const geometry_msgs::TransformStamped::ConstPtr& msg ) {
	if( msg_count >= AV_MSG) {
		ROS_INFO( "Average: %0.2f", running_dt/msg_count );

		msg_count = 0;
		running_dt = 0;
	}

	double dt = ( msg->header.stamp - last_msg ).toSec();

	if( dt > TIMEOUT )
		ROS_ERROR( "There was a large gap in timestamps: +%0.2fs", dt );

	last_msg = msg->header.stamp;
	running_dt += dt;
	msg_count++;
}

int main( int argc, char **argv ) {
	ros::init( argc, argv, "drop_check" );

	ros::NodeHandle nh;
	ros::Subscriber sub = nh.subscribe( "/vicon/cicada/cicada", 1000, chatterCallback );

	ros::spin();

	return 0;
}
