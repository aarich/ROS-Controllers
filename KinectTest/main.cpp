#include "ros/ros.h"
#include "std_msgs/String.h"
#include "cv_bridge/cv_bridge.h"
#include "sensor_msgs/Image.h"
#include "sensor_msgs/image_encodings.h"
#include "ros/ros.h"
#include <opencv2/opencv.hpp>
#include "sensor_msgs/Image.h"
#include <cv_bridge/cv_bridge.h>
#include <image_transport/image_transport.h>
#include <sensor_msgs/image_encodings.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/nonfree/nonfree.hpp>
#include "opencv2/core/core.hpp"
#include "opencv2/features2d/features2d.hpp"
#include <boost/algorithm/string.hpp>

#include "boost/filesystem.hpp"

#include <ros/ros.h>
//Use image_transport for publishing and subscribing to images in ROS
#include <image_transport/image_transport.h>
//Use cv_bridge to convert between ROS and OpenCV Image formats
#include <cv_bridge/cv_bridge.h>
//Include some useful constants for image encoding. Refer to: http://www.ros.org/doc/api/sensor_msgs/html/namespacesensor__msgs_1_1image__encodings.html for more info.
#include <sensor_msgs/image_encodings.h>
//Include headers for OpenCV Image processing
#include <opencv2/imgproc/imgproc.hpp>
//Include headers for OpenCV GUI handling
#include <opencv2/highgui/highgui.hpp>

#include "../src/Helpers/Perspective.h"


#include <sstream>
#include <stdio.h>
#include <iostream>
#include <cmath>
#include <ctime>

using namespace std;
using namespace cv;
using namespace ros;

ros::Publisher  movement_publisher; 
image_transport::Publisher  data_publisher; 
ros::Subscriber mcl_data_subscriber;
ros::Subscriber ksub;

const std::string publish_image_data_under = "ROBOT_IMAGE_PUBLISHER";
const std::string mcl_data_publisher_name = "MCL_DATA_PUBLISHER";

const int handshake = 15;
const int readymove = 25;
const int starting_move = 10;
const int finished_move = 20;

bool handshake_recieved = false;

float round(float x);

void imageCallback (const sensor_msgs::ImageConstPtr& img)
{
	cv_bridge::CvImagePtr im2;
	try 
	{
		im2 = cv_bridge::toCvCopy(img, sensor_msgs::image_encodings::BGR8); //enc::RGB8 also used
	}
	catch (cv_bridge::Exception& e) 
	{
		ROS_ERROR("cv_bridge exception: %s", e.what());
		return;
	}

	imshow("Robot Image", im2->image);
	waitKey(2);
	data_publisher.publish(img);
}

void publish_Move()
{
	stringstream ss;
	std_msgs::String msg;

	msg.data = "10";
	movement_publisher.publish(msg);

	float translate;
	int turn;
	cout << "Translate: ";
	cin >> translate;
	cin.get();
	cout << endl;
	cout << "Turn: ";
	cin >> turn;
	cin.get();
	cout << endl;
	ss << "20" << " " << translate << " " << turn << " ";
	msg.data = ss.str();
	movement_publisher.publish(msg);
}

void MyDataCallback(const std_msgs::String msg)
{
	string str = msg.data;

	if(atoi(str.c_str())== handshake)
	{
		handshake_recieved = true;
	}
	else if(atoi(str.c_str()) == readymove)
	{
		std::cout << "Movement Command Recieved, starting move" << std::endl;
		publish_Move();
	}
}

int main(int argc, char **argv)
{
	ros::init(argc ,argv, "ROS_Publisher");
	NodeHandle node;
	namedWindow("Robot Image");

	image_transport::ImageTransport it(node);

	mcl_data_subscriber = node.subscribe(mcl_data_publisher_name, 4, MyDataCallback);

	time_t temptime = time(0);
	std::cout << "Waiting for Handshake from Program .." << std::endl;
	while(!handshake_recieved && (time(0) - temptime) < 20)
	{
		ros::Duration(0.05).sleep();
		ros::spinOnce();
	}
	if(handshake_recieved)
		std::cout << "Handshake recieved" << std::endl;
	else
	{
		std::cout << "No handshake recieved";
		return -1;
	}
	
	movement_publisher = node.advertise<std_msgs::String>("ROBOT_MOVEMENT_PUBLISHER", 4);
	data_publisher = it.advertise(publish_image_data_under, 4, true);
	ksub = node.subscribe("/camera/rgb/image_color", 10, imageCallback);
	spin();
}

float round(float x)
{
	return (float) ((int) (x*100))/100.0;
}