#include "ros/ros.h"
#include "std_msgs/String.h"
#include "cv_bridge/cv_bridge.h"
#include "sensor_msgs/Image.h"
#include "sensor_msgs/image_encodings.h"
#include <opencv2/opencv.hpp>
#include "sensor_msgs/Image.h"
#include <image_transport/image_transport.h>
#include <sensor_msgs/image_encodings.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/nonfree/nonfree.hpp>
#include "opencv2/core/core.hpp"

#include <boost/algorithm/string.hpp>

#include <tf/tf.h>
#include <std_msgs/Empty.h>
#include <std_srvs/Empty.h>
#include <std_msgs/Float32.h>
#include <geometry_msgs/Twist.h>
#include <ardrone_autonomy/Navdata.h>
#include <ardrone_autonomy/navdata_magneto.h>

#include "../Robot/Robot.h"

#include "boost/filesystem.hpp"

#include <sstream>
#include <stdio.h>
#include <iostream>
#include <cmath>
#include <ctime>

using namespace std;
using namespace cv;
using namespace ros;

Publisher mcl_movement_publisher; 
image_transport::Publisher image_publisher; 
Subscriber mcl_data_subscriber;

Subscriber imageSub;
// Subscriber navDataSub;

Publisher landPub;
Publisher takeOffPub;
Publisher cmdVelPub;

std_msgs::Empty empty_msg;
geometry_msgs::Twist cmd_msg;

Robot robot;

const std::string publish_image_data_under = "ROBOT_IMAGE_PUBLISHER";
const std::string mcl_data_publisher_name = "MCL_DATA_PUBLISHER";

const int handshake = 15;
const int readymove = 25;
const int guessdata = 35;
const int starting_move = 10;
const int finished_move = 20;

bool handshake_recieved = false;

bool fly = false;

// Some declarations
float round(float x);
void SetCommand(float roll, float pitch, float yaw_velocity, float z_velocity);
void Command(float x, float yaw_velocity);
void Move(vector<float> v);

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

    int h = im2->image.rows;
    int w = im2->image.cols;
    cv::Rect myROI((w-h)/2, 0, h, h);

    cv::Mat croppedImage = im2->image(myROI);

    imshow("Robot Image", im2->image);
    waitKey(2);

    cv_bridge::CvImage out_msg;
    ros::Time imtime = ros::Time::now();
    out_msg.header.stamp = imtime;
    out_msg.header.frame_id = "robot_image";
    out_msg.encoding = sensor_msgs::image_encodings::BGR8;
    out_msg.image = croppedImage;
        
    if(!out_msg.image.empty())
        image_publisher.publish(out_msg.toImageMsg());
    ros::spinOnce();
    // image_publisher.publish(img);
}

void publish_Move()
{
    stringstream ss;
    std_msgs::String msg;

    msg.data = "10";
    mcl_movement_publisher.publish(msg);

    vector<float> move = robot.NextMove();
    Move(move);

    ss << "20" << " " << move[1] << " " << move[0] << " ";
    msg.data = ss.str();
    mcl_movement_publisher.publish(msg);
}

void MyDataCallback(const std_msgs::String msg)
{
    string str = msg.data;
    std::vector<std::string> strs;
    std::vector<float> vals;
    boost::split(strs, str, boost::is_any_of("_"));

    int command = atoi(strs[0].c_str());

    if (command== handshake)
        handshake_recieved = true;
    else if (command == readymove)
        publish_Move();
    else if (command == guessdata)
    {
        if(strs.size() != 7)
        {
            cout << "Invalid guess data format" << endl;
            return;
        }
        vector<float> vars;

        for(int i = 1; i < 7 && i < strs.size(); i++)
        {
            vars.push_back(atof(strs[i].c_str()));
        }
        robot.SetLocation(vars[0], vars[1], vars[2]);
        robot.SetHeading(vars[3], vars[4], vars[5]);
    }
}

#define TRANLATESPEED 0.5 // meters per second

void translate(float d)
{
    float t = d / TRANLATESPEED;
    float r = 0.5;

    if (d < 0)
    {
        t *= -1;
        r *= -1;
    }

    if (fly)
        Command(r, 0);
    Duration(t).sleep();
    Command(0, 0);
}

#define TIMETOTURNAROUND 4.0

void turn(int deg)
{
    float t = (float) deg * TIMETOTURNAROUND / 360.0;
    int r = 1;

    if (deg < 0)
    {
        t *= -1;
        r *= -1;
    }

    if (fly)
        Command(0, r);
    Duration(t).sleep();
    Command(0, 0);
}

void Move(vector<float> v)
{
    cout << "Turn: " << v[0] << ", Tranlsate: " << v[1] << endl;
    if (v[0] > 0.1)
        turn((int) v[0]);
    if (v[1] > 0.1)
        translate(v[1]);
    if (abs(v[0]+v[1]) < 0.1)
        landPub.publish(empty_msg);
}

int main(int argc, char **argv)
{
    init(argc ,argv, "ROS_Publisher");
    NodeHandle node;
    namedWindow("Robot Image");

    image_transport::ImageTransport it(node);

    mcl_data_subscriber = node.subscribe(mcl_data_publisher_name, 4, MyDataCallback);

    time_t temptime = time(0);
    std::cout << "Waiting for Handshake from Program .." << std::endl;
    while(!handshake_recieved && (time(0) - temptime) < 20)
    {
        Duration(0.05).sleep();
        spinOnce();
    }
    if(handshake_recieved)
        std::cout << "Handshake recieved" << std::endl;
    else
    {
        std::cout << "No handshake recieved";
        return -1;
    }
    
    mcl_movement_publisher = node.advertise<std_msgs::String>("ROBOT_MOVEMENT_PUBLISHER", 4);
    image_publisher = it.advertise(publish_image_data_under, 4, true);

    // ARDrone stuff
    imageSub    = node.subscribe("/ardrone/image_raw", 10, imageCallback);
    // navDataSub  = node.subscribe("/ardrone/navdata", 3, navCallback);
    landPub     = node.advertise<std_msgs::Empty>("/ardrone/land", 1);
    takeOffPub  = node.advertise<std_msgs::Empty>("/ardrone/takeoff", 1);
    cmdVelPub   = node.advertise<geometry_msgs::Twist>("/camd_vel", 1);

    robot.SetDestination(0, 0, 0);

    // Wait for connection
    Duration(4).sleep();

    // Take Off!
    if (fly)
        takeOffPub.publish(empty_msg);

    spin();

    // Land!
    if (fly)
        landPub.publish(empty_msg);
    destroyAllWindows();
}

float round(float x)
{
    return (float) ((int) (x*100))/100.0;
}

void Command(float x, float yaw_velocity)
{
    SetCommand(x, 0, 0, yaw_velocity);
    cmdVelPub.publish(cmd_msg);
}

void SetCommand(float x, float y, float z, float yaw_velocity)
{
    cmd_msg.linear.x  = x;
    cmd_msg.linear.y  = y;
    cmd_msg.linear.z  = z;
    cmd_msg.angular.z = yaw_velocity;
}