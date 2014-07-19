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
Subscriber navDataSub;

Publisher landPub;
Publisher takeOffPub;
Publisher cmdVelPub;

std_msgs::Empty empty_msg;
geometry_msgs::Twist cmd_msg;


const std::string publish_image_data_under = "ROBOT_IMAGE_PUBLISHER";
const std::string mcl_data_publisher_name = "MCL_DATA_PUBLISHER";

const int handshake = 15;
const int readymove = 25;
const int starting_move = 10;
const int finished_move = 20;

bool handshake_recieved = false;

float round(float x);
void tl(int torl);
void SetCommand(float roll, float pitch, float yaw_velocity, float z_velocity);

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
    image_publisher.publish(img);
}

void navCallback(ardrone_autonomy::Navdata msg)
    {
        ;
    }


void publish_Move()
{
    stringstream ss;
    std_msgs::String msg;

    msg.data = "10";
    mcl_movement_publisher.publish(msg);

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
    mcl_movement_publisher.publish(msg);
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

void move(float t)
{
    if (abs(t) > 1)
        return;
    SetCommand(t, 0, 0, 0);
    cmdVelPub.publish(cmd_msg);
    Duration(1).sleep();
    SetCommand(0, 0, 0, 0);
    cmdVelPub.publish(cmd_msg);
}

#define TIMETOTURNAROUND 1.0

void turn(int deg)
{
    SetCommand(0, 0, 1, 0);
    float t = (float) deg * TIMETOTURNAROUND / 360.0;
    Duration(t).sleep();
    cout << t << endl;
    SetCommand(0, 0, 0, 0);

}

int main(int argc, char **argv)
{
    init(argc ,argv, "ROS_Publisher");
    NodeHandle node;
    namedWindow("Robot Image");

    image_transport::ImageTransport it(node);

    // mcl_data_subscriber = node.subscribe(mcl_data_publisher_name, 4, MyDataCallback);

    // time_t temptime = time(0);
    // std::cout << "Waiting for Handshake from Program .." << std::endl;
    // while(!handshake_recieved && (time(0) - temptime) < 20)
    // {
    //     Duration(0.05).sleep();
    //     spinOnce();
    // }
    // if(handshake_recieved)
    //     std::cout << "Handshake recieved" << std::endl;
    // else
    // {
    //     std::cout << "No handshake recieved";
    //     return -1;
    // }
    
    mcl_movement_publisher = node.advertise<std_msgs::String>("ROBOT_MOVEMENT_PUBLISHER", 4);
    image_publisher = it.advertise(publish_image_data_under, 4, true);

    // ARDrone stuff
    imageSub    = node.subscribe("/ardrone/image_raw", 10, imageCallback);
    navDataSub  = node.subscribe("/ardrone/navdata", 3, navCallback);
    landPub     = node.advertise<std_msgs::Empty>("/ardrone/land", 1);
    takeOffPub  = node.advertise<std_msgs::Empty>("/ardrone/takeoff", 1);
    cmdVelPub   = node.advertise<geometry_msgs::Twist>("/cmd_vel", 1);

    Duration(4).sleep();
    cout << "Liftoff" << endl;

    tl(0);
    char c = getchar();
    char q = 'q';
    while (c != q)
    {
        c = getchar();
        if (c == 'i')
            move(1);
        else if (c == 'k')
            move(-1);
        else if (c == 'j')
            turn(360);
        else if (c == 'l')
            turn(-1);
        // spinOnce();
    }
    tl(1);

    destroyAllWindows();
}

void tl(int torl) // takeoff or land?
{
    if (torl == 0)
        takeOffPub.publish(empty_msg);
    else
        landPub.publish(empty_msg);
}

float round(float x)
{
    return (float) ((int) (x*100))/100.0;
}

void SetCommand(float x, float y, float yaw_velocity, float z)
{
    cmd_msg.linear.x  = x;
    cmd_msg.linear.y  = y;
    cmd_msg.linear.z  = z;
    cmd_msg.angular.z = yaw_velocity;
}