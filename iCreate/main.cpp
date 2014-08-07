#include "ros/ros.h"
#include "std_msgs/String.h"
#include "cv_bridge/cv_bridge.h"
#include "sensor_msgs/Image.h"
#include "sensor_msgs/image_encodings.h"
#include <opencv2/opencv.hpp>
#include <image_transport/image_transport.h>
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

// Robot Movement Algorithm
#define P2P 0
#define USERINPUT 1
#define TESTSTATE 2

Publisher create_move;
Publisher mcl_movement_publisher;
image_transport::Publisher image_publisher;
Subscriber mcl_data_subscriber;
Subscriber move_subscriber;

Subscriber imageSub;
// Subscriber navDataSub;

Robot robot;
vector<float> move;

const std::string publish_image_data_under = "ROBOT_IMAGE_PUBLISHER";
const std::string mcl_data_publisher_name = "MCL_DATA_PUBLISHER";

const int handshake = 15;
const int readymove = 25;
const int guessdata = 35;
const int starting_move = 10;
const int finished_move = 20;

bool handshake_recieved = false;

// Some declarations
float round(float x);
void Command(float x, float turn);
void Move(vector<float> v);

void moveDone (const std_msgs::String str)
{
    cout << "Move Is Done" << endl;
    std_msgs::String msg;
    stringstream ss;
    ss << "20" << " " << move[1] << " " << move[0] << " ";
    msg.data = ss.str();
    mcl_movement_publisher.publish(msg);
}

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

    char key = 'k';
    imshow("Robot Image", im2->image);
    key = waitKey(2);
    if (key == 'q')
    {
        std_msgs::String msg;
        stringstream ss;
        ss << "20" << " " << move[1] << " " << move[0] << " ";
        msg.data = ss.str();
        mcl_movement_publisher.publish(msg);
        ss.str("");
        ss << 666 << "_";
        msg.data = ss.str();
        mcl_movement_publisher.publish(msg);
        ros::shutdown();
        return;
    }

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

    move = robot.NextMove();
    Move(move);
    // ss.str("");
    // ss << "20" << " " << move[1] << " " << move[0] << " ";
    // msg.data = ss.str();
    // mcl_movement_publisher.publish(msg);
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

void Move(vector<float> v)
{
    cout << "Turn: " << v[0] << ", Translate: " << v[1] << endl;
    if (abs(v[0]) + abs(v[1]) > 0.1)
        Command(v[1], (int) v[0]);
    else
        Command(0, 0);
    // Duration(2).sleep();
}

int main(int argc, char **argv)
{
    init(argc ,argv, "ROS_Publisher");
    NodeHandle node;
    namedWindow("Robot Image");

    image_transport::ImageTransport it(node);

    if (argc == 2)
        robot.SetState(atoi(argv[1]));
    else
        robot.SetState(TESTSTATE);
    robot.SetDestination(0.5, 0.1, 1.5);

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
    
    create_move = node.advertise<std_msgs::String>("move_commands", 4);
    mcl_movement_publisher = node.advertise<std_msgs::String>("ROBOT_MOVEMENT_PUBLISHER", 1);
    image_publisher = it.advertise(publish_image_data_under, 1, true);

    // ARDrone stuff
    imageSub    = node.subscribe("/ardrone/image_raw", 1, imageCallback);
    move_subscriber = node.subscribe("move_done", 1, moveDone);
    cout << "Connected!" << endl;
    // robot.SetDestination(0, 0, 0);

    // Wait for connection
    Duration(4).sleep();

    spin();

    destroyAllWindows();
}

float round(float x)
{
    return (float) ((int) (x*100))/100.0;
}

void Command(float forward, float turn)
{
    stringstream ss;
    ss << forward << " " << turn;
    std_msgs::String msg;
    msg.data = ss.str();
    create_move.publish(msg);
}
