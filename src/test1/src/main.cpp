#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/Point32.h>
#include <geometry_msgs/Pose.h>
#include <nav_msgs/Odometry.h>
#include <sensor_msgs/Joy.h>
#include <cstdlib>
#include <ctime>
#include "sensor_msgs/Range.h"
#include <sensor_msgs/PointCloud.h>
#include <tf/LinearMath/Quaternion.h>
#include <tf/transform_listener.h>

#include <sensor_msgs/LaserScan.h>


#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/opencv.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/core/core.hpp>
#include <opencv2/features2d/features2d.hpp>


using namespace std;
using namespace cv;


class MyPoint
{
public:

    MyPoint()
    {
    }

    ~MyPoint()
    {
    }
    double x;
    double y;
    double distance;
};

//definicje parametrow

int scale = 60; // ile px na 1 metr
int x_centre = 200; // polozenie srodka ukladu wspolrzednych (odom)
int y_centre = 200; // polozenie srodka ukladu wspolrzednych (odom)
int x_size = 801; // rozmiar mapy
int y_size = 801; // rozmiar mapy


bool laser_first_run = 1;
bool pose_first_run = 1;
double last_pose_x = 0; // ostatnie odebrane polozenie robota
double last_pose_y = 0; // ostatnie odebrane polozenie robota
double last_pose_angle = 0; // ostatnie odebrane polozenie robota

Scalar color = Scalar( 255, 255, 255 ); // kolor przeszkody na mapie
Scalar color2 = Scalar( 255, 0, 0 ); // kolor trasy robota

sensor_msgs::LaserScan pastData;
MyPoint outData[2000];

float num_ranges; // zmienna przechowujaca ilosc pomiarow z lasera

cv::Mat mapa = Mat::zeros( x_size, y_size, CV_8UC3 ); // zmienna zawierajaca budowana mape
cv::Mat trasa = Mat::zeros( x_size, y_size, CV_8UC3 ); // zmienna zawierajaca trase po ktorej poruszal sie robot

void poseCallback(const nav_msgs::Odometry &msg)
{
    // funkcja aktualizujaca pozycje robota w pamieci
    tf::Pose pozycja;
    tf::poseMsgToTF(msg.pose.pose, pozycja);

    tf::Quaternion quaternion(0,0,0,0);
    quaternion.setW(msg.pose.pose.orientation.w);
    quaternion.setZ(msg.pose.pose.orientation.z);
    quaternion.normalize();
    last_pose_x=msg.pose.pose.position.x;
    last_pose_y=msg.pose.pose.position.y;
    last_pose_angle=tf::getYaw(pozycja.getRotation());
    pose_first_run=0;

}

void DrawPoints()
{
    //funkcja rysujaca mape i trase

    for (int i = 0; i < num_ranges-1; i++)
    {
        if(outData[i].distance > 0.0 && outData[i].distance==outData[i].distance) // jesli pomiar z lasera jest poprawny
        {
            circle(mapa,Point( x_centre + scale*outData[i].x, y_centre + scale*outData[i].y ),1,color,1,8,0); // oznacz przeszkode na mapie
            line(mapa,Point( x_centre + scale*outData[i].x, y_centre + scale*outData[i].y ),Point( x_centre + scale*last_pose_y, y_centre + scale*last_pose_x ),Scalar(0,0,0),1,8,0); // oznacz obszar miedzy robotem a przeszkoda jako przejezdny
        }
    }
    circle(trasa,Point( x_centre + scale*last_pose_y, y_centre + scale*last_pose_x),1,color2,1,8,0); // oznacz pozycje robota na trasie

    char* source_window = "Mapa rastrowa";
    namedWindow( source_window, CV_WINDOW_AUTOSIZE );
    imshow( source_window, mapa+trasa ); // wyswietl mape z nalozona trasa
    waitKey(1);

}


void laserCallback(const sensor_msgs::LaserScan & laserscan)
{
    // funkcja obslugujaca skany z lasera
    if(laser_first_run)
    {
        // w pierwszej iteracji wyczysc mape i trase
        mapa = Scalar(0,0,0);
        trasa = Scalar(0,0,0);
        laser_first_run=0;
    }
    if(pose_first_run) return; // jesli pozycja robota nie jest znana pomin ten skan

    num_ranges = laserscan.ranges.size(); // liczba punktow z lasera
    float dangle = laserscan.angle_increment; // kat pomiedzy kolejnymi wiazkami lasera

    for (int i = 0; i < num_ranges; i++) // dla kazdej wiazki obliczany jest punkt na mapie
    {
        double angle = laserscan.angle_min + i * dangle + last_pose_angle; // kat bezwzgledny

        if (laserscan.ranges[i] > 0.019 && laserscan.ranges[i] < 4) // tylko punkty z zadanego zakresu
        {
            outData[i].x = laserscan.ranges[i] * sin(angle) + last_pose_y;
            outData[i].y = laserscan.ranges[i] * cos(angle) + last_pose_x;
            outData[i].distance = laserscan.ranges[i];
        }
        else
        {
            outData[i].distance = 0;
        }
    }

    DrawPoints(); // rysuj punkty na mapie
}

// ----------------- funkcja główna ----------------- 
int main(int argc, char** argv)
{
    ros::init(argc, argv, "aifl1");
    ROS_INFO("NODE STARTED");

    //Ros variables
    ros::NodeHandle n;
    ros::Rate rate(10);

    ros::Subscriber odompub = n.subscribe("/batman/aria/pose", 1, poseCallback); //Odometry subscriber
    ros::Subscriber lasersub = n.subscribe("/batman/scan", 1, laserCallback); //laserscan subscriber

    // ----------------- pętla główna -----------------
    while(n.ok())
    {
        //Sleep
        rate.sleep();
        ros::spinOnce();
    }

    // ----------------- koniec pętli głównej -----------------
    
    imwrite( "map.png", mapa); // zapis mapy
    ROS_WARN("THX GBYE");

    return 0;
}
