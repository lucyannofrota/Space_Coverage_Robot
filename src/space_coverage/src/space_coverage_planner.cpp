#include <ros/ros.h>
// #include <sensor_msgs/LaserScan.h>

#include <opencv2/core.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/imgproc.hpp>

#include <nav_msgs/Odometry.h>
#include <nav_msgs/OccupancyGrid.h>
#include <std_msgs/String.h>

const float cell_size = 5;

#include "SC_planner.hpp"

SC_planner *planner;
geometry_msgs::Point new_pose;
cv::Mat_<int8_t> *map;

// #include <opencv2/imgproc.hpp>

// void lidarCallback(const sensor_msgs::LaserScan& scan){}

#include <iostream>
// #include <GridMap2D.h>

void drawCallback(void){
    try{
        cv::imshow("Space Coverage Planner", *map);
        cv::waitKey(1);
    }
    catch(...){
        std::cout << "Error" << std::endl;
    }
}

void odomCallback(const nav_msgs::Odometry& msg){
    std::cout << "x: " << msg.pose.pose.position.x 
              << "\ty: " << msg.pose.pose.position.y
              << "\tz: " << msg.pose.pose.position.z
              << std::endl;
    if(planner!=NULL){
        if(planner->iterate(msg.pose.pose.position,new_pose)){
            // new_pose is valid!
        }
    }
    // cell_size
    drawCallback();
}

void initializeMap(const nav_msgs::OccupancyGrid& msg){
    // Code based on:
    //      https://github.com/ROBOTIS-GIT/humanoid_navigation/blob/master/gridmap_2d/src/GridMap2D.cpp

    std::vector<signed char>::const_iterator mapDataIter = msg.data.begin();

    *map = cv::Mat::zeros(msg.info.height,msg.info.width,CV_8UC1);
    // *map = cv::Mat::zeros(400,400,CV_8UC1);

    for(int i = msg.info.height; i >= 0; i--){
        for(int j = 0; j < msg.info.width; j++){
    // for(int i = 0; i < 400; i++){
    //     for(int j = 0; j < 400; j++){
            // std::cout << *mapDataIter << std::endl;
            if(*mapDataIter == -1){
                map->at<uchar>(i,j) = 0;
            }
            else{
                map->at<uchar>(i,j) = 127-(255/100)*(*mapDataIter);
                // if(*mapDataIter < 0){
                //     map->at<uchar>(i,j) = 0;
                // }
                // else map->at<uchar>(i,j) = 127;
            }

            // }
            // map->at<uchar>(i,j) = *mapDataIter;
            ++mapDataIter;
        }
        
    }
}

void mapCallBack(const nav_msgs::OccupancyGrid& msg){
    

    initializeMap(msg);

    
    // cv::imshow("Space Coverage Planner", img0);
	// cv::waitKey(1);
    drawCallback();
}

int main(int argc, char **argv){
	ros::init(argc, argv, "SC_Planner");
	ros::NodeHandle nh;

    map = new cv::Mat_<int8_t>;
    cv::namedWindow("Space Coverage Planner");

    planner = new SC_planner();

	ros::Subscriber subs_odom = nh.subscribe("/odom", 10, odomCallback);
    ros::Subscriber subs_map = nh.subscribe("/map", 3, mapCallBack);

	ros::Rate rate(10);
	while(ros::ok()){
		rate.sleep();
		ros::spinOnce();
	}
}







