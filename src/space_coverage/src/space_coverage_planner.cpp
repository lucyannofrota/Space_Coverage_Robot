#include <ros/ros.h>
// #include <sensor_msgs/LaserScan.h>

#include <opencv2/core.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/imgproc.hpp>

#include <nav_msgs/Odometry.h>
#include <nav_msgs/OccupancyGrid.h>
#include <std_msgs/String.h>

#include <visualization_msgs/Marker.h>

#include <move_base_msgs/MoveBaseAction.h>
#include <actionlib/client/simple_action_client.h>


const float cell_size_m = (0.225*2.0)*(0.7); // 2* Robot radius
const float occ_threshold = 0.5;

#include "SC_planner.hpp"
#include <stdio.h>

static SC_planner *planner = NULL;
// static geometry_msgs::Point new_pose;
static cv::Mat_<uchar> *map = NULL;
static unsigned int mapExtremes[4]; // [min_x,max_x,min_y,max_y]
// static cv::Mat_<uint8_t> *gridOverlay = NULL;

// static ros::Publisher vis_pub;

// #include <opencv2/imgproc.hpp>

// void lidarCallback(const sensor_msgs::LaserScan& scan){}

#include <iostream>
// #include <GridMap2D.h>

void drawCallback(void){
    try{
        if(map != NULL){
            // cv::imshow("Space Coverage Planner", *map);
            // cv::waitKey(1);
        }
    }
    catch(.../*cv::Exception ex*/){
        std::cout << "drawCallback error!" << std::endl;
        // std::cout << ex.msg << std::endl;
    }
}

void odomCallback(const nav_msgs::Odometry& msg){
    try{
        // std::cout << "x: " << msg.pose.pose.position.x 
        //       << "\ty: " << msg.pose.pose.position.y
        //       << "\tz: " << msg.pose.pose.position.z
        //       << std::endl;
        if(planner!=NULL){
            if(planner->iterate(msg.pose.pose)){
                // new_pose is valid!
                // std::cout << "planner iteration" << std::endl;
            }
        }
        // cell_size
        drawCallback();
    }
    catch (...){
        std::cout << "odomCallback error!" << std::endl;
    }
}

void initializeMap(const nav_msgs::OccupancyGrid& msg){
    // Code based on:
    //      https://github.com/ROBOTIS-GIT/humanoid_navigation/blob/master/gridmap_2d/src/GridMap2D.cpp
    try{
        std::vector<signed char>::const_iterator mapDataIter = msg.data.begin();

        map = new cv::Mat_<uchar>;

        *map = cv::Mat::zeros(msg.info.height,msg.info.width,CV_8UC1);

        for(int i = 0; i < msg.info.height; i++){
            for(int j = 0; j < msg.info.width; j++){
                if(*mapDataIter == -1){
                    map->at<uchar>(i,j) = 127;
                }
                else{
                    if(*mapDataIter >= occ_threshold) map->at<uchar>(i,j) = 0;
                    else map->at<uchar>(i,j) = 255;
                }
                ++mapDataIter;
            }
        }

        mapExtremes[0] = 9999;
        mapExtremes[1] = 0;
        mapExtremes[2] = 9999;
        mapExtremes[3] = 0;

        for(int i = 0; i < msg.info.height; i++){
            for(int j = 0; j < msg.info.width; j++){
                if(map->at<uchar>(i,j) == 255){
                    if(j < mapExtremes[0]) mapExtremes[0] = j;
                    if(j > mapExtremes[1]) mapExtremes[1] = j;
                    if(i < mapExtremes[2]) mapExtremes[2] = i;
                    if(i > mapExtremes[3]) mapExtremes[3] = i;
                }
            }
        }
        printf("X: [%u,%u], Y: [%u,%u]\n",mapExtremes[0],mapExtremes[1],mapExtremes[2],mapExtremes[3]);
    }
    catch (...) {
        std::cout << "initializeMap function error!" << std::endl;
    }

}

void mapCallBack(const nav_msgs::OccupancyGrid& msg){
    try{
        if(map == NULL){
            initializeMap(msg);
            planner->setup_planner(msg,mapExtremes,*map,msg.info.resolution);
        }


        
        // cv::imshow("Space Coverage Planner", img0);
        // cv::waitKey(1);
        drawCallback();
    }
    catch (...){
        std::cout << "mapCallback error!" << std::endl;
    }
}

int main(int argc, char **argv){
    try{
        ros::init(argc, argv, "SC_Planner");
        ros::NodeHandle nh;

        cv::namedWindow("Space Coverage Planner");

        //tell the action client that we want to spin a thread by default
        MoveBaseClient ac("move_base", true);

        //wait for the action server to come up
        while(!ac.waitForServer(ros::Duration(5.0))){
            ROS_INFO("Waiting for the move_base action server to come up");
        }

        planner = new SC_planner(ac,cell_size_m);


        ros::Publisher vis_pub = nh.advertise<visualization_msgs::Marker>( "/SC_Planner_Markers", 5 );
        // ros::Publisher goal_pub = nh.advertise<visualization_msgs::Marker>( "/move_base/goal", 5 );

        planner->define_pubMarker(vis_pub);
        // planner->define_pubGoal(goal_pub);


        ros::Subscriber subs_map = nh.subscribe("/map", 3, mapCallBack);
        ros::Subscriber subs_odom = nh.subscribe("/odom", 5, odomCallback);

        ros::Rate rate(10);
        while(ros::ok()){

            // planner->iterate();
        //     // publishMarkers(vis_pub);

            rate.sleep();
            ros::spinOnce();
        }
    }
    catch (...){
        std::cout << "main function error!" << std::endl;
    }

}








