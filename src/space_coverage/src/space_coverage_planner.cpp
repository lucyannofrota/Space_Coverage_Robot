#include <ros/ros.h>
// #include <sensor_msgs/LaserScan.h>

#include <opencv2/core.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/imgproc.hpp>

#include <nav_msgs/Odometry.h>
#include <nav_msgs/OccupancyGrid.h>
#include <std_msgs/String.h>

#include <visualization_msgs/Marker.h>

const float cell_size_m = (0.225*2.0)*(3.0/5.0); // 2* Robot radius
const float occ_threshold = 0.5;

#include "SC_planner.hpp"

static SC_planner *planner = NULL;
static geometry_msgs::Point new_pose;
static cv::Mat_<uint8_t> *map = NULL;
// static cv::Mat_<uint8_t> *gridOverlay = NULL;

// static ros::Publisher vis_pub;

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

    map = new cv::Mat_<uint8_t>;

    *map = cv::Mat::zeros(msg.info.height,msg.info.width,CV_8UC1);

    for(int i = msg.info.height; i >= 0; i--){
        for(int j = 0; j < msg.info.width; j++){
            if(*mapDataIter == -1){
                map->at<uchar>(i,j) = 127;
            }
            else{
                if(*mapDataIter >= occ_threshold) map->at<uchar>(i,j) = 0;
                else map->at<uchar>(i,j) = 255;
            }
            // map->at<uchar>(i,j) = 127-(255/100)*(*mapDataIter);
            ++mapDataIter;
        }
    }





    // TODO Fazer crop da imagem. E resolver a transformada em outra escala e que relaciona um ponto na imagem com o mapa
}

// void initializeGridOverlay(void){
//     // gridOverlay
//     // *gridOverlay = cv::Mat::zeros(msg.info.height,msg.info.width,CV_8UC1);
// }

void mapCallBack(const nav_msgs::OccupancyGrid& msg){
    
    if(map == NULL){
        initializeMap(msg);
        planner->setup_planner(msg);
    }


    
    // cv::imshow("Space Coverage Planner", img0);
	// cv::waitKey(1);
    drawCallback();
}

int main(int argc, char **argv){
	ros::init(argc, argv, "SC_Planner");
	ros::NodeHandle nh;

    cv::namedWindow("Space Coverage Planner");

    planner = new SC_planner(cell_size_m);

    ros::Publisher vis_pub = nh.advertise<visualization_msgs::Marker>( "SC_Planner", 0 );

    planner->define_pub(vis_pub);






    ros::Subscriber subs_odom = nh.subscribe("/odom", 10, odomCallback);
    ros::Subscriber subs_map = nh.subscribe("/map", 3, mapCallBack);

	ros::Rate rate(10);
	while(ros::ok()){

        // planner->iterate();
        // publishMarkers(vis_pub);

		rate.sleep();
		ros::spinOnce();
	}
}







