#include <tf/tf.h>
#include <list> //For the path list
// #include <boost/bind/bind.hpp>

typedef actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> MoveBaseClient;

int DEBUG = 0;

enum SC_CELL_TYPE{OCC = 2,FREE = 1,INVALID = 0};

enum dir{UP,DOWN,LEFT,RIGHT};

struct grid_cord{
    uint16_t i; // y Height
    uint16_t j; // x Width
    dir direction;
    grid_cord(void){
        this->i = 0;
        this->j = 0;
        direction = dir::UP;
    }
    grid_cord(uint16_t i_,uint16_t j_){
        this->i = i_;
        this->j = j_;
    }
    grid_cord(uint16_t i_,uint16_t j_, dir direction_){
        this->i = i_;
        this->j = j_;
        this->direction = direction_;
    }
};

class SC_planner{

    private:

    float cell_size_m;
    uint16_t cell_size_pix;
    cv::Mat_<uchar> gridMap; //0 - Invalid Cell, 1 - Free Cell, 2 - Occ Cell
    cv::Mat_<uchar> visitedTimes; //Array with the number of times a cell has been visited
    cv::Mat_<uchar> *baseMap;
    float baseMap_resolution;

    // Mekers_Publisher
    ros::Publisher *MarkerPub; // Will publish if defined
    visualization_msgs::Marker cell_marker;
    visualization_msgs::Marker path_marker;


    // Cell_colors
    std_msgs::ColorRGBA c_occ;
    std_msgs::ColorRGBA c_free;
    bool initialized_markers;
    unsigned int mapExtremes[4];


    // geometry_msgs::Pose last_pose;
    grid_cord gridPose;
    bool initialPose_defined = false;
    // grid_cord last_goal;
    grid_cord current_goal;
    std::list <grid_cord> path; //List with the nodes the robot went 
    bool stuck; //If the robot is stuck with no new cells to discover


    MoveBaseClient *MB_client;

    public:
    bool goal_reached = true;
    SC_planner(MoveBaseClient &MB_client_,float cell_m){
        this->cell_size_m = cell_m;

        this->cell_size_pix = 0;

        this->MarkerPub = NULL; 
        this->MB_client = &MB_client_;

        this->c_occ.a = 0.2;
        this->c_occ.r = 0.0;
        this->c_occ.g = 0.5;
        this->c_occ.b = 0.0;

        this->c_free.a = 0.2;
        this->c_free.r = 80.0/255.0;
        this->c_free.g = 80.0/255.0;
        this->c_free.b = 1.0;

        this->initialized_markers = false;
    }

    ~SC_planner(){
        
    };

    uint16_t get_cell_size_pix(void){
        return this->cell_size_pix;
    }

    void setup_planner(const nav_msgs::OccupancyGrid& msg, unsigned int *mapExtremes, cv::Mat_<uchar> &base_map, float map_res){
        
        this->baseMap_resolution = map_res;
        this->baseMap = &base_map;
        this->cell_size_pix = ceil(this->cell_size_m*(1/msg.info.resolution));

        uint16_t height = (int)floor((float) msg.info.height/cell_size_pix);
        uint16_t width = (int)floor((float) msg.info.width/cell_size_pix);

        this->gridMap = cv::Mat::zeros(height,width,CV_8UC1);
        this->visitedTimes = cv::Mat::zeros(this->gridMap.rows,this->gridMap.cols,CV_8UC1);

        std::cout << "Cell Size: " << this->cell_size_pix << std::endl;
        std::cout << "Map h: " << this->gridMap.rows << std::endl;
        std::cout << "Map w: " << this->gridMap.cols << std::endl;

        this->mapExtremes[0] = mapExtremes[0];
        this->mapExtremes[1] = mapExtremes[1];
        this->mapExtremes[2] = mapExtremes[2];
        this->mapExtremes[3] = mapExtremes[3];

        // Fill the grid
        for(uint16_t i = 0; i < this->gridMap.rows; i++){
            for(uint16_t j = 0; j < this->gridMap.cols; j++){
                bool free = true;
                for(uint16_t cy = 0; cy < this->cell_size_pix; cy++){
                    uint16_t y = cy+i*this->cell_size_pix;
                    if(y > this->baseMap->rows || y < this->mapExtremes[2] || y > this->mapExtremes[3]){
                        this->gridMap.at<uchar>(i,j) = SC_CELL_TYPE::INVALID;
                        goto label1;
                    }
                    for(uint16_t cx = 0; cx < this->cell_size_pix; cx++){
                        uint16_t x = cx+j*this->cell_size_pix;
                        if(x > this->baseMap->cols || x < this->mapExtremes[0] || x > this->mapExtremes[1]){
                            this->gridMap.at<uchar>(i,j) = SC_CELL_TYPE::INVALID;
                            goto label1;
                        }

                        if(this->baseMap->at<uchar>(y,x) <= 200) free = false;
                    }
                }
                if(free) this->gridMap.at<uchar>(i,j) = SC_CELL_TYPE::FREE;
                else this->gridMap.at<uchar>(i,j) = SC_CELL_TYPE::INVALID;
                label1:
                continue;
            }
        }

    }

    void define_rviz_Markers(ros::Publisher &publisher){
        this->MarkerPub = &publisher;

        // Base marker definition
        this->cell_marker.header.frame_id = "map";
        this->cell_marker.header.stamp = ros::Time();
        this->cell_marker.ns = "SC_Planner_Cells";
        this->cell_marker.id = 0;
        this->cell_marker.type = visualization_msgs::Marker::CUBE_LIST;
        this->cell_marker.action = visualization_msgs::Marker::ADD;
        this->cell_marker.pose.position.x = 0;
        this->cell_marker.pose.position.y = 0;
        this->cell_marker.pose.position.z = 0;
        this->cell_marker.pose.orientation.x = 0.0;
        this->cell_marker.pose.orientation.y = 0.0;
        this->cell_marker.pose.orientation.z = 0.0;
        this->cell_marker.pose.orientation.w = 1.0;
        this->cell_marker.scale.x = this->cell_size_m;
        this->cell_marker.scale.y = this->cell_size_m;
        this->cell_marker.scale.z = 0.01;
        // this->marker.color.a = 1.0; // Don't forget to set the alpha!
        this->cell_marker.lifetime.fromSec(0.15);


        this->path_marker.header.frame_id = "map";
        this->path_marker.header.stamp = ros::Time();
        this->path_marker.ns = "SC_Planner_Path";
        this->path_marker.id = 1;
        this->path_marker.type = visualization_msgs::Marker::LINE_STRIP;
        this->path_marker.action = visualization_msgs::Marker::ADD;
        this->path_marker.pose.position.x = 0;
        this->path_marker.pose.position.y = 0;
        this->path_marker.pose.position.z = 0;
        this->path_marker.pose.orientation.x = 0.0;
        this->path_marker.pose.orientation.y = 0.0;
        this->path_marker.pose.orientation.z = 0.0;
        this->path_marker.pose.orientation.w = 1.0;
        this->path_marker.scale.x = 0.03;
        this->path_marker.scale.y = 0.03;
        this->path_marker.scale.z = 0.03;
        this->path_marker.color.a = 1;
        this->path_marker.color.r = 1;
        this->path_marker.color.g = 0;
        this->path_marker.color.b = 0;
        this->path_marker.lifetime.fromSec(0.15);


        // Path points demo
        // geometry_msgs::Point pt;
        // bool tmp = false;
        // pt.z = 0.01;
        // for(int a = 7; a < 14; a++){
        //     pt.x = a;
        //     pt.y = tmp ? 7 : 3;
        //     tmp = !tmp; 
        //     this->path_marker.points.push_back(pt);
        // }
    }

    // bool validDisplacement(geometry_msgs::Pose pose){
    //     //TODO


    //     // float disp = sqrt(msg.pose.pose.position.x^2)
    //     // msg.pose.pose.pos
    //     // this->last_pose = 
    //     return true;
    // }

    void detect_neighbors(const grid_cord &current_cell, bool *validCells){
        if(this->gridMap.at<uchar>(current_cell.i,current_cell.j-1) == 1)
            validCells[0] = true;
        else
            validCells[0] = false; 

        if(this->gridMap.at<uchar>(current_cell.i-1,current_cell.j) == 1)
            validCells[1] = true;
        else
            validCells[1] = false;

        if(this->gridMap.at<uchar>(current_cell.i,current_cell.j+1) == 1)
            validCells[2] = true;
        else
            validCells[2] = false;
        
        if(this->gridMap.at<uchar>(current_cell.i+1,current_cell.j) == 1)
            validCells[3] = true;
        else
            validCells[3] = false; 
    }

    bool iterate(const geometry_msgs::Pose &current_pose){
        
        // Hit detection
        static grid_cord temp_gridPose;
        temp_gridPose = this->transform_world_to_grid(current_pose);
        if(this->gridMap.at<uchar>(temp_gridPose.i,temp_gridPose.j) == SC_CELL_TYPE::FREE) this->gridMap.at<uchar>(temp_gridPose.i,temp_gridPose.j) = SC_CELL_TYPE::OCC;

        bool validCells[4]; //0 - esquerda 1- baixo ... 

        if (DEBUG == 1)
            printf("GridPose: (%u,%u,%u,%u)\n",temp_gridPose.j,temp_gridPose.i,temp_gridPose.direction,this->gridMap.at<uchar>(temp_gridPose.i,temp_gridPose.j));

        if(this->goal_reached){
            
            spiralSTC_online(temp_gridPose,validCells);

            this->goal_pub(temp_gridPose);
            
        }

        this->pubMarkers();
        return true;
    }

    void spiralSTC_online(const grid_cord &temp_gridPose, bool *validCells){
        this->detect_neighbors(temp_gridPose,validCells);
        if (DEBUG == 1)
        {
            printf("Curr Cell: %u,%u,%u\n",temp_gridPose.j,temp_gridPose.i,temp_gridPose.direction);
            if(validCells[0] == true) printf("Left Cell is valid!\n");
            if(validCells[1] == true) printf("Down Cell is valid!\n");
            if(validCells[2] == true) printf("Right Cell is valid!\n");
            if(validCells[3] == true) printf("Up Cell is valid!\n");
        }

        grid_cord next_goal;
        bool stuck = this->decisionBase(temp_gridPose,next_goal,validCells);

        // If the Robot gets stuck
        while (stuck == true)
        {
            this->path.reverse(); //Reverse the list to get the last nodes
            for (auto itr = this->path.begin(); itr != this->path.end(); itr++) //Go through the list
            {
                this->detect_neighbors(*itr,validCells); //Test the node
                // this->current_goal = 
                stuck = this->decisionBase(*itr,next_goal,validCells);
                //If the node "unstucks" the robot -> Proceed
                if(stuck == false)
                {
                    this->path.reverse();
                    break;
                }
            }
        }
        this->current_goal = next_goal;
    }
/*
    void spiralSTC(const grid_cord &temp_gridPose,bool *validCells){
        this->detect_neighbors(temp_gridPose,validCells);
        if (DEBUG == 1)
        {
            printf("Curr Cell: %u,%u,%u\n",temp_gridPose.j,temp_gridPose.i,temp_gridPose.direction);
            if(validCells[0] == true) printf("Left Cell is valid!\n");
            if(validCells[1] == true) printf("Down Cell is valid!\n");
            if(validCells[2] == true) printf("Right Cell is valid!\n");
            if(validCells[3] == true) printf("Up Cell is valid!\n");
        }

        grid_cord next_goal;
        bool stuck = this->decisionBase(temp_gridPose,next_goal,validCells);

        // If the Robot gets stuck
        while (stuck == true)
        {
            this->path.reverse(); //Reverse the list to get the last nodes
            for (auto itr = this->path.begin(); itr != this->path.end(); itr++) //Go through the list
            {
                this->detect_neighbors(*itr,validCells); //Test the node
                // this->current_goal = 
                stuck = this->decisionBase(*itr,next_goal,validCells);
                //If the node "unstucks" the robot -> Proceed
                if(stuck == false)
                {
                    this->path.reverse();
                    break;
                }
            }
        }
        this->current_goal = next_goal;
    }*/

    void goal_pub(const grid_cord &temp_gridPose){
        // Publishing goal
        if(this->MB_client != NULL){
            move_base_msgs::MoveBaseGoal goal;
            goal.target_pose.header.frame_id = "map";
            goal.target_pose.header.stamp = ros::Time::now();

            goal.target_pose.pose = this->transform_grid_to_world(this->current_goal);
            printf("Sending Goal!\n");
            this->visitedTimes.at<uchar>(temp_gridPose.i,temp_gridPose.j) += 1;
            this->path.push_back(this->current_goal);
            if (this->path.size()>100){
                for (int i = 0;i<20;i++){
                    printf("Clearing the path, for memory management...\n");
                    this->path.erase(this->path.begin());
                }
            }
            printf("Goal: %f,%f,%f\n",goal.target_pose.pose.position.x,goal.target_pose.pose.position.y,tf::getYaw(goal.target_pose.pose.orientation));
            printf("Cell Goal: %u,%u,%u\n",this->current_goal.j,this->current_goal.i,this->current_goal.direction);
            this->goal_reached = false;
            this->MB_client->sendGoal(goal,
                                        // &SC_planner::goalCallback,
                                        boost::bind(&SC_planner::goalCallback, this, _1, _2),
                                        MoveBaseClient::SimpleActiveCallback(),
                                        MoveBaseClient::SimpleFeedbackCallback());

        }
        else ROS_WARN("No Move_Base client defined!\n");
    }

    void goalCallback(const actionlib::SimpleClientGoalState& state,
                        const move_base_msgs::MoveBaseResult::ConstPtr& result) {
        // https://answers.ros.org/question/202310/using-callbacks-with-simple-action-clients/
        if(state == actionlib::SimpleClientGoalState::SUCCEEDED){
            ROS_INFO("Goal reached!");
            // obj.goal_reached
            this->goal_reached = true;
        }
        else ROS_INFO("Goal failed");
    }

    bool decisionBase(const grid_cord &actual_pose, grid_cord &next_pose, bool *validCells){
        // return true if it got stuck

        // grid_cord next_pose;
        next_pose = actual_pose;

        if (validCells[0] == true)
        {
            next_pose= grid_cord(actual_pose.i,actual_pose.j-1,dir::LEFT);
            return false;
        }
        else if (validCells[1] == true){
            next_pose = grid_cord(actual_pose.i-1,actual_pose.j,dir::DOWN);
            return false;
        }
        else if (validCells[2] == true){
            next_pose = grid_cord(actual_pose.i,actual_pose.j+1,dir::RIGHT);
            return false;
        }
        else if (validCells[3] == true){
            next_pose = grid_cord(actual_pose.i+1,actual_pose.j,dir::UP);
            return false;
        }
        else
            return true;
    }

    geometry_msgs::Pose transform_grid_to_world(const grid_cord &cord){
        //TODO introduzir o valor da TF dinamico
        geometry_msgs::Pose out_pose;

        // TF comp
        out_pose.position.x += -1.0;
        out_pose.position.y += -1.0;

        // Center offset
        out_pose.position.x += this->cell_marker.scale.x/2;
        out_pose.position.y += this->cell_marker.scale.y/2;

        // Grid offset
        //BUG O tamanho de celula interfere na componente de overlap de escala
        out_pose.position.x += cord.j*(this->cell_marker.scale.x+this->baseMap_resolution/4); //   /2 para 0.6
        out_pose.position.y += cord.i*(this->cell_marker.scale.y+this->baseMap_resolution/4); // 0.02
        // tf::getYaw(pose.orientation)
        float angle;
        switch(cord.direction){
            case dir::RIGHT:
                angle = 0;
                break;
            case dir::UP:
                angle = M_PI_2;
                break;
            case dir::LEFT:
                angle = M_PI;
                break;
            case dir::DOWN:
                angle = -M_PI_2;
                break;
        }
        tf::Quaternion tempQ = tf::createQuaternionFromYaw(angle);
        // tempQ.
        out_pose.orientation.w = tempQ.getW();
        out_pose.orientation.x = tempQ.getX();
        out_pose.orientation.y = tempQ.getY();
        out_pose.orientation.z = tempQ.getZ();
        return out_pose;
    }

    grid_cord transform_world_to_grid(const geometry_msgs::Pose &pose){
        //TODO introduzir o valor da TF dinamico
        geometry_msgs::Pose temp = pose;
        grid_cord cord_out = grid_cord();

        // TF comp
        temp.position.x -= -1.0;
        temp.position.y -= -1.0;

        // Center offset
        temp.position.x -= this->cell_marker.scale.x/2;
        temp.position.y -= this->cell_marker.scale.y/2;

        // Grid offset
        cord_out.j = round(temp.position.x/(this->cell_marker.scale.x+this->baseMap_resolution/4));
        cord_out.i = round(temp.position.y/(this->cell_marker.scale.y+this->baseMap_resolution/4));

        if(cord_out.j >= this->gridMap.cols) cord_out.j = this->gridMap.cols;
        if(cord_out.i >= this->gridMap.rows) cord_out.i = this->gridMap.rows;
        // out_pose.x += j*this->marker.scale.x;
        // out_pose.y += i*this->marker.scale.y;
        // printf("world_to_grid: (%i, %i)\n", cord_out.j, cord_out.i);
        
        double aDir = tf::getYaw(pose.orientation);
        if(aDir >= M_PI_4 && aDir < 3*M_PI_4) cord_out.direction = dir::UP;
        else {
            if(aDir <= -M_PI_4 && aDir > -3*M_PI_4) cord_out.direction = dir::DOWN;
            else{
                if(aDir >= 3*M_PI_4 && aDir < 4*M_PI_4 || aDir <= -3*M_PI_4 && aDir > -4*M_PI_4) cord_out.direction = dir::LEFT;
                else cord_out.direction = dir::RIGHT;
            }
        }

        return cord_out;
    }

    private:

    void pubMarkers(void){
        geometry_msgs::Pose pose;

        const bool is_empty = this->cell_marker.points.empty();

        uint16_t idx = 0;

        for(uint16_t i = 0; i < this->gridMap.rows; i++){
            for(uint16_t j = 0; j < this->gridMap.cols; j++){
                pose = this->transform_grid_to_world(grid_cord(i,j));
                pose.position.z = 0.05;
                switch(this->gridMap.at<uchar>(i,j)){
                    case SC_CELL_TYPE::INVALID:
                        // This case should be empty!
                        // Used to see the complete gridMap on rviz
                        // Remove to filter the cells
                        // if(is_empty){
                        //     this->cell_marker.points.push_back(pose.position);
                        //     this->cell_marker.colors.push_back(this->c_free);
                        // }
                        // else this->cell_marker.colors[i*this->gridMap.cols+j] = this->c_free;
                        // idx++;
                        // if(is_empty){
                        //     this->path_marker.points.push_back(pose.position);
                        // }
                        break;
                    case SC_CELL_TYPE::FREE:
                        if(is_empty){
                            this->cell_marker.points.push_back(pose.position);
                            this->cell_marker.colors.push_back(this->c_free);
                        }
                        else this->cell_marker.colors[idx] = this->c_free;
                        idx++;
                        break;
                    case SC_CELL_TYPE::OCC:
                        if(is_empty){
                            this->cell_marker.points.push_back(pose.position);
                            this->cell_marker.colors.push_back(this->c_occ);
                        }
                        else this->cell_marker.colors[idx] = this->c_occ;
                        idx++;
                        break;
                }
                // if(this->gridMap.at<uchar>(i,j) == 0;
            }
        }
        // this->initialized_markers = true;
        this->MarkerPub->publish(this->cell_marker);
        this->MarkerPub->publish(this->path_marker);
    }

    // void move(void){
    //     for(uint16_t i = 0; i < this->gridMap.rows; i++){
    //         for(uint16_t j = 0; j < this->gridMap.cols; j++){

    //             // this->gridMap.at<uchar>(i,j);
    //             // pose = this->transform_grid_to_world(grid_cord(i,j));
    //             // pose.z = 0.05;
    //             // switch(this->gridMap.at<uchar>(i,j)){
    //             //     case 0:
    //             //         // this->marker.points.push_back(pose);
    //             //         // this->marker.colors.push_back(this->c_free);
    //             //         break;
    //             //     case 1:
    //             //         if(is_empty){
    //             //             this->marker.points.push_back(pose);
    //             //             this->marker.colors.push_back(this->c_free);
    //             //         }
    //             //         else this->marker.colors[i*this->gridMap.cols+j] = this->c_free;
    //             //         break;
    //             //     case 2:
    //             //         if(is_empty){
    //             //             this->marker.points.push_back(pose);
    //             //             this->marker.colors.push_back(this->c_occ);
    //             //         }
    //             //         else this->marker.colors[i*this->gridMap.cols+j] = this->c_occ;
    //             //         break;
    //             // }
    //             // if(this->gridMap.at<uchar>(i,j) == 0;
    //         }
    //     }
    // }

    // void getPlan(grid_cord startCell){
    //     //this->visitedTimes.at<uchar>(startCell.i,startCell.j) += 1;
    //     //printf("Test: %u\n",this->visitedTimes.at<uchar>(startCell.i,startCell.j));

    // }
};