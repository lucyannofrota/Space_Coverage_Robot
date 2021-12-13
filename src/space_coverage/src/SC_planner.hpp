#include <tf/tf.h>

typedef actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> MoveBaseClient;


enum SC_CELL_TYPE{OCC = 1,FREE = 0};

enum dir{UP,DOWN,LEFT,RIGHT};

struct grid_cord{
    uint16_t i; // Height
    uint16_t j; // Width
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
    cv::Mat_<uchar> *baseMap;
    float baseMap_resolution;

    // Mekers_Publisher
    ros::Publisher *MarkgerPub; // Will publish if defined
    visualization_msgs::Marker marker;


    // Cell_colors
    std_msgs::ColorRGBA c_occ;
    std_msgs::ColorRGBA c_free;
    bool initialized_markers;
    unsigned int mapExtremes[4];


    geometry_msgs::Pose last_pose;
    grid_cord gridPose;
    bool initialPose_defined = false;
    grid_cord last_goal;
    grid_cord current_goal;

    MoveBaseClient *MB_client;

    public:
    SC_planner(MoveBaseClient &MB_client_,float cell_m){
        this->cell_size_m = cell_m;

        this->cell_size_pix = 0;

        this->MarkgerPub = NULL; 
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
                        this->gridMap.at<uchar>(i,j) = 0;
                        goto label1;
                    }
                    for(uint16_t cx = 0; cx < this->cell_size_pix; cx++){
                        uint16_t x = cx+j*this->cell_size_pix;
                        if(x > this->baseMap->cols || x < this->mapExtremes[0] || x > this->mapExtremes[1]){
                            this->gridMap.at<uchar>(i,j) = 0;
                            goto label1;
                        }

                        if(this->baseMap->at<uchar>(y,x) <= 200) free = false;
                    }
                }
                if(free) this->gridMap.at<uchar>(i,j) = 1;
                else this->gridMap.at<uchar>(i,j) = 0;
                label1:
                continue;
            }
        }

    }

    void define_pubMarker(ros::Publisher &publisher){
        this->MarkgerPub = &publisher;

        // Base marker definition
        this->marker.header.frame_id = "map";
        this->marker.header.stamp = ros::Time();
        this->marker.ns = "SC_Planner";
        this->marker.id = 0;
        this->marker.type = visualization_msgs::Marker::CUBE_LIST;
        this->marker.action = visualization_msgs::Marker::ADD;
        this->marker.pose.position.x = 0;
        this->marker.pose.position.y = 0;
        this->marker.pose.position.z = 0;
        this->marker.pose.orientation.x = 0.0;
        this->marker.pose.orientation.y = 0.0;
        this->marker.pose.orientation.z = 0.0;
        this->marker.pose.orientation.w = 1.0;
        this->marker.scale.x = this->cell_size_m;
        this->marker.scale.y = this->cell_size_m;
        this->marker.scale.z = 0.01;
        // this->marker.color.a = 1.0; // Don't forget to set the alpha!
        this->marker.lifetime.fromSec(0.15);
    }

    bool validDisplacement(geometry_msgs::Pose pose){
        //TODO


        // float disp = sqrt(msg.pose.pose.position.x^2)
        // msg.pose.pose.pos
        // this->last_pose = 
        return true;
    }

    void detect_neighbors(const grid_cord &current_cell){
        
    }

    bool iterate(const geometry_msgs::Pose &current_pose){
        
        // Hit detection
        static grid_cord temp_gridPose;
        temp_gridPose = this->transform_world_to_grid(current_pose);
        if(this->gridMap.at<uchar>(temp_gridPose.i,temp_gridPose.j) == 1) this->gridMap.at<uchar>(temp_gridPose.i,temp_gridPose.j) = 2;

        // TODO downsampling
        if(!this->initialPose_defined){
            this->last_pose = current_pose;

            this->current_goal = temp_gridPose;//decisionBase();
            this->last_goal = temp_gridPose;
            this->initialPose_defined = true;
        }
        else{
            if(this->gridMap.at<uchar>(this->current_goal.i,this->current_goal.j) == 2){
                static grid_cord temp_last_goal;
                temp_last_goal = this->current_goal;
                this->current_goal = grid_cord(24,14,dir::RIGHT);
                this->last_goal = temp_last_goal;
                this->last_pose = current_pose;


                // Publishing goal
                if(this->MB_client != NULL){
                    move_base_msgs::MoveBaseGoal goal;
                    goal.target_pose.header.frame_id = "map";
                    goal.target_pose.header.stamp = ros::Time::now();

                    goal.target_pose.pose = this->transform_grid_to_world(this->current_goal);
                    printf("Sending Goal!\n");
                    printf("Goal: %f,%f,%f\n",goal.target_pose.pose.position.x,goal.target_pose.pose.position.y,tf::getYaw(goal.target_pose.pose.orientation));
                    printf("Cell Goal: %u,%u,%u\n",this->current_goal.j,this->current_goal.i,this->current_goal.direction);
                    this->MB_client->sendGoal(goal);
                }
                else ROS_WARN("No Move_Base client defined!\n");
            }
        }

        this->pubMarkers();
        return true;
    }

    grid_cord decisionBase(void){
        grid_cord dummy;
        return dummy;
    }

    geometry_msgs::Pose transform_grid_to_world(const grid_cord &cord){
        //TODO introduzir o valor da TF dinamico
        geometry_msgs::Pose out_pose;

        // TF comp
        out_pose.position.x += -1.0;
        out_pose.position.y += -1.0;

        // Center offset
        out_pose.position.x += this->marker.scale.x/2;
        out_pose.position.y += this->marker.scale.y/2;

        // Grid offset
        //BUG O tamanho de celula interfere na componente de overlap de escala
        out_pose.position.x += cord.j*(this->marker.scale.x+this->baseMap_resolution/4); //   /2 para 0.6
        out_pose.position.y += cord.i*(this->marker.scale.y+this->baseMap_resolution/4); // 0.02
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
        temp.position.x -= this->marker.scale.x/2;
        temp.position.y -= this->marker.scale.y/2;

        // Grid offset
        cord_out.j = round(temp.position.x/(this->marker.scale.x+this->baseMap_resolution/4));
        cord_out.i = round(temp.position.y/(this->marker.scale.y+this->baseMap_resolution/4));

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

        const bool is_empty = this->marker.points.empty();

        for(uint16_t i = 0; i < this->gridMap.rows; i++){
            for(uint16_t j = 0; j < this->gridMap.cols; j++){
                pose = this->transform_grid_to_world(grid_cord(i,j));
                pose.position.z = 0.05;
                switch(this->gridMap.at<uchar>(i,j)){
                    case 0:
                        // This case should be empty!
                        // Used to see the complete gridMap on rviz
                        // Remove to filter the cells
                        if(is_empty){
                            this->marker.points.push_back(pose.position);
                            this->marker.colors.push_back(this->c_free);
                        }
                        else this->marker.colors[i*this->gridMap.cols+j] = this->c_free;
                        break;
                    case 1:
                        if(is_empty){
                            this->marker.points.push_back(pose.position);
                            this->marker.colors.push_back(this->c_free);
                        }
                        else this->marker.colors[i*this->gridMap.cols+j] = this->c_free;
                        break;
                    case 2:
                        if(is_empty){
                            this->marker.points.push_back(pose.position);
                            this->marker.colors.push_back(this->c_occ);
                        }
                        else this->marker.colors[i*this->gridMap.cols+j] = this->c_occ;
                        break;
                }
                // if(this->gridMap.at<uchar>(i,j) == 0;
            }
        }
        // this->initialized_markers = true;
        this->MarkgerPub->publish(this->marker);
    }

    void move(void){
        for(uint16_t i = 0; i < this->gridMap.rows; i++){
            for(uint16_t j = 0; j < this->gridMap.cols; j++){

                // this->gridMap.at<uchar>(i,j);
                // pose = this->transform_grid_to_world(grid_cord(i,j));
                // pose.z = 0.05;
                // switch(this->gridMap.at<uchar>(i,j)){
                //     case 0:
                //         // this->marker.points.push_back(pose);
                //         // this->marker.colors.push_back(this->c_free);
                //         break;
                //     case 1:
                //         if(is_empty){
                //             this->marker.points.push_back(pose);
                //             this->marker.colors.push_back(this->c_free);
                //         }
                //         else this->marker.colors[i*this->gridMap.cols+j] = this->c_free;
                //         break;
                //     case 2:
                //         if(is_empty){
                //             this->marker.points.push_back(pose);
                //             this->marker.colors.push_back(this->c_occ);
                //         }
                //         else this->marker.colors[i*this->gridMap.cols+j] = this->c_occ;
                //         break;
                // }
                // if(this->gridMap.at<uchar>(i,j) == 0;
            }
        }
    }

};