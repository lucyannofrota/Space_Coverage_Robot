#include <tf/tf.h>
#include <list> //For the path list

typedef actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> MoveBaseClient;

int DEBUG = 0;

bool ORIENTATION = true;

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
    void print(void){
        printf("(X: %u,Y: %u)\n", this->j, this->i);
    }
};

class SC_planner{

    private:

    float cell_size_m;
    uint16_t cell_size_pix;
    cv::Mat_<uchar> gridMap; //0 - Invalid Cell, 1 - Free Cell, 2 - Occ Cell
    cv::Mat_<uchar> img_Map;
    cv::Mat_<uchar> *baseMap;
    uint16_t l_side;
    geometry_msgs::Pose gridMap_pose;
    float baseMap_resolution;
    bool grid_initialized = false;
    bool setup_done = false;

    // Mekers_Publisher
    ros::Publisher *MarkerPub; // Will publish if defined
    visualization_msgs::Marker path_cell_marker;
    visualization_msgs::Marker invalid_cell_marker;
    visualization_msgs::Marker path_marker;


    // Cell_colors
    std_msgs::ColorRGBA c_occ;
    std_msgs::ColorRGBA c_free;
    std_msgs::ColorRGBA c_invalid;
    unsigned int mapExtremes[4];


    double map_offset[2];
    grid_cord gridPose;
    grid_cord current_goal;
    double goal_dist;
    bool resend_goal;


    MoveBaseClient *MB_client;

    public:

    bool goal_reached = true;
    SC_planner(MoveBaseClient &MB_client_,float cell_m, double *map_offset_, double goal_dist_){
        this->cell_size_m = cell_m;

        this->cell_size_pix = 0;

        this->map_offset[0] = map_offset_[0];
        this->map_offset[1] = map_offset_[1];

        this->goal_dist = goal_dist_;
        this->resend_goal = false;

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

        this->c_invalid.a = 0.2;
        this->c_invalid.r = 115.0/255.0;
        this->c_invalid.g = 115.0/255.0;
        this->c_invalid.b = 115.0/255.0;
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

        this->l_side = (height >= width ? height : width);

        this->gridMap = cv::Mat::zeros(this->l_side*2+1,this->l_side*2+1,CV_8UC1);
        // this->visitedTimes = cv::Mat::zeros(this->gridMap.rows,this->gridMap.cols,CV_8UC1);

        std::cout << "Cell Size: " << this->cell_size_pix << std::endl;
        std::cout << "Map h: " << this->gridMap.rows << std::endl;
        std::cout << "Map w: " << this->gridMap.cols << std::endl;

        this->mapExtremes[0] = mapExtremes[0];
        this->mapExtremes[1] = mapExtremes[1];
        this->mapExtremes[2] = mapExtremes[2];
        this->mapExtremes[3] = mapExtremes[3];

        this->setup_done = true;
    }

    void initialize_gridMap(const geometry_msgs::Pose &current_pose){
        using std::cout;
        using std::endl;

        this->gridMap_pose = current_pose;

        // Fill the grid
        // uint16_t l_side = floor(this->gridMap.rows/2); // Total side is 2*l_side+1
        for(int16_t i = -this->l_side; i < this->l_side; i++){
            for(int16_t j = -this->l_side; j < this->l_side; j++){
                bool free = true;
                for(uint16_t cy = 0; cy < this->cell_size_pix; cy++){
                    uint16_t y = cy+i*this->cell_size_pix-this->gridMap_pose.position.x/this->baseMap_resolution;
                    if(y > this->baseMap->rows || y < this->mapExtremes[2] || y > this->mapExtremes[3]){
                        this->gridMap.at<uchar>(i+this->l_side,j+this->l_side) = SC_CELL_TYPE::INVALID;
                        goto label1;
                    }
                    for(uint16_t cx = 0; cx < this->cell_size_pix; cx++){
                        uint16_t x = cx+j*this->cell_size_pix-this->gridMap_pose.position.y/this->baseMap_resolution;
                        if(x > this->baseMap->cols || x < this->mapExtremes[0] || x > this->mapExtremes[1]){
                            this->gridMap.at<uchar>(i+this->l_side,j+this->l_side) = SC_CELL_TYPE::INVALID;
                            goto label1;
                        }

                        if(this->baseMap->at<uchar>(y,x) <= 200) free = false;
                    }
                }
                if(free) this->gridMap.at<uchar>(i+this->l_side,j+this->l_side) = SC_CELL_TYPE::FREE;
                else this->gridMap.at<uchar>(i+this->l_side,j+this->l_side) = SC_CELL_TYPE::INVALID;
                label1:
                continue;
            }
        }

        // img_Map
        cv::resize(this->gridMap, this->img_Map, cv::Size(), 6, 6);
        // resize(image, scaled_f_up, Size(), scale_up_x, scale_up_y, INTER_LINEAR);
        std::cout << "Initializing_Rviz_Markers" << std::endl;
        this->define_rviz_Markers();
        this->grid_initialized = true;
    }

    void set_rviz_handle(ros::Publisher &publisher){
        this->MarkerPub = &publisher;
    }

    bool iterate(const geometry_msgs::Pose &current_pose){
        // this->pubMarkers();
        // return false;
        using std::cout;
        using std::endl;

        if(!this->setup_done) return false;
        if(!grid_initialized) this->initialize_gridMap(current_pose);

        // cv::imshow("Space Coverage Planner", this->img_Map);
        // cv::waitKey(1);

        this->pubMarkers();

        return true;

        // Hit detection
        static grid_cord temp_gridPose;
        printf("Current Pose: (%f,%f)\n",current_pose.position.x,current_pose.position.y);
        temp_gridPose = this->transform_world_to_grid(current_pose);

        if (DEBUG == 1)
            printf("GridPose: (%u,%u,%u,%u)\n",temp_gridPose.j,temp_gridPose.i,temp_gridPose.direction,this->gridMap.at<uchar>(temp_gridPose.i,temp_gridPose.j));

        static std::list <grid_cord> full_path;

        if(full_path.empty()){

            cout << "Creating New Path" << endl;

            // this->last_goal = transform_world_to_grid(current_pose);

            cv::Mat_<uchar> temp_gridMap;
            this->gridMap.copyTo(temp_gridMap);
            cout << "Current Grid Pose: "; temp_gridPose.print();
            spiralSTC(temp_gridMap,temp_gridPose,full_path);

            // geometry_msgs::Point pt = (this->transform_grid_to_world(this->transform_world_to_grid(current_pose))).position;
            if(!full_path.empty()){
                this->path_marker.points.push_back((this->transform_grid_to_world(this->transform_world_to_grid(current_pose))).position);

                for(auto itr = full_path.begin(); itr != full_path.end(); itr++){
                    geometry_msgs::Pose tempP = transform_grid_to_world(*itr);
                    this->path_marker.points.push_back(tempP.position);
                }

                // this->gridMap.at<uchar>(temp_gridPose.i,temp_gridPose.j) = SC_CELL_TYPE::OCC;

                this->current_goal = (*(full_path.begin())); full_path.pop_front();

                cout << "Done!" << endl;

                this->goal_pub(current_pose);
            }
        }
        else{
            if(!this->resend_goal){
                bool hit = this->hit_detect(current_pose);


                if(hit || this->goal_reached){

                    printf("Hit: %u, gr: %u\n",hit,this->goal_reached);

                    this->gridMap.at<uchar>(this->current_goal.i,this->current_goal.j) = SC_CELL_TYPE::OCC;

                    // this->current_goal = spiralSTC_online(temp_gridPose);
                    // this->current_goal.direction = temp_gridPose.direction;

                    // full_path.


                    // this->last_goal = this->current_goal;

                    this->current_goal = (*(full_path.begin())); full_path.pop_front();


                    this->goal_pub(current_pose);
                    
                }
            }
            else{
                this->goal_pub(current_pose);
            }
        }

        

        return true;
    }


    private:

    // Goal

    void goal_pub(const geometry_msgs::Pose &current_pose){
        // Publishing goal
        static move_base_msgs::MoveBaseGoal goal;
        goal.target_pose.header.frame_id = "map";
        goal.target_pose.header.stamp = ros::Time::now();
        if(this->MB_client != NULL){
            if(!this->resend_goal){
                goal.target_pose.pose = this->transform_grid_to_world(this->current_goal);
                if(ORIENTATION) goal.target_pose.pose.orientation = current_pose.orientation;
            }
            printf("Sending Goal!\n");
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
        else{
            this->goal_reached = false;
            this->resend_goal = true;
            ROS_INFO("Goal failed");
        }
    }

    // STC

    bool decisionBase(const grid_cord &actual_pose, grid_cord &next_pose, bool *validCells){
        // return true if it got stuck

        std::cout << "Decision Base" << std::endl;
        printf("Actual Grid Pose: (%u,%u)\n",actual_pose.j,actual_pose.i);

        // grid_cord next_pose;
        next_pose = actual_pose;

        if (validCells[0] == true)
        {
            next_pose= grid_cord(actual_pose.i,actual_pose.j-1,dir::LEFT);
            next_pose.print();
            return false;
        }
        else if (validCells[1] == true){
            next_pose = grid_cord(actual_pose.i-1,actual_pose.j,dir::DOWN);
            next_pose.print();
            return false;
        }
        else if (validCells[2] == true){
            next_pose = grid_cord(actual_pose.i,actual_pose.j+1,dir::RIGHT);
            next_pose.print();
            return false;
        }
        else if (validCells[3] == true){
            next_pose = grid_cord(actual_pose.i+1,actual_pose.j,dir::UP);
            next_pose.print();
            return false;
        }
        else
            return true;
    }

    bool has_free_neighbors(bool *validCells){
        uint8_t counter = 0;
        for(uint8_t i = 0; i < 4; i++){
            counter += validCells[i] ? 1 : 0;
        }
        return counter != 0;
    }

    void detect_neighbors(cv::Mat_<uchar> &gridMap, const grid_cord &current_cell, bool *validCells){
        printf("Hit Detect Current Cell: (%u,%u)\n",current_cell.j,current_cell.i);
        if(gridMap.at<uchar>(current_cell.i,current_cell.j-1) == SC_CELL_TYPE::FREE)
            validCells[0] = true;
        else
            validCells[0] = false; 

        if(gridMap.at<uchar>(current_cell.i-1,current_cell.j) == SC_CELL_TYPE::FREE)
            validCells[1] = true;
        else
            validCells[1] = false;

        if(gridMap.at<uchar>(current_cell.i,current_cell.j+1) == SC_CELL_TYPE::FREE)
            validCells[2] = true;
        else
            validCells[2] = false;
        
        if(gridMap.at<uchar>(current_cell.i+1,current_cell.j) == SC_CELL_TYPE::FREE)
            validCells[3] = true;
        else
            validCells[3] = false; 
    }

    bool hit_detect(const geometry_msgs::Pose &current_pose){
        // const float goal_dist = 0.85;
        static grid_cord temp_gridPose;
        temp_gridPose = this->transform_world_to_grid(current_pose);
        if(this->gridMap.at<uchar>(temp_gridPose.i,temp_gridPose.j) == SC_CELL_TYPE::FREE){
            static geometry_msgs::Pose temp_worldPose;
            temp_worldPose = this->transform_grid_to_world(temp_gridPose);
            // return true;
            static float dx,dy;
            dx = abs(current_pose.position.x-temp_worldPose.position.x);
            dy = abs(current_pose.position.y-temp_worldPose.position.y);
            // if(dx <= 0.7*((this->cell_size_pix*this->baseMap_resolution)/2) && dy <= 0.7*((this->cell_size_pix*this->baseMap_resolution)/2))
            // return sqrt((pow(current_pose.position.x-temp_worldPose.position.x,2)+pow(current_pose.position.y-temp_worldPose.position.y,2))) <= 0.9*((this->cell_size_pix*this->baseMap_resolution)/2);
            return (dx <= this->goal_dist*((this->cell_size_pix*this->baseMap_resolution)/2) && dy <= this->goal_dist*((this->cell_size_pix*this->baseMap_resolution)/2));
        }
        else return false;
    }

    grid_cord spiralSTC_online(const grid_cord &current_gridPose){
        bool validCells[4]; //0 - esquerda 1- baixo ... 
        static std::list <grid_cord> path; //List with the nodes the robot went 
        static grid_cord next_goal;
        this->detect_neighbors(this->gridMap,current_gridPose,validCells);
        if (DEBUG == 1)
        {
            printf("Curr Cell: %u,%u,%u\n",current_gridPose.j,current_gridPose.i,current_gridPose.direction);
            if(validCells[0] == true) printf("Left Cell is valid!\n");
            if(validCells[1] == true) printf("Down Cell is valid!\n");
            if(validCells[2] == true) printf("Right Cell is valid!\n");
            if(validCells[3] == true) printf("Up Cell is valid!\n");
        }
        bool stuck = this->decisionBase(current_gridPose,next_goal,validCells);

        // If the Robot gets stuck
        while(stuck == true)
        {
            path.reverse(); //Reverse the list to get the last nodes
            for(auto itr = path.begin(); itr != path.end(); itr++) //Go through the list
            {
                this->detect_neighbors(this->gridMap,*itr,validCells); //Test the node
                stuck = this->decisionBase(*itr,next_goal,validCells);
                //If the node "unstucks" the robot -> Proceed
                if(stuck == false)
                {
                    path.reverse();
                    break;
                }
            }
        }

        static cv::Mat_<uchar> visitedTimes = cv::Mat::zeros(this->gridMap.rows,this->gridMap.cols,CV_8UC1); //Array with the number of times a cell has been visited

        visitedTimes.at<uchar>(current_gridPose.i,current_gridPose.j) += 1;
        path.push_back(next_goal);
        // if(path.size()>100){
        //     for(int i = 0;i<20;i++){
        //         printf("Clearing the path, for memory management...\n");
        //         path.erase(path.begin());
        //     }
        // }
        return next_goal;
    }

    void spiralSTC(cv::Mat_<uchar> &temp_gridMap, grid_cord current_gridPose, std::list <grid_cord> &path){
        //TODO Criar uma forma de aceitar caminhos divididos
        //TODO Aceitar uma posicao inicial invalida
        bool validCells[4]; //0 - esquerda 1- baixo ... 
        grid_cord next_gridPose;

        printf("current_gridPose: (%u,%u)\n",current_gridPose.j,current_gridPose.i);
        temp_gridMap.at<uchar>(current_gridPose.i,current_gridPose.j) = SC_CELL_TYPE::OCC;

        do{
            this->detect_neighbors(temp_gridMap,current_gridPose,validCells);
            if(this->decisionBase(current_gridPose,next_gridPose,validCells)){
                std::cout << "Stuck" << std::endl;
                return;
            }
            else{
                // next_gridPose.print();
                path.push_back(next_gridPose);
                spiralSTC(temp_gridMap,next_gridPose, path);
            }
        }while(has_free_neighbors(validCells));
    }

    // Transforms

    geometry_msgs::Pose transform_grid_to_world(const grid_cord &cord){
        geometry_msgs::Pose out_pose;

        // TF comp
        out_pose.position.x -= this->gridMap_pose.position.x;
        out_pose.position.y -= this->gridMap_pose.position.y;

        // Center offset
        // out_pose.position.x += this->path_cell_marker.scale.x/2;
        // out_pose.position.y += this->path_cell_marker.scale.y/2;

        // Grid offset
        out_pose.position.x += (cord.j-this->l_side)*(this->cell_size_pix*this->baseMap_resolution); //   /2 para 0.6
        out_pose.position.y += (cord.i-this->l_side)*(this->cell_size_pix*this->baseMap_resolution); // 0.02
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
        geometry_msgs::Pose temp = pose;
        grid_cord cord_out = grid_cord();

        // TF comp
        temp.position.x -= this->gridMap_pose.position.x;
        temp.position.y -= this->gridMap_pose.position.y;

        // Center offset
        temp.position.x -= this->path_cell_marker.scale.x/2;
        temp.position.y -= this->path_cell_marker.scale.y/2;

        // Grid offset
        cord_out.j = round(temp.position.x/(this->cell_size_pix*this->baseMap_resolution));
        cord_out.i = round(temp.position.y/(this->cell_size_pix*this->baseMap_resolution));

        if(cord_out.j >= this->gridMap.cols) cord_out.j = this->gridMap.cols;
        if(cord_out.i >= this->gridMap.rows) cord_out.i = this->gridMap.rows;
        
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

    // Markers

    void define_rviz_Markers(void){
        const float spacing = 0.01;

        ros::Duration marker_lifetime_in_sec(0.15);
    

        // Base marker definition
        this->path_cell_marker.header.frame_id = "map";
        this->path_cell_marker.header.stamp = ros::Time();
        this->path_cell_marker.ns = "Path Cells";
        this->path_cell_marker.id = 0;
        this->path_cell_marker.type = visualization_msgs::Marker::CUBE_LIST;
        this->path_cell_marker.action = visualization_msgs::Marker::ADD;
        this->path_cell_marker.pose.position.x = 0;
        this->path_cell_marker.pose.position.y = 0;
        this->path_cell_marker.pose.position.z = 0;
        this->path_cell_marker.pose.orientation.x = 0.0;
        this->path_cell_marker.pose.orientation.y = 0.0;
        this->path_cell_marker.pose.orientation.z = 0.0;
        this->path_cell_marker.pose.orientation.w = 1.0;
        this->path_cell_marker.scale.x = this->cell_size_pix*this->baseMap_resolution-spacing;
        printf("Cell_size: %f\n",this->cell_size_pix*this->baseMap_resolution-spacing);
        this->path_cell_marker.scale.y = this->cell_size_pix*this->baseMap_resolution-spacing;
        this->path_cell_marker.scale.z = 0.01;
        // this->marker.color.a = 1.0; // Don't forget to set the alpha!
        this->path_cell_marker.lifetime = marker_lifetime_in_sec;

        // Base marker definition
        this->invalid_cell_marker.header.frame_id = "map";
        this->invalid_cell_marker.header.stamp = ros::Time();
        this->invalid_cell_marker.ns = "Invalid Cells";
        this->invalid_cell_marker.id = 1;
        this->invalid_cell_marker.type = visualization_msgs::Marker::CUBE_LIST;
        this->invalid_cell_marker.action = visualization_msgs::Marker::ADD;
        this->invalid_cell_marker.pose.position.x = 0;
        this->invalid_cell_marker.pose.position.y = 0;
        this->invalid_cell_marker.pose.position.z = 0;
        this->invalid_cell_marker.pose.orientation.x = 0.0;
        this->invalid_cell_marker.pose.orientation.y = 0.0;
        this->invalid_cell_marker.pose.orientation.z = 0.0;
        this->invalid_cell_marker.pose.orientation.w = 1.0;
        this->invalid_cell_marker.scale.x = this->cell_size_pix*this->baseMap_resolution-spacing;
        this->invalid_cell_marker.scale.y = this->cell_size_pix*this->baseMap_resolution-spacing;
        this->invalid_cell_marker.scale.z = 0.01;
        // this->marker.color.a = 1.0; // Don't forget to set the alpha!
        // this->invalid_cell_marker.lifetime = 0;


        this->path_marker.header.frame_id = "map";
        this->path_marker.header.stamp = ros::Time();
        this->path_marker.ns = "Planner Path";
        this->path_marker.id = 2;
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
        this->path_marker.lifetime = marker_lifetime_in_sec;

    }

    void pubMarkers(void){

        static ros::Time last_tstamp = ros::Time::now();

        if(ros::Time::now() - last_tstamp < this->path_cell_marker.lifetime*0.6) return;
            
        geometry_msgs::Pose pose;

        const bool is_empty = this->path_cell_marker.points.empty() || this->invalid_cell_marker.points.empty();

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
                        if(is_empty){
                            this->invalid_cell_marker.points.push_back(pose.position);
                            this->invalid_cell_marker.colors.push_back(this->c_invalid);
                        }
                        // else this->invalid_cell_marker.colors[i*this->gridMap.cols+j] = this->c_invalid;
                        idx++;
                        break;
                    case SC_CELL_TYPE::FREE:
                        if(is_empty){
                            this->path_cell_marker.points.push_back(pose.position);
                            this->path_cell_marker.colors.push_back(this->c_free);
                        }
                        else this->path_cell_marker.colors[idx] = this->c_free;
                        idx++;
                        break;
                    case SC_CELL_TYPE::OCC:
                        if(is_empty){
                            this->path_cell_marker.points.push_back(pose.position);
                            this->path_cell_marker.colors.push_back(this->c_occ);
                        }
                        else this->path_cell_marker.colors[idx] = this->c_occ;
                        idx++;
                        break;
                }
            }
        }
        if(this->path_cell_marker.action == visualization_msgs::Marker::ADD || 
        //    this->invalid_cell_marker.action == visualization_msgs::Marker::ADD ||
           this->path_marker.action == visualization_msgs::Marker::ADD){
            this->path_cell_marker.action = visualization_msgs::Marker::MODIFY;
            // this->invalid_cell_marker.action = visualization_msgs::Marker::MODIFY;
            this->path_marker.action = visualization_msgs::Marker::MODIFY;
        }
        last_tstamp = ros::Time::now();
        this->MarkerPub->publish(this->path_cell_marker);
        this->MarkerPub->publish(this->path_marker);

        static bool ivalidPub = false;
        if(!ivalidPub){
            ivalidPub = true;
            this->MarkerPub->publish(this->invalid_cell_marker);
        }
    }

};