
enum SC_CELL_TYPE{OCC = 1,FREE = 0};

struct grid_cord{
    uint16_t i; // Height
    uint16_t j; // Width
    grid_cord(void){
        this->i = 0;
        this->j = 0;
    }
    grid_cord(uint16_t i_,uint16_t j_){
        this->i = i_;
        this->j = j_;
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
    ros::Publisher *pub; // Will publish if defined
    visualization_msgs::Marker marker;


    // Cell_colors
    std_msgs::ColorRGBA c_occ;
    std_msgs::ColorRGBA c_free;
    bool initialized_markers;
    unsigned int mapExtremes[4];


    geometry_msgs::Point last_pose;
    grid_cord gridPose;
    bool initialPose_defined = false;
    grid_cord current_goal;

    public:
    SC_planner(float cell_m){
        this->cell_size_m = cell_m;

        this->cell_size_pix = 0;

        this->pub = NULL; 

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
        this->pub = &publisher;

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

    bool validDisplacement(geometry_msgs::Point pose){
        //TODO


        // float disp = sqrt(msg.pose.pose.position.x^2)
        // msg.pose.pose.pos
        // this->last_pose = 
        return true;
    }

    bool iterate(const geometry_msgs::Point &current_pose){

        // Hit detection
        static grid_cord temp_gridPose;
        temp_gridPose = this->transform_world_to_grid(current_pose);
        if(this->gridMap.at<uchar>(temp_gridPose.i,temp_gridPose.j) == 1)
            this->gridMap.at<uchar>(temp_gridPose.i,temp_gridPose.j) = 2;


        if(!this->initialPose_defined){
            // TODO downsampling

            this->last_pose = current_pose;

            current_goal = decisionBase();



            this->initialPose_defined = true;
        }
        else{

        }
        // if(!this->initialized_markers){
        // }
        this->pubMarkers();
        return true;
    }

    grid_cord decisionBase(void){

    }

    geometry_msgs::Point transform_grid_to_world(const grid_cord &cord){
        //TODO introduzir o valor da TF dinamico
        geometry_msgs::Point out_pose;

        // TF comp
        out_pose.x += -1.0;
        out_pose.y += -1.0;

        // Center offset
        out_pose.x += this->marker.scale.x/2;
        out_pose.y += this->marker.scale.y/2;

        // Grid offset
        //BUG O tamanho de celula interfere na componente de overlap de escala
        out_pose.x += cord.j*(this->marker.scale.x+this->baseMap_resolution/4); //   /2 para 0.6
        out_pose.y += cord.i*(this->marker.scale.y+this->baseMap_resolution/4); // 0.02
        return out_pose;
    }

    grid_cord transform_world_to_grid(const geometry_msgs::Point &point){
        //TODO introduzir o valor da TF dinamico
        geometry_msgs::Point temp = point;
        grid_cord cord_out = grid_cord();

        // TF comp
        temp.x -= -1.0;
        temp.y -= -1.0;

        // Center offset
        temp.x -= this->marker.scale.x/2;
        temp.y -= this->marker.scale.y/2;

        // Grid offset
        cord_out.j = round(temp.x/(this->marker.scale.x+this->baseMap_resolution/4));
        cord_out.i = round(temp.y/(this->marker.scale.y+this->baseMap_resolution/4));

        if(cord_out.j >= this->gridMap.cols) cord_out.j = this->gridMap.cols;
        if(cord_out.i >= this->gridMap.rows) cord_out.i = this->gridMap.rows;
        // out_pose.x += j*this->marker.scale.x;
        // out_pose.y += i*this->marker.scale.y;
        printf("world_to_grid: (%i, %i)\n", cord_out.j, cord_out.i);
        return cord_out;
    }

    private:

    void pubMarkers(void){
        geometry_msgs::Point pose;

        const bool is_empty = this->marker.points.empty();

        for(uint16_t i = 0; i < this->gridMap.rows; i++){
            for(uint16_t j = 0; j < this->gridMap.cols; j++){
                pose = this->transform_grid_to_world(grid_cord(i,j));
                pose.z = 0.05;
                switch(this->gridMap.at<uchar>(i,j)){
                    case 0:
                        // This case should be empty!
                        // Used to see the complete gridMap on rviz
                        if(is_empty){
                            this->marker.points.push_back(pose);
                            this->marker.colors.push_back(this->c_free);
                        }
                        else this->marker.colors[i*this->gridMap.cols+j] = this->c_free;
                        break;
                    case 1:
                        if(is_empty){
                            this->marker.points.push_back(pose);
                            this->marker.colors.push_back(this->c_free);
                        }
                        else this->marker.colors[i*this->gridMap.cols+j] = this->c_free;
                        break;
                    case 2:
                        if(is_empty){
                            this->marker.points.push_back(pose);
                            this->marker.colors.push_back(this->c_occ);
                        }
                        else this->marker.colors[i*this->gridMap.cols+j] = this->c_occ;
                        break;
                }
                // if(this->gridMap.at<uchar>(i,j) == 0;
            }
        }
        // this->initialized_markers = true;
        this->pub->publish(this->marker);
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