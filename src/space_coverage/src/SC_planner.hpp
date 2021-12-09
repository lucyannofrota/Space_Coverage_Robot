// #include <iostream>
// #include <nav_msgs/

enum SC_CELL_TYPE{OCC = 1,FREE = 0};
// struct id_ref{
//     int32_t id;
//     int16_t row;
//     int16_t col;
// };

class SC_planner{

    private:
    float cell_size_m;
    uint16_t cell_size_pix;
    cv::Mat_<uint8_t> map;

    ros::Publisher *pub; // Will publish if defined
    visualization_msgs::Marker marker;

    geometry_msgs::Point last_pose;

    std_msgs::ColorRGBA c_occ;
    std_msgs::ColorRGBA c_free;

    bool initialized_markers;

    // id_ref marker_id;

    public:
    SC_planner(float cell_m){
        this->cell_size_m = cell_m;

        this->cell_size_pix = 0;

        // this->map = cv::Mat::zeros(ceil(msg.info.height/cell_size_pix),ceil(msg.info.width/cell_size_pix),CV_8UC1);

        this->pub = NULL; 

        this->c_occ.a = 1.0;
        this->c_occ.r = 0.5;
        this->c_occ.g = 1.0;
        this->c_occ.b = 0.5;

        this->c_free = this->c_occ;
        this->c_free.g = 0.5;

        this->initialized_markers = false;
    }
    /*SC_planner(float cell_m,const nav_msgs::OccupancyGrid& msg, const cv::Mat_<uint8_t> *base_map, ros::Publisher &publisher){
        this->cell_size_m = cell_m;
        // std::cout << "Csize: " << this->cell_size_m << std::endl;
        // std::cout << "Resolution: " << msg.info.resolution << std::endl;
        this->cell_size_pix = ceil(this->cell_size_m*(1/msg.info.resolution));

        // this->map = new 

        // cv::Size()
        // std::cout << "Size: " << floor(msg.info.height/this->cell_size_pix) << std::endl;
        std::cout << "Cell_size_pix: " << this->cell_size_pix << std::endl;
        std::cout << "H: " << floor(msg.info.height/cell_size_pix) << std::endl;
        std::cout << "W: " << floor(msg.info.width/cell_size_pix) << std::endl;

        this->map = cv::Mat::zeros(ceil(msg.info.height/cell_size_pix),ceil(msg.info.width/cell_size_pix),CV_8UC1);

        std::cout << "Starting SC_Planner" << std::endl;
        std::cout << this->map.size() << std::endl;

        this->pub = NULL;
    };*/
    ~SC_planner(){
        
    };

    uint16_t get_cell_size_pix(void){
        return this->cell_size_pix;
    }

    void setup_planner(const nav_msgs::OccupancyGrid& msg/*const cv::Mat_<uint8_t> *base_map*/){

        this->cell_size_pix = ceil(this->cell_size_m*(1/msg.info.resolution));

        this->map = cv::Mat::zeros(ceil(msg.info.height/cell_size_pix),ceil(msg.info.width/cell_size_pix),CV_8UC1);

        // // cv::MatSize temp = this->map.size;
        // // this->map.
        
        // // (this->map).rows()

        // std::cout << "Map h: " << this->map.rows << std::endl;
        // std::cout << "Map w: " << this->map.cols << std::endl;
    }

    void define_pub(ros::Publisher &publisher){
        this->pub = &publisher;

        
        this->marker.header.frame_id = "map";
        this->marker.header.stamp = ros::Time();
        this->marker.ns = "SC_Planner";
        this->marker.id = 0;
        this->marker.type = visualization_msgs::Marker::CUBE;
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
        this->marker.scale.z = 0.001;
        this->marker.color.a = 1; // Don't forget to set the alpha!
        this->marker.color.r = 1.0;
        this->marker.color.g = 0.0;
        this->marker.color.b = 0.0;
        // this->marker.lifetime.fromSec(0.15);
    }


    bool validDisplacement(geometry_msgs::Point pose){
        //TODO


        // float disp = sqrt(msg.pose.pose.position.x^2)
        // msg.pose.pose.pos
        // this->last_pose = 
        return true;
    }


    bool iterate(const geometry_msgs::Point &current_pose,geometry_msgs::Point &new_pose){
        //TODO
        if(!this->initialized_markers){
            this->initialize_markers();
        }
        return true;
    }


    void transform_grid_to_map(void){
        //TODO
    }

    void setMarker(bool new_marker, int32_t id, SC_CELL_TYPE type, geometry_msgs::Pose &pose){
        if(new_marker) this->marker.action = visualization_msgs::Marker::ADD;
        else this->marker.action = visualization_msgs::Marker::MODIFY;
        this->marker.id = id;
        if(type) this->marker.color = this->c_occ;
        else this->marker.color = this->c_free;
        this->marker.pose = pose;
    }


    void initialize_markers(void){
        geometry_msgs::Pose pose;
        for(uint16_t i = 0; i < this->map.rows; i++){
            for(uint16_t j = 0; j < this->map.cols; j++){
                // TODO set markers positons
                this->setMarker(true,this->id_ref(i,j),SC_CELL_TYPE::FREE,pose); // ID's limitados a mapas pequenos
                this->pub->publish(this->marker);
            }
        }
        this->initialized_markers = true;
    }

    int32_t id_ref(uint16_t i, uint16_t j){
        return i*10000+j;
    }

    // void createMarker(void){
    //     visualization_msgs::Marker marker;
    //     marker.header.frame_id = "map";
    //     marker.header.stamp = ros::Time();
    //     marker.ns = "SC_Planner";
    //     marker.id = id;
    //     marker.type = visualization_msgs::Marker::CUBE;
    //     marker.action = visualization_msgs::Marker::ADD;
    //     marker.pose.position.x = 0;
    //     marker.pose.position.y = 0;
    //     marker.pose.position.z = 0;
    //     marker.pose.orientation.x = 0.0;
    //     marker.pose.orientation.y = 0.0;
    //     marker.pose.orientation.z = 0.0;
    //     marker.pose.orientation.w = 1.0;
    //     marker.scale.x = this->cell_size_m;
    //     marker.scale.y = this->cell_size_m;
    //     marker.scale.z = 0.001;
    //     marker.color.a = 1; // Don't forget to set the alpha!
    //     marker.color.r = 0.0;
    //     marker.color.g = 1.0;
    //     marker.color.b = 0.0;
    //     marker.lifetime.fromSec(0.15);
    //     //only if using a MESH_RESOURCE marker type:
    //     // marker.mesh_resource = "package://pr2_description/meshes/base_v0/base.dae";
    //     // this->pub->publish( marker );
    // }
};