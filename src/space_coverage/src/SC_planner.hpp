#include <iostream>


class SC_planner{

    private:
    float cell_size;
    int map; //temp

    geometry_msgs::Point last_pose;




    public:
    SC_planner(){
        this->cell_size = 1;
    };
    ~SC_planner(){

    };


    bool validDisplacement(geometry_msgs::Point pose){
        //TODO


        // float disp = sqrt(msg.pose.pose.position.x^2)
        // msg.pose.pose.pos
        // this->last_pose = 
        return true;
    }


    bool iterate(const geometry_msgs::Point &current_pose,geometry_msgs::Point &new_pose){
        //TODO
        return true;
    }
};