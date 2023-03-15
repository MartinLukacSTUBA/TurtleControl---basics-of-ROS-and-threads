#ifndef PROJECT_TURTLECONTROL_H
#define PROJECT_TURTLECONTROL_H

// Include ros base
#include <ros/ros.h>

//Include ros msgs
#include <geometry_msgs/Twist.h>
#include <turtlesim/Pose.h>
#include <turtlesim/TeleportAbsolute.h>
#include <turtlesim/SetPen.h>
#include <rrm_cv1_lukac/Draw.h>
#include <rrm_cv1_lukac/Stop.h>


//main class for turtle control
class TurtleControl {

public:
    //Constructor
    TurtleControl();

    //Callbacks
    bool drawCallback(rrm_cv1_lukac::Draw::Request &req, rrm_cv1_lukac::Draw::Response &res);
    bool stopCallback(rrm_cv1_lukac::Stop::Request &req_stop, rrm_cv1_lukac::Stop::Response &res_stop);
    void poseCallback(const turtlesim::Pose::ConstPtr& msg);

    // Other public methods
    void publish();
    double getRate();
    bool getDrawingStatus();

private:

    // Consts
    const double PUB_RATE = 100.0;
    const double  WINDOW_CENTER = 5.544444561;
    const double WINDOW_EDGE = 11.088889122;

    // member variables
    geometry_msgs::Twist velocity_msg_;
    bool drawing_status_;
    bool first_draw;
    turtlesim::Pose pose_msg_;

    //ROS communication tools
    ros::Publisher velocity_pub_;
    ros::Subscriber pose_sub_;
    ros::ServiceServer draw_service_;
    ros::ServiceServer stop_service_;
    ros::ServiceClient teleport_client_;
    ros::ServiceClient clear_client_;
    ros::ServiceClient pen_client;

};


#endif //PROJECT_TURTLECONTROL_H