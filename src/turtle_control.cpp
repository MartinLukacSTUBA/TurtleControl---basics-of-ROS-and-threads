#include "rrm_cv1_lukac/turtle_control.h"
#include "std_srvs/Empty.h"

TurtleControl::TurtleControl(){

    // NodeHandler
    ros::NodeHandle n;

    // Publisher
    velocity_pub_ = n.advertise<geometry_msgs::Twist>("turtle1/cmd_vel", 1);

    // Subscriber
    pose_sub_ = n.subscribe("turtle1/pose", 1, &TurtleControl::poseCallback, this);

    // Service server
    draw_service_ = n.advertiseService("/turtle_control/draw", &TurtleControl::drawCallback, this);
    stop_service_ = n.advertiseService("/turtle_control/stop", &TurtleControl::stopCallback,this);

    // Service client
    teleport_client_ = n.serviceClient<turtlesim::TeleportAbsolute>("/turtle1/teleport_absolute");
    pen_client = n.serviceClient<turtlesim::SetPen>("turtle1/set_pen");
    clear_client_ = n.serviceClient<std_srvs::Empty>("/clear");


    //init variables
    velocity_msg_ = geometry_msgs::Twist();

    this->first_draw = true;

    //variable init
    this->drawing_status_ = false;
    this->pose_msg_ = turtlesim::Pose();
}

// service server callback for starting the drawing and drawing speed configuration
bool TurtleControl::drawCallback(rrm_cv1_lukac::Draw::Request &req, rrm_cv1_lukac::Draw::Response &res)
{
    int width = 1;
    int r = 0;
    int g = 0;
    int b = 0;
    ros::NodeHandle n1;

    n1.getParam("/turtle_control/line/width", width);
    n1.getParam("/turtle_control/line/r", r);
    n1.getParam("/turtle_control/line/g", g);
    n1.getParam("/turtle_control/line/b", b);

    ROS_INFO("PARAMETRE SIRKA %d \n", width);
    ROS_INFO("PARAMETRE r %d \n", r);
    ROS_INFO("PARAMETRE g %d \n", g);
    ROS_INFO("PARAMETRE b %d \n", b);


    if(first_draw) {


        turtlesim::SetPen first_setpen_srv;
        first_setpen_srv.request.width = width;
        first_setpen_srv.request.r = r;
        first_setpen_srv.request.g = g;
        first_setpen_srv.request.b = b;
        first_setpen_srv.request.off = false;

        if (pen_client.call(first_setpen_srv))ROS_INFO("Hrubka ciary nastavena");
        velocity_msg_.linear.x = 2;
        this->first_draw = false;
    }
        velocity_msg_.angular.z=(2.0)/req.radius;
        ROS_INFO("RADIUS %d",req.radius);

        this->drawing_status_=true;
        res.success = true;
        return true;
}

bool TurtleControl::stopCallback(rrm_cv1_lukac::Stop::Request &req_stop,rrm_cv1_lukac::Stop::Response &res_stop)
{
    ROS_WARN("TURTLE STOPCALLBACK");
    if(req_stop.stop){
        this->drawing_status_ = false;
        ROS_WARN("TURTLE STOPPED");
    }else{
        this->drawing_status_ = true;
        ROS_WARN("TURTLE RUNNING");
    }
    res_stop.success = true;
    return true;
}

// topic callback a for listening to the pose message from the turtle
void TurtleControl::poseCallback(const turtlesim::Pose::ConstPtr& msg)
{
    if ((msg->x >= WINDOW_EDGE)||(msg->y >= WINDOW_EDGE)||(msg->x <= 0)||(msg->y <= 0))
    {

        turtlesim::TeleportAbsolute teleport_srv;
        teleport_srv.request.theta = 0;
        teleport_srv.request.x = WINDOW_CENTER;
        teleport_srv.request.y = WINDOW_CENTER;
        teleport_client_.call(teleport_srv);

        std_srvs::Empty clear_srv;
        clear_client_.call(clear_srv);
    }
    this->pose_msg_ = *msg;
}

// publishing the configured velocity
void TurtleControl::publish()
{
    velocity_pub_.publish(velocity_msg_);
}

//frequency setup
double TurtleControl::getRate()
{
    return PUB_RATE;
}

//getter if drawing is enabled or not
bool TurtleControl::getDrawingStatus()
{
    return this->drawing_status_;
}



