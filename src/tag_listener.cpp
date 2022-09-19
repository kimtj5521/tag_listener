#include <ros/ros.h>
#include <tf/transform_listener.h>
#include <iostream>
#include <std_msgs/Bool.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Pose.h>
#include <string>

class tagListener
{
public:
    tagListener();
    ~tagListener();

    void set_param();

    void run();

    double offset_x;
    double offset_y;
    std::string camera_frame_id;
    std::string tag_frame_id;
    std::string map_frame_id;
    bool fixed_frame_flag;

private:
    ros::NodeHandle nh;
    ros::NodeHandle private_nh;

    tf::TransformListener listener;

    ros::Publisher goal_pose_pub;
    ros::Publisher tag_detecting_flag_pub;

    std_msgs::Bool b_tag_detecting;

    double prev_x, curr_x;

    geometry_msgs::PoseStamped goal_pose_msg;
};

tagListener::tagListener()
{
    ROS_INFO("\033[1;32m---->\033[0m Apriltag tf listener started.");

    set_param();

    goal_pose_pub = nh.advertise<geometry_msgs::PoseStamped>("/Docking/goal_pose", 1);
    tag_detecting_flag_pub = nh.advertise<std_msgs::Bool>("/Docking/tag_detecting_flag",1);
}

tagListener::~tagListener()
{
    ROS_INFO("\033[1;32m---->\033[0m Apriltag tf listener is destroyed");
}

void tagListener::set_param()
{
    nh.getParam("/offset_x", offset_x);
    nh.getParam("/offset_y", offset_y);

    nh.param<std::string>("/camera_frame", camera_frame_id, "camera_link");
    nh.param<std::string>("/tag_frame", tag_frame_id, "tag_0");
    nh.param<std::string>("/map_frame", map_frame_id, "map");
    nh.param<bool>("fixed_frame_flag", fixed_frame_flag, false);
}

void tagListener::run()
{
    tf::StampedTransform transform;
    try{
        listener.lookupTransform(camera_frame_id, tag_frame_id,
                                ros::Time(0), transform);
    }
    catch(tf::TransformException ex){
        ROS_ERROR("%s",ex.what());
        // ros::Duration(1.0).sleep();
    }

    curr_x = transform.getOrigin().x();
    // std::cout << "curr_x : " << curr_x << std::endl;
    // std::cout << "prev_x : " << prev_x<< std::endl;


    if(curr_x != prev_x){
        b_tag_detecting.data = true;
        // std::cout << "1 "<< std::endl;
    }
    else{
        b_tag_detecting.data = false;
        // std::cout << "0"<< std::endl;
    }

    tag_detecting_flag_pub.publish(b_tag_detecting);

    prev_x = curr_x;

    if(b_tag_detecting.data == true){
        // std::cout << "test " << std::endl;

        if( fixed_frame_flag == false){
            goal_pose_msg.header.frame_id = map_frame_id;
        }else{
            goal_pose_msg.header.frame_id = camera_frame_id;
        }
        goal_pose_msg.header.stamp = ros::Time::now();

        goal_pose_msg.pose.position.x = transform.getOrigin().x() + offset_x;
        goal_pose_msg.pose.position.y = transform.getOrigin().y() + offset_y;
        goal_pose_msg.pose.position.z = 0.0;

        goal_pose_msg.pose.orientation.x = 0.0;
        goal_pose_msg.pose.orientation.y = 0.0;
        goal_pose_msg.pose.orientation.z = 0.0;
        goal_pose_msg.pose.orientation.w = 1.0;


        goal_pose_pub.publish(goal_pose_msg);
    }
}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "tag_listener");

    tagListener tag_listener;

    ros::Rate loop_rate(20);
    while(ros::ok())
    {
        ros::spinOnce();
        tag_listener.run();
        loop_rate.sleep();
    }

    return 0;
}