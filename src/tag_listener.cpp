#include <ros/ros.h>
#include <tf/transform_listener.h>
#include <iostream>
#include <std_msgs/Bool.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Pose.h>
#include <string>
#include <apriltag_ros/AprilTagDetection.h>
#include <apriltag_ros/AprilTagDetectionArray.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <std_msgs/Int8.h>

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

    void callbackTagDetectInfo(const apriltag_ros::AprilTagDetectionArray::ConstPtr &msg);
    void callbackRearCameraTagDetectInfo(const apriltag_ros::AprilTagDetectionArray::ConstPtr &msg);
    std::string m_tag_detect_info_topic_name;
    std::string m_rear_camera_tag_detect_info_topic_name;

private:
    ros::NodeHandle nh;
    ros::NodeHandle private_nh;

    tf::TransformListener listener;

    ros::Publisher first_goal_pose_pub;
    ros::Publisher second_goal_pose_pub;
    ros::Publisher big_tag_detecting_flag_pub;
    ros::Publisher small_tag_detecting_flag_pub;

    ros::Subscriber tag_detect_info_sub;
    ros::Subscriber rear_camera_tag_detect_info_sub;

    std_msgs::Bool b_big_tag_detecting_flag;
    std_msgs::Bool b_small_tag_detecting_flag;

    double front_cam_big_tag_prev_x, front_cam_big_tag_curr_x, front_cam_small_tag_prev_x, front_cam_small_tag_curr_x;
    double rear_cam_big_tag_prev_x, rear_cam_big_tag_curr_x, rear_cam_small_tag_prev_x, rear_cam_small_tag_curr_x;

    geometry_msgs::PoseStamped first_goal_pose_msg;
    geometry_msgs::PoseStamped second_goal_pose_msg;
    geometry_msgs::PoseWithCovarianceStamped front_cam_big_tag_pose;
    geometry_msgs::PoseWithCovarianceStamped front_cam_small_tag_pose;

    geometry_msgs::PoseWithCovarianceStamped rear_cam_big_tag_pose;
    geometry_msgs::PoseWithCovarianceStamped rear_cam_small_tag_pose;

    std_msgs::Int8 m_mission_num;

};

tagListener::tagListener()
{
    ROS_INFO("\033[1;32m---->\033[0m Apriltag tf listener started.");

    front_cam_big_tag_prev_x = 0.0;
    front_cam_big_tag_curr_x = 0.0;
    front_cam_small_tag_prev_x = 0.0;
    front_cam_small_tag_curr_x = 0.0;
    rear_cam_big_tag_prev_x = 0.0;
    rear_cam_big_tag_curr_x = 0.0;
    rear_cam_small_tag_prev_x = 0.0;
    rear_cam_small_tag_curr_x = 0.0;

    set_param();

    first_goal_pose_pub = nh.advertise<geometry_msgs::PoseStamped>("/Docking/first_goal_pose", 1);
    second_goal_pose_pub = nh.advertise<geometry_msgs::PoseStamped>("/Docking/second_goal_pose", 1);
    big_tag_detecting_flag_pub = nh.advertise<std_msgs::Bool>("/Docking/big_tag_detecting_flag",1);
    small_tag_detecting_flag_pub = nh.advertise<std_msgs::Bool>("/Docking/small_tag_detecting_flag",1);

    tag_detect_info_sub = nh.subscribe(m_tag_detect_info_topic_name, 10, &tagListener::callbackTagDetectInfo, this);
    rear_camera_tag_detect_info_sub = nh.subscribe(m_rear_camera_tag_detect_info_topic_name, 10, &tagListener::callbackRearCameraTagDetectInfo, this);
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

    nh.param<std::string>("/tag_detect_info_topic_name", m_tag_detect_info_topic_name, "tag_detections");
    nh.param<std::string>("/rear_camera_tag_detect_info_topic_name", m_rear_camera_tag_detect_info_topic_name, "rear_tag_detections");
}

void tagListener::callbackTagDetectInfo(const apriltag_ros::AprilTagDetectionArray::ConstPtr &msg)
{
    // curr_x = msg->detections[0].pose.pose.pose.position.x;
    if(!msg->detections.empty()){
        front_cam_big_tag_pose = msg->detections[0].pose;
        front_cam_small_tag_pose = msg->detections[1].pose;
    }
    std::cout << "front cam" << std::endl;
    std::cout << front_cam_big_tag_pose.pose.pose.position.x << std::endl;
    std::cout << front_cam_small_tag_pose.pose.pose.position.x << std::endl;
}

void tagListener::callbackRearCameraTagDetectInfo(const apriltag_ros::AprilTagDetectionArray::ConstPtr &msg)
{
    // rear_cam_curr_x = msg->detections[0].pose.pose.pose.position.x;
    if(!msg->detections.empty()){
        rear_cam_big_tag_pose = msg->detections[0].pose;
        rear_cam_small_tag_pose = msg->detections[1].pose;
    }
    std::cout << "rear cam" << std::endl;
    std::cout << rear_cam_big_tag_pose.pose.pose.position.x << std::endl;
    std::cout << rear_cam_small_tag_pose.pose.pose.position.x << std::endl;

}

void tagListener::run()
{
    // tf::StampedTransform transform;
    // try{
    //     listener.lookupTransform(camera_frame_id, tag_frame_id,
    //                             ros::Time(0), transform);
    // }
    // catch(tf::TransformException ex){
    //     ROS_ERROR("%s",ex.what());
    //     // ros::Duration(1.0).sleep();
    // }
    // curr_x = transform.getOrigin().x();

    front_cam_big_tag_curr_x = front_cam_big_tag_pose.pose.pose.position.x;
    front_cam_small_tag_curr_x = front_cam_small_tag_pose.pose.pose.position.x;

    rear_cam_big_tag_curr_x = rear_cam_big_tag_pose.pose.pose.position.x;
    rear_cam_small_tag_curr_x = rear_cam_small_tag_pose.pose.pose.position.x;


    if(front_cam_big_tag_curr_x != front_cam_big_tag_prev_x){
        b_big_tag_detecting_flag.data = true;
        std::cout << b_big_tag_detecting_flag.data << std::endl;
    }
    else{
        b_big_tag_detecting_flag.data = false;
        std::cout << b_big_tag_detecting_flag.data << std::endl;
    }

    big_tag_detecting_flag_pub.publish(b_big_tag_detecting_flag);

    // if(b_big_tag_detecting_flag.data == true){

    //     if( fixed_frame_flag == false){
    //         first_goal_pose_msg.header.frame_id = map_frame_id;
    //     }else{
    //         first_goal_pose_msg.header.frame_id = camera_frame_id;
    //     }
    //     first_goal_pose_msg.header.stamp = ros::Time::now();

    //     first_goal_pose_msg.pose.position.x = front_cam_big_tag_pose.pose.pose.position.x + offset_x;
    //     first_goal_pose_msg.pose.position.y = front_cam_big_tag_pose.pose.pose.position.y + offset_y;
    //     first_goal_pose_msg.pose.position.z = 0.0;

    //     first_goal_pose_msg.pose.orientation.x = 0.0;
    //     first_goal_pose_msg.pose.orientation.y = 0.0;
    //     first_goal_pose_msg.pose.orientation.z = 0.0;
    //     first_goal_pose_msg.pose.orientation.w = 1.0;


    //     first_goal_pose_pub.publish(first_goal_pose_msg);
    // }

    front_cam_big_tag_prev_x = front_cam_big_tag_curr_x;
}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "tag_listener");

    tagListener tag_listener;
 
    ros::Rate loop_rate(10);
    while(ros::ok())
    {
        ros::spinOnce();
        tag_listener.run();
        loop_rate.sleep();
    }

    return 0;
}