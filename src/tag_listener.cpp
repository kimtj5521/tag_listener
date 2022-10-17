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
#include <std_msgs/Int32.h>

class tagListener
{
public:
    tagListener();
    ~tagListener();

    void set_param();

    void run();

    double front_cam_big_tag_offset_x;
    double front_cam_big_tag_offset_y;
    double front_cam_small_tag_offset_x;
    double front_cam_small_tag_offset_y;
    double rear_cam_small_tag_offset_x;
    double rear_cam_small_tag_offset_y;
    std::string camera_frame_id;
    std::string tag_frame_id;
    std::string map_frame_id;
    bool fixed_frame_flag;

    void callbackTagDetectInfo(const apriltag_ros::AprilTagDetectionArray::ConstPtr &msg);
    void callbackRearCameraTagDetectInfo(const apriltag_ros::AprilTagDetectionArray::ConstPtr &msg);
    std::string m_tag_detect_info_topic_name;
    std::string m_rear_camera_tag_detect_info_topic_name;

    double approach_dist_threshold;
    int approach_time;
    // apriltag_ros::AprilTagDetectionArray temp_april_msg;

private:
    ros::NodeHandle nh;
    ros::NodeHandle private_nh;

    tf::TransformListener listener;

    ros::Publisher first_goal_pose_pub;
    ros::Publisher second_goal_pose_pub;
    
    ros::Publisher front_big_tag_detecting_flag_pub;
    ros::Publisher front_small_tag_detecting_flag_pub;
    ros::Publisher rear_big_tag_detecting_flag_pub;
    ros::Publisher rear_small_tag_detecting_flag_pub;
    
    ros::Publisher mission_num_pub;

    ros::Subscriber tag_detect_info_sub;
    ros::Subscriber rear_camera_tag_detect_info_sub;

    std_msgs::Bool b_front_big_tag_detecting_flag;
    std_msgs::Bool b_front_small_tag_detecting_flag;
    std_msgs::Bool b_rear_big_tag_detecting_flag;
    std_msgs::Bool b_rear_small_tag_detecting_flag;

    double front_cam_big_tag_prev_x, front_cam_big_tag_curr_x, front_cam_small_tag_prev_x, front_cam_small_tag_curr_x;
    double rear_cam_big_tag_prev_x, rear_cam_big_tag_curr_x, rear_cam_small_tag_prev_x, rear_cam_small_tag_curr_x;

    geometry_msgs::PoseStamped first_goal_pose_msg;
    geometry_msgs::PoseStamped second_goal_pose_msg;

    geometry_msgs::PoseWithCovarianceStamped front_cam_big_tag_pose;
    geometry_msgs::PoseWithCovarianceStamped front_cam_small_tag_pose;

    geometry_msgs::PoseWithCovarianceStamped rear_cam_big_tag_pose;
    geometry_msgs::PoseWithCovarianceStamped rear_cam_small_tag_pose;

    std_msgs::Int8 m_mission_num;

    int m_front_big_tag_count;
    int m_front_small_tag_count;
    int m_rear_big_tag_count;
    int m_rear_small_tag_count;

    double small_tag_distance;
    int mission_count;
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

    m_front_big_tag_count = 0;
    m_front_small_tag_count = 0;
    m_rear_big_tag_count = 0;
    m_rear_small_tag_count = 0;

    m_mission_num.data = 0;
    mission_count = 0;

    set_param();

    first_goal_pose_pub = nh.advertise<geometry_msgs::PoseStamped>("/Docking/first_goal_pose", 1);
    second_goal_pose_pub = nh.advertise<geometry_msgs::PoseStamped>("/Docking/second_goal_pose", 1);

    front_big_tag_detecting_flag_pub = nh.advertise<std_msgs::Bool>("/Docking/front_big_tag_detecting_flag",1);
    front_small_tag_detecting_flag_pub = nh.advertise<std_msgs::Bool>("/Docking/front_small_tag_detecting_flag",1);
    rear_big_tag_detecting_flag_pub = nh.advertise<std_msgs::Bool>("/Docking/rear_big_tag_detecting_flag",1);
    rear_small_tag_detecting_flag_pub = nh.advertise<std_msgs::Bool>("/Docking/rear_small_tag_detecting_flag",1);

    mission_num_pub = nh.advertise<std_msgs::Int8>("/Docking/mission_num",1);

    tag_detect_info_sub = nh.subscribe(m_tag_detect_info_topic_name, 10, &tagListener::callbackTagDetectInfo, this);
    rear_camera_tag_detect_info_sub = nh.subscribe(m_rear_camera_tag_detect_info_topic_name, 10, &tagListener::callbackRearCameraTagDetectInfo, this);
}

tagListener::~tagListener()
{
    ROS_INFO("\033[1;32m---->\033[0m Apriltag tf listener is destroyed");
}

void tagListener::set_param()
{
    nh.getParam("front_cam_big_tag_offset_x", front_cam_big_tag_offset_x);
    nh.getParam("front_cam_big_tag_offset_y", front_cam_big_tag_offset_y);
    nh.getParam("front_cam_small_tag_offset_x", front_cam_small_tag_offset_x);
    nh.getParam("front_cam_small_tag_offset_y", front_cam_small_tag_offset_y);

    nh.getParam("rear_cam_small_tag_offset_x", rear_cam_small_tag_offset_x);
    nh.getParam("rear_cam_small_tag_offset_y", rear_cam_small_tag_offset_y);

    nh.param<std::string>("camera_frame", camera_frame_id, "camera_link");
    nh.param<std::string>("tag_frame", tag_frame_id, "tag_0");
    nh.param<std::string>("map_frame", map_frame_id, "map");
    nh.param<bool>("fixed_frame_flag", fixed_frame_flag, false);

    nh.param<std::string>("tag_detect_info_topic_name", m_tag_detect_info_topic_name, "tag_detections");
    nh.param<std::string>("rear_camera_tag_detect_info_topic_name", m_rear_camera_tag_detect_info_topic_name, "rear_/tag_detections");

    nh.getParam("approach_dist_threshold", approach_dist_threshold);
    nh.getParam("approach_time", approach_time);
}

void tagListener::callbackTagDetectInfo(const apriltag_ros::AprilTagDetectionArray::ConstPtr &msg)
{
    if(!msg->detections.empty()){
        for (auto detection: msg->detections){
            if (detection.id[0] == 0){
                front_cam_big_tag_pose = detection.pose;;
            }
            if (detection.id[0] == 1){
                front_cam_small_tag_pose = detection.pose;
            }
        }
    }
    // std::cout << "front cam" << std::endl;
    // std::cout << front_cam_big_tag_pose.pose.pose.position.x << std::endl;
    // std::cout << front_cam_small_tag_pose.pose.pose.position.x << std::endl;
}

void tagListener::callbackRearCameraTagDetectInfo(const apriltag_ros::AprilTagDetectionArray::ConstPtr &msg)
{
    if(!msg->detections.empty()){
        for (auto detection: msg->detections){
            if (detection.id[0] == 0){
                rear_cam_big_tag_pose = detection.pose;;
            }
            if (detection.id[0] == 1){
                rear_cam_small_tag_pose = detection.pose;
            }
        }
    }
    // std::cout << "rear cam" << std::endl;
    // std::cout << rear_cam_big_tag_pose.pose.pose.position.x << std::endl;
    // std::cout << rear_cam_small_tag_pose.pose.pose.position.x << std::endl;
}

void tagListener::run()
{
    front_cam_big_tag_curr_x = front_cam_big_tag_pose.pose.pose.position.x;
    front_cam_small_tag_curr_x = front_cam_small_tag_pose.pose.pose.position.x;

    rear_cam_big_tag_curr_x = rear_cam_big_tag_pose.pose.pose.position.x;
    rear_cam_small_tag_curr_x = rear_cam_small_tag_pose.pose.pose.position.x;

    // front big tag detection
    if(front_cam_big_tag_curr_x != front_cam_big_tag_prev_x){
        b_front_big_tag_detecting_flag.data = true;
        m_front_big_tag_count++;
        if(m_front_big_tag_count >30){
            m_front_big_tag_count = 30;
        }
        std::cout << "++front_big_tag_count: " << m_front_big_tag_count << std::endl;
    }
    else{
        b_front_big_tag_detecting_flag.data = false;
        m_front_big_tag_count --;
        if(m_front_big_tag_count < 0){
            m_front_big_tag_count = 0;
        }
        std::cout << "--front_big_tag_count: " << m_front_big_tag_count << std::endl;
    }
    front_big_tag_detecting_flag_pub.publish(b_front_big_tag_detecting_flag);

    // front small tag detection
    if(front_cam_small_tag_curr_x != front_cam_small_tag_prev_x){
        b_front_small_tag_detecting_flag.data = true;
        m_front_small_tag_count++;
        if(m_front_small_tag_count > 15){
            m_front_small_tag_count = 15;
        }
        std::cout << "++front_small_tag_count: " << m_front_small_tag_count << std::endl;
    }
    else{
        b_front_small_tag_detecting_flag.data = false;
        m_front_small_tag_count--;
        if(m_front_small_tag_count < 0){
            m_front_small_tag_count = 0;
        }
        std::cout << "--front_small_tag_count: " << m_front_small_tag_count << std::endl;
    }
    front_small_tag_detecting_flag_pub.publish(b_front_small_tag_detecting_flag);

    // rear big tag detection
    if(rear_cam_big_tag_curr_x != rear_cam_big_tag_prev_x){
        b_rear_big_tag_detecting_flag.data = true;
        m_rear_big_tag_count++;
        if(m_rear_big_tag_count > 30){
            m_rear_big_tag_count = 30;
        }
        std::cout << "++rear_big_tag_count: " << m_rear_big_tag_count << std::endl;
    }
    else{
        b_rear_big_tag_detecting_flag.data = false;
        m_rear_big_tag_count--;
        if(m_rear_big_tag_count < 0){
            m_rear_big_tag_count = 0;
        }
        std::cout << "--rear_big_tag_count: " << m_rear_big_tag_count << std::endl;
    }
    rear_big_tag_detecting_flag_pub.publish(b_rear_big_tag_detecting_flag);

    // rear small tag detection
    if(rear_cam_small_tag_curr_x != rear_cam_small_tag_prev_x){
        b_rear_small_tag_detecting_flag.data = true;
        m_rear_small_tag_count++;
        if(m_rear_small_tag_count > 30){
            m_rear_small_tag_count = 30;
        }
        std::cout << "++rear_small_tag_count: " << m_rear_small_tag_count << std::endl;
    }
    else{
        b_rear_small_tag_detecting_flag.data = false;
        m_rear_small_tag_count--;
        if(m_rear_small_tag_count < 0){
            m_rear_small_tag_count = 0;
        }
        std::cout << "--rear_small_tag_count: " << m_rear_small_tag_count << std::endl;
    }
    rear_small_tag_detecting_flag_pub.publish(b_rear_small_tag_detecting_flag);

    // mission number publish
    if (m_mission_num.data !=3 && m_front_big_tag_count >= approach_time && m_front_small_tag_count <(approach_time - 10)){
        m_mission_num.data = 1;

        first_goal_pose_msg.header.frame_id = map_frame_id;
        first_goal_pose_msg.header.stamp = ros::Time::now();

        first_goal_pose_msg.pose.position.x = front_cam_big_tag_pose.pose.pose.position.x + front_cam_big_tag_offset_x;
        first_goal_pose_msg.pose.position.y = front_cam_big_tag_pose.pose.pose.position.y + front_cam_big_tag_offset_y;
        first_goal_pose_msg.pose.position.z = 0.0;

        first_goal_pose_msg.pose.orientation.x = 0.0;
        first_goal_pose_msg.pose.orientation.y = 0.0;
        first_goal_pose_msg.pose.orientation.z = 0.0;
        first_goal_pose_msg.pose.orientation.w = 1.0;

        first_goal_pose_pub.publish(first_goal_pose_msg);
    }
    else if(m_front_small_tag_count >= (approach_time - 10)){
        second_goal_pose_msg.header.frame_id = map_frame_id;
        second_goal_pose_msg.header.stamp = ros::Time::now();

        second_goal_pose_msg.pose.position.x = front_cam_small_tag_pose.pose.pose.position.x + front_cam_small_tag_offset_x;
        second_goal_pose_msg.pose.position.y = front_cam_small_tag_pose.pose.pose.position.y + front_cam_small_tag_offset_y;
        second_goal_pose_msg.pose.position.z = 0.0;

        second_goal_pose_msg.pose.orientation.x = 0.0;
        second_goal_pose_msg.pose.orientation.y = 0.0;
        second_goal_pose_msg.pose.orientation.z = 0.0;
        second_goal_pose_msg.pose.orientation.w = 1.0;

        second_goal_pose_pub.publish(second_goal_pose_msg);

        small_tag_distance = sqrt(pow(front_cam_small_tag_pose.pose.pose.position.x , 2) + pow(front_cam_small_tag_pose.pose.pose.position.y , 2));

        if(small_tag_distance < approach_dist_threshold){
            m_mission_num.data = 3;
        }
        else{
            m_mission_num.data = 2;
        }
    }
    else if(m_mission_num.data == 3 && m_rear_small_tag_count >= approach_time && m_rear_big_tag_count <approach_time){
        m_mission_num.data = 4;

        second_goal_pose_msg.header.frame_id = map_frame_id;
        second_goal_pose_msg.header.stamp = ros::Time::now();

        second_goal_pose_msg.pose.position.x = rear_cam_small_tag_pose.pose.pose.position.x + rear_cam_small_tag_offset_x;
        second_goal_pose_msg.pose.position.y = rear_cam_small_tag_pose.pose.pose.position.y + rear_cam_small_tag_offset_y;
        second_goal_pose_msg.pose.position.z = 0.0;

        second_goal_pose_msg.pose.orientation.x = 0.0;
        second_goal_pose_msg.pose.orientation.y = 0.0;
        second_goal_pose_msg.pose.orientation.z = 0.0;
        second_goal_pose_msg.pose.orientation.w = 1.0;

        second_goal_pose_pub.publish(second_goal_pose_msg);
    }
    else if(m_mission_num.data == 4 && m_rear_big_tag_count >= approach_time){
        m_mission_num.data = 5;
    }

    if( m_mission_num.data == 5){
        mission_count ++;
        // if(mission_count > approach_time){
        //     mission_count = approach_time;
        // }
        if(mission_count >= approach_time){
            m_mission_num.data = 0;
            mission_count = 0;

            m_front_big_tag_count = 0;
            m_front_small_tag_count = 0;
            m_rear_big_tag_count = 0;
            m_rear_small_tag_count = 0;
        }
    }

    mission_num_pub.publish(m_mission_num);


    front_cam_big_tag_prev_x = front_cam_big_tag_curr_x;
    front_cam_small_tag_prev_x = front_cam_small_tag_curr_x;

    rear_cam_big_tag_prev_x = rear_cam_big_tag_curr_x;
    rear_cam_small_tag_prev_x = rear_cam_small_tag_curr_x;
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