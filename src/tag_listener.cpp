#include <ros/ros.h>
#include <tf/transform_listener.h>
#include <tf/transform_datatypes.h>
#include <iostream>
#include <std_msgs/Bool.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/TwistStamped.h>
#include <string>
#include <apriltag_ros/AprilTagDetection.h>
#include <apriltag_ros/AprilTagDetectionArray.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <std_msgs/Int8.h>
#include <std_msgs/Int32.h>

#include <tf2_msgs/TFMessage.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Transform.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

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
    std::string base_frame_id;

    void callbackTagDetectInfo(const apriltag_ros::AprilTagDetectionArray::ConstPtr &msg);
    void callbackRearCameraTagDetectInfo(const apriltag_ros::AprilTagDetectionArray::ConstPtr &msg);
    std::string m_tag_detect_info_topic_name;
    std::string m_rear_camera_tag_detect_info_topic_name;

    double approach_long_dist_threshold;
    double approach_dist_threshold;
    int approach_time;

    double neubility_cam_tf_x;
    double neubility_cam_tf_y;
    double neubility_cam_tf_z;

    void pub_enter_goal_pose(geometry_msgs::PoseWithCovarianceStamped tag_pose);
    void pub_enter_small_tag_goal_pose(geometry_msgs::PoseWithCovarianceStamped tag_pose);
    void pub_leave_goal_pose(geometry_msgs::PoseWithCovarianceStamped tag_pose);
    void stop();
    void turn();
    // void ready_to_go();
    // void leave();
    // void stop_and_mode_change();

    int stop_count;

    // bool b_half_turn_flag;
    bool b_is_cover_closed_flag;
    bool b_localization_init_flag;

    // void callbackHalfTurn(const std_msgs::Bool::ConstPtr &msg);
    void callbackIsCoverClosed(const std_msgs::Bool::ConstPtr &msg);
    void callbackLocalizationInit(const std_msgs::Bool::ConstPtr &msg);

    double heading_align_margin;
    double translation_align_margin;

private:
    ros::NodeHandle nh;
    ros::NodeHandle private_nh;

    tf::TransformListener listener;

    ros::Publisher goal_pose_pub;
    
    ros::Publisher front_big_tag_detecting_flag_pub;
    ros::Publisher front_small_tag_detecting_flag_pub;
    ros::Publisher rear_big_tag_detecting_flag_pub;
    ros::Publisher rear_small_tag_detecting_flag_pub;
    
    ros::Publisher mission_num_pub;
    // ros::Publisher docking_command_pub;
    ros::Publisher turn_command_pub;

    ros::Subscriber tag_detect_info_sub;
    ros::Subscriber rear_camera_tag_detect_info_sub;

    ros::Subscriber half_turn_sub;
    ros::Subscriber is_cover_closed_sub;
    ros::Subscriber localization_init_sub;

    std_msgs::Bool b_front_big_tag_detecting_flag;
    std_msgs::Bool b_front_small_tag_detecting_flag;
    std_msgs::Bool b_rear_big_tag_detecting_flag;
    std_msgs::Bool b_rear_small_tag_detecting_flag;

    double front_cam_big_tag_prev_x, front_cam_big_tag_curr_x, front_cam_small_tag_prev_x, front_cam_small_tag_curr_x;
    double rear_cam_big_tag_prev_x, rear_cam_big_tag_curr_x, rear_cam_small_tag_prev_x, rear_cam_small_tag_curr_x;

    geometry_msgs::PoseStamped goal_pose_msg;

    geometry_msgs::PoseWithCovarianceStamped front_cam_big_tag_pose;
    geometry_msgs::PoseWithCovarianceStamped front_cam_small_tag_pose;

    geometry_msgs::PoseWithCovarianceStamped rear_cam_big_tag_pose;
    geometry_msgs::PoseWithCovarianceStamped rear_cam_small_tag_pose;

    std_msgs::Int32 m_mission_num;

    int m_front_big_tag_count;
    int m_front_small_tag_count;
    int m_rear_big_tag_count;
    int m_rear_small_tag_count;

    double small_tag_distance;
    double big_tag_distance;

    geometry_msgs::TwistStamped  m_TagCommandMsg;

    std_msgs::Bool b_turn_command_flag;

    double heading_error;
    double translation_error;
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

    stop_count = 0;

    // b_half_turn_flag = false;
    b_is_cover_closed_flag = false;
    b_localization_init_flag = false;

    heading_error = 180.0;
    translation_error = 10.0;

    set_param();

    goal_pose_pub = nh.advertise<geometry_msgs::PoseStamped>("/Docking/goal_pose", 1);
    
    front_big_tag_detecting_flag_pub = nh.advertise<std_msgs::Bool>("/Docking/front_big_tag_detecting_flag",1);
    front_small_tag_detecting_flag_pub = nh.advertise<std_msgs::Bool>("/Docking/front_small_tag_detecting_flag",1);
    rear_big_tag_detecting_flag_pub = nh.advertise<std_msgs::Bool>("/Docking/rear_big_tag_detecting_flag",1);
    rear_small_tag_detecting_flag_pub = nh.advertise<std_msgs::Bool>("/Docking/rear_small_tag_detecting_flag",1);

    mission_num_pub = nh.advertise<std_msgs::Int32>("/Docking/mission_num", 1);
    // docking_command_pub = nh.advertise<geometry_msgs::TwistStamped>("/Docking/command", 1);
    turn_command_pub = nh.advertise<std_msgs::Bool>("/Docking/turn_command", 1);

    tag_detect_info_sub = nh.subscribe(m_tag_detect_info_topic_name, 10, &tagListener::callbackTagDetectInfo, this);
    rear_camera_tag_detect_info_sub = nh.subscribe(m_rear_camera_tag_detect_info_topic_name, 10, &tagListener::callbackRearCameraTagDetectInfo, this);

    // half_turn_sub = nh.subscribe("/Docking/half_turn", 10, &tagListener::callbackHalfTurn, this);
    is_cover_closed_sub = nh.subscribe("/Docking/cover_closed", 10, &tagListener::callbackIsCoverClosed, this);
    localization_init_sub = nh.subscribe("/Docking/localization_init", 10, &tagListener::callbackLocalizationInit, this);
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

    nh.param<std::string>("base_frame", base_frame_id, "base_link");
    nh.param<std::string>("tag_detect_info_topic_name", m_tag_detect_info_topic_name, "tag_detections");
    nh.param<std::string>("rear_camera_tag_detect_info_topic_name", m_rear_camera_tag_detect_info_topic_name, "rear_/tag_detections");

    nh.getParam("approach_long_dist_threshold", approach_long_dist_threshold);
    nh.getParam("approach_dist_threshold", approach_dist_threshold);
    nh.getParam("approach_time", approach_time);

    nh.getParam("neubility_cam_tf_x", neubility_cam_tf_x);
    nh.getParam("neubility_cam_tf_y", neubility_cam_tf_y);
    nh.getParam("neubility_cam_tf_z", neubility_cam_tf_z);

    nh.getParam("heading_align_margin", heading_align_margin);
    nh.getParam("translation_align_margin", translation_align_margin);
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
}

// void tagListener::callbackHalfTurn(const std_msgs::Bool::ConstPtr &msg)
// {
//     b_half_turn_flag = true;
// }

void tagListener::callbackIsCoverClosed(const std_msgs::Bool::ConstPtr &msg)
{
    b_is_cover_closed_flag = msg->data;
}

void tagListener::callbackLocalizationInit(const std_msgs::Bool::ConstPtr &msg)
{
    b_localization_init_flag = msg->data;
}

void tagListener::pub_enter_goal_pose(geometry_msgs::PoseWithCovarianceStamped tag_pose)
{
    goal_pose_msg.header.frame_id = base_frame_id;
    goal_pose_msg.header.stamp = ros::Time::now();

    goal_pose_msg.pose.position.x = neubility_cam_tf_x + tag_pose.pose.pose.position.z + front_cam_big_tag_offset_x;
    goal_pose_msg.pose.position.y = -neubility_cam_tf_y - tag_pose.pose.pose.position.x + front_cam_big_tag_offset_y;
    goal_pose_msg.pose.position.z = 0.0;

    goal_pose_msg.pose.orientation.x = 0.0;
    goal_pose_msg.pose.orientation.y = 0.0;
    goal_pose_msg.pose.orientation.z = 0.0;
    goal_pose_msg.pose.orientation.w = 1.0;

    goal_pose_pub.publish(goal_pose_msg);
}

void tagListener::pub_enter_small_tag_goal_pose(geometry_msgs::PoseWithCovarianceStamped tag_pose)
{
    goal_pose_msg.header.frame_id = base_frame_id;
    goal_pose_msg.header.stamp = ros::Time::now();

    goal_pose_msg.pose.position.x = neubility_cam_tf_x + tag_pose.pose.pose.position.z + front_cam_small_tag_offset_x;
    goal_pose_msg.pose.position.y = -neubility_cam_tf_y - tag_pose.pose.pose.position.x + front_cam_small_tag_offset_y;
    goal_pose_msg.pose.position.z = 0.0;

    goal_pose_msg.pose.orientation.x = 0.0;
    goal_pose_msg.pose.orientation.y = 0.0;
    goal_pose_msg.pose.orientation.z = 0.0;
    goal_pose_msg.pose.orientation.w = 1.0;

    goal_pose_pub.publish(goal_pose_msg);
}

void tagListener::pub_leave_goal_pose(geometry_msgs::PoseWithCovarianceStamped tag_pose)
{
    goal_pose_msg.header.frame_id = base_frame_id;
    goal_pose_msg.header.stamp = ros::Time::now();

    // goal_pose_msg.pose.position.x = -neubility_cam_tf_x - tag_pose.pose.pose.position.z + rear_cam_small_tag_offset_x;
    // goal_pose_msg.pose.position.y = -neubility_cam_tf_y + tag_pose.pose.pose.position.x + rear_cam_small_tag_offset_y;

    goal_pose_msg.pose.position.x = rear_cam_small_tag_offset_x;
    goal_pose_msg.pose.position.y = rear_cam_small_tag_offset_y;
    goal_pose_msg.pose.position.z = 0.0;

    // goal_pose_msg.pose.orientation.x = 0.0;
    // goal_pose_msg.pose.orientation.y = 0.0;
    // goal_pose_msg.pose.orientation.z = 0.0;
    // goal_pose_msg.pose.orientation.w = 1.0;

    tf2::Transform transform_init_cam;
    transform_init_cam.setOrigin(tf2::Vector3(0.0, 0.0, 0.0));
    tf2::Quaternion quat_init_cam;
    quat_init_cam.setRPY(-1.570796, 0.0, -1.570796);
    transform_init_cam.setRotation(quat_init_cam);

    geometry_msgs::Quaternion quat_msg;
    tf2::Quaternion quat_tag;
    quat_msg = tag_pose.pose.pose.orientation;
    tf2::convert(quat_msg, quat_tag); 

    tf2::Transform transform_tag;
    transform_tag.setRotation(quat_tag);

    tf2::Transform transform_normal;
    transform_normal.setOrigin(tf2::Vector3(0.0, 0.0, 0.0));
    tf2::Quaternion quat_normal;
    quat_normal.setRPY(0.0, 1.570796, 0.0);
    transform_normal.setRotation(quat_normal);


    tf2::Transform transform_total = transform_init_cam * transform_tag * transform_normal;

    tf2::Quaternion q = transform_total.getRotation();

    goal_pose_msg.pose.orientation.x = q.x(); //tag_pose.pose.pose.orientation.x;
    goal_pose_msg.pose.orientation.y = q.y(); //tag_pose.pose.pose.orientation.y;
    goal_pose_msg.pose.orientation.z = q.z(); //tag_pose.pose.pose.orientation.z;
    goal_pose_msg.pose.orientation.w = q.w(); //tag_pose.pose.pose.orientation.w;

    goal_pose_pub.publish(goal_pose_msg);
}

void tagListener::stop()
{
    goal_pose_msg.header.frame_id = base_frame_id;
    goal_pose_msg.header.stamp = ros::Time::now();

    goal_pose_msg.pose.position.x = 0.0;
    goal_pose_msg.pose.position.y = 0.0;
    goal_pose_msg.pose.position.z = 0.0;

    goal_pose_msg.pose.orientation.x = 0.0;
    goal_pose_msg.pose.orientation.y = 0.0;
    goal_pose_msg.pose.orientation.z = 0.0;
    goal_pose_msg.pose.orientation.w = 1.0;

    goal_pose_pub.publish(goal_pose_msg);
}

void tagListener::turn()
{
    b_turn_command_flag.data = true;
}

void tagListener::run()
{
    front_cam_big_tag_curr_x = front_cam_big_tag_pose.pose.pose.position.z;
    front_cam_small_tag_curr_x = front_cam_small_tag_pose.pose.pose.position.z;

    rear_cam_big_tag_curr_x = rear_cam_big_tag_pose.pose.pose.position.z;
    rear_cam_small_tag_curr_x = rear_cam_small_tag_pose.pose.pose.position.z;

    // front big tag detection
    if(front_cam_big_tag_curr_x != front_cam_big_tag_prev_x){
        b_front_big_tag_detecting_flag.data = true;
        m_front_big_tag_count++;
        if(m_front_big_tag_count >30){
            m_front_big_tag_count = 30;
        }
        // std::cout << "++front_big_tag_count: " << m_front_big_tag_count << std::endl;
    }
    else{
        b_front_big_tag_detecting_flag.data = false;
        m_front_big_tag_count --;
        if(m_front_big_tag_count < 0){
            m_front_big_tag_count = 0;
        }
        // std::cout << "--front_big_tag_count: " << m_front_big_tag_count << std::endl;
    }
    front_big_tag_detecting_flag_pub.publish(b_front_big_tag_detecting_flag);

    // front small tag detection
    if(front_cam_small_tag_curr_x != front_cam_small_tag_prev_x){
        b_front_small_tag_detecting_flag.data = true;
        m_front_small_tag_count++;
        if(m_front_small_tag_count > 15){
            m_front_small_tag_count = 15;
        }
        // std::cout << "++front_small_tag_count: " << m_front_small_tag_count << std::endl;
    }
    else{
        b_front_small_tag_detecting_flag.data = false;
        m_front_small_tag_count--;
        if(m_front_small_tag_count < 0){
            m_front_small_tag_count = 0;
        }
        // std::cout << "--front_small_tag_count: " << m_front_small_tag_count << std::endl;
    }
    front_small_tag_detecting_flag_pub.publish(b_front_small_tag_detecting_flag);

    // rear big tag detection
    if(rear_cam_big_tag_curr_x != rear_cam_big_tag_prev_x){
        b_rear_big_tag_detecting_flag.data = true;
        m_rear_big_tag_count++;
        if(m_rear_big_tag_count > 30){
            m_rear_big_tag_count = 30;
        }
        // std::cout << "++rear_big_tag_count: " << m_rear_big_tag_count << std::endl;
    }
    else{
        b_rear_big_tag_detecting_flag.data = false;
        m_rear_big_tag_count--;
        if(m_rear_big_tag_count < 0){
            m_rear_big_tag_count = 0;
        }
        // std::cout << "--rear_big_tag_count: " << m_rear_big_tag_count << std::endl;
    }
    rear_big_tag_detecting_flag_pub.publish(b_rear_big_tag_detecting_flag);

    // rear small tag detection
    if(rear_cam_small_tag_curr_x != rear_cam_small_tag_prev_x){
        b_rear_small_tag_detecting_flag.data = true;
        m_rear_small_tag_count++;
        if(m_rear_small_tag_count > 30){
            m_rear_small_tag_count = 30;
        }
        // std::cout << "++rear_small_tag_count: " << m_rear_small_tag_count << std::endl;
    }
    else{
        b_rear_small_tag_detecting_flag.data = false;
        m_rear_small_tag_count--;
        if(m_rear_small_tag_count < 0){
            m_rear_small_tag_count = 0;
        }
        // std::cout << "--rear_small_tag_count: " << m_rear_small_tag_count << std::endl;
    }
    rear_small_tag_detecting_flag_pub.publish(b_rear_small_tag_detecting_flag);

    // mission number publish
    // ( approach tag )
    if (m_mission_num.data == 0 || m_mission_num.data == 1 || m_mission_num.data == 2){
        big_tag_distance = sqrt(pow(front_cam_big_tag_pose.pose.pose.position.z, 2)
                                     + pow(front_cam_big_tag_pose.pose.pose.position.x, 2));

        if (big_tag_distance < approach_long_dist_threshold) {
            if(m_front_big_tag_count >= approach_time && m_front_small_tag_count < (approach_time -10)){
                m_mission_num.data = 1;
            }
            if(m_front_small_tag_count >= (approach_time-10)){
                small_tag_distance = sqrt(pow(front_cam_small_tag_pose.pose.pose.position.z, 2)
                                        + pow(front_cam_small_tag_pose.pose.pose.position.x, 2));

                if(small_tag_distance < approach_dist_threshold){
                    m_mission_num.data = 3;
                }
                else{
                    m_mission_num.data = 2;
                }
            }            
        }
        else {
            m_mission_num.data = 0;
        }


    }

    // ( stop )
    if (m_mission_num.data == 3){
        stop_count++;
        if (stop_count > 30){
            stop_count = 0;
            m_mission_num.data = 4;
        }
        else{
            m_mission_num.data = 3;
        }
    }

    // ( turn)
    if(m_mission_num.data == 4 ){
        if(b_rear_small_tag_detecting_flag.data == true){
            // calculate whether heading is aligned or not 
            tf::Quaternion q(rear_cam_small_tag_pose.pose.pose.orientation.x, 
                            rear_cam_small_tag_pose.pose.pose.orientation.y,
                            rear_cam_small_tag_pose.pose.pose.orientation.z,
                            rear_cam_small_tag_pose.pose.pose.orientation.w);
            tf::Matrix3x3 m(q);
            double roll, pitch, yaw;
            m.getRPY(roll, pitch, yaw);
            
            translation_error = fabs(rear_cam_small_tag_pose.pose.pose.position.x);
            heading_error = fabs(pitch * 180.0 / M_PI);
            // std::cout << "t_error : " << translation_error << std::endl;
            // std::cout << "h_error : " << heading_error << std::endl;
            // std::cout << "---------------------" << std::endl;
        }

        if ( translation_error <= translation_align_margin){
            if( heading_error <= heading_align_margin ){
                // heading is aligned
                b_turn_command_flag.data = false;
                // pub stop flag
                m_mission_num.data = 5;
            }
            else{
                m_mission_num.data = 4;
            }
        }
        else {
            // heading is not aligned yet;
            // keep turing 
            m_mission_num.data = 4;
        }
    }

    // (ready to go)
    if(m_mission_num.data == 5){
        if (m_rear_small_tag_count >= approach_time){
            if (b_is_cover_closed_flag == true){
                b_is_cover_closed_flag = false;
                m_mission_num.data = 6;
            }
            else {
                m_mission_num.data = 5;
            }
        }
        else{
            m_mission_num.data = 5;
        }
    }

    // (leave)
    if(m_mission_num.data == 6){
        if(m_rear_big_tag_count >= approach_time){
            m_mission_num.data = 7;
        }
        else{
            m_mission_num.data = 6;
        }
    }

    // (stop & mode change (mission number init) )
    if(m_mission_num.data == 7){
        if (b_localization_init_flag == true){
            m_front_big_tag_count = 0;
            m_front_small_tag_count = 0;
            m_rear_big_tag_count = 0;
            m_rear_small_tag_count = 0;

            b_localization_init_flag = false;

            m_mission_num.data = 0;
        }
        else{
            m_mission_num.data = 7;
        }
    }

    mission_num_pub.publish(m_mission_num);

    switch(m_mission_num.data){
        // mission # 1 ( approach big tag )
        case 1:
            pub_enter_goal_pose(front_cam_big_tag_pose);
            break;
        // mission # 2 ( aapproach small tag )
        case 2:
            pub_enter_small_tag_goal_pose(front_cam_small_tag_pose);
            break;
        // mission # 3 ( stop )
        case 3:
            stop();
            break;
        // mission # 4 ( turn 180 deg )
        case 4:
            turn();
            break;
        // mission # 5 ( ready to go)
        case 5:
            stop();
            break;
        // mission # 6 ( leave station )
        case 6:
            pub_leave_goal_pose(rear_cam_small_tag_pose);
            break;
        // mission # 7 ( stop & change to auto mode )
        case 7:
            stop();
            break;
        // auto mode ( mission # : 0 )
        default:
            break;

    }

    turn_command_pub.publish(b_turn_command_flag);

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