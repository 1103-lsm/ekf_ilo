#include "ros/ros.h"
#include "utils/parameters.h"
#include <message_filters/subscriber.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/exact_time.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <message_filters/time_synchronizer.h>

#include <ros/console.h>
#include <sensor_msgs/Imu.h>
#include <nav_msgs/Odometry.h>
#include <nav_msgs/Path.h>
#include <sensor_msgs/JointState.h>

#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/TwistStamped.h>
#include <tf2/LinearMath/Quaternion.h>

#include "kalmanFilter/basicEKF.h"
#include "utils/utility.h"

BasicEKF kf;
SensorData sensor_data;

double curr_t;
bool first_sensor_received = false;

// debug print filtered data
ros::Publisher filterd_imu_pub;
ros::Publisher filterd_joint_pub;
ros::Publisher filterd_odom_pub;
ros::Publisher filterd_path_pub;
geometry_msgs::PoseStamped estimated_pose;
geometry_msgs::TwistStamped estimated_twist;

ros::Publisher pub_truth_path;
ros::Publisher pub_truth_odometry;

// truth pose
nav_msgs::Path truth_path;
nav_msgs::Odometry truth_odometry;

nav_msgs::Path filterd_path;
nav_msgs::Odometry filterd_odom_msg;

void sensor_callback(const sensor_msgs::Imu::ConstPtr &imu_msg, const sensor_msgs::JointState::ConstPtr &joint_msg);
void geometry_callback(const geometry_msgs::PoseStamped::ConstPtr& poseMsg, const geometry_msgs::TwistStamped::ConstPtr& twistMsg);

int main(int argc, char *argv[])
{
    // 执行 ros 节点初始化
    ros::init(argc,argv,"ilo");
    ros::NodeHandle n("~");
    ros::console::set_logger_level(ROSCONSOLE_DEFAULT_NAME, ros::console::levels::Info);

    // 输入两个参数
    if (argc != 2)
    {
        printf("please intput: rosrun ilo ilo [config file] \n"
               "for example: rosrun ilo ilo config/aliengo_config/aliengo_ilo_config.yaml");
        return 1;
    }

    // 读取 yaml 文件参数
    std::string config_file = argv[1];
    printf("config_file: %s\n", argv[1]);
    readParameters(config_file);

    /*** IMU and leg ***/
    message_filters::Subscriber<sensor_msgs::Imu> imu_sub;
    message_filters::Subscriber<sensor_msgs::JointState> joint_state_sub;
    imu_sub.subscribe(n, IMU_TOPIC, 200);
    joint_state_sub.subscribe(n, LEG_TOPIC, 200);
    typedef message_filters::sync_policies::ApproximateTime<sensor_msgs::Imu, sensor_msgs::JointState> imu_leg_SyncPolicy;
    message_filters::Synchronizer<imu_leg_SyncPolicy> imu_leg_sync(imu_leg_SyncPolicy(30), imu_sub, joint_state_sub);

    imu_leg_sync.registerCallback(boost::bind(&sensor_callback, _1, _2));
    /****************************************************************************************************************************/

    /************************************************** truth data *****************************************************************/
    message_filters::Subscriber<geometry_msgs::PoseStamped> pose_sub(n, POSE_TOPIC, 200); // 200 为队列深度
    message_filters::Subscriber<geometry_msgs::TwistStamped> twist_sub(n, TWIST_TOPIC, 200); 

    // 定义同步策略（近似时间同步）
    typedef message_filters::sync_policies::ApproximateTime<geometry_msgs::PoseStamped, geometry_msgs::TwistStamped> pose_twist_SyncPolicy;
    message_filters::Synchronizer<pose_twist_SyncPolicy> pose_twist_sync(pose_twist_SyncPolicy(30), pose_sub, twist_sub);

    pose_twist_sync.registerCallback(boost::bind(&geometry_callback, _1, _2));

    /********************************************************************************************************************************/

    /* publishers */
    filterd_path_pub = n.advertise<nav_msgs::Path>("/filterd_path", 1000);
    filterd_odom_pub = n.advertise<nav_msgs::Odometry>("/filterd_odom", 1000);

    pub_truth_path = n.advertise<nav_msgs::Path>("truth_path", 1000);
    pub_truth_odometry = n.advertise<nav_msgs::Odometry>("truth_odometry", 1000);

    // 循环处理消息
    ros::spin();

    return 0;
}



void sensor_callback(const sensor_msgs::Imu::ConstPtr& imu_msg, const sensor_msgs::JointState::ConstPtr& joint_msg) 
{
    // 获取代码开始时间戳
    ros::Time start_time = ros::Time::now();

    double t = imu_msg->header.stamp.toSec();

    Eigen::Vector3d acc = Eigen::Vector3d(imu_msg->linear_acceleration.x, imu_msg->linear_acceleration.y, imu_msg->linear_acceleration.z);
    Eigen::Vector3d ang_vel = Eigen::Vector3d(imu_msg->angular_velocity.x, imu_msg->angular_velocity.y, imu_msg->angular_velocity.z);
    Eigen::Quaterniond orientation = Eigen::Quaterniond(imu_msg->orientation.w, imu_msg->orientation.x, imu_msg->orientation.y, imu_msg->orientation.z);

    Eigen::Matrix<double, NUM_DOF,1> joint_pos; 
    Eigen::Matrix<double, NUM_DOF,1> joint_vel;
    Eigen::Matrix<double, NUM_LEG,1> plan_contacts;

    for (int i = 0; i < NUM_DOF; ++i)
    {
        joint_pos[i] = joint_msg->position[i];
        joint_vel[i] = joint_msg->velocity[i];
    }
    for (int i = 0; i < NUM_LEG; ++i)
    {
        plan_contacts[i] = joint_msg->effort[NUM_DOF + i];
    }

    double dt;
    sensor_data.input_imu(acc, ang_vel);
    sensor_data.input_leg(joint_pos, joint_vel, plan_contacts);
    sensor_data.input_orientation(orientation);

    if ( !kf.is_inited() && first_sensor_received == false ) {
        dt = 0;
        curr_t = t;
        sensor_data.input_dt(dt);
    } else if ( !kf.is_inited()) {
        dt = t- curr_t;
        sensor_data.input_dt(dt);
        curr_t = t;
    } else {
        dt = t- curr_t;
        
        sensor_data.input_dt(dt);
        kf.update_filter(sensor_data);
        curr_t = t;
    }
    // 获取代码结束时间戳
    ros::Time end_time = ros::Time::now();
    // 计算代码的运行时间
    ros::Duration duration = end_time - start_time;
    double runtime = duration.toSec()*1000;
    if(runtime != 0)
    {
        ROS_INFO("runtime: %.6f ms",runtime );
    }


    first_sensor_received = true;

    if  (!kf.is_inited())
    {
        return;
    }

    Eigen::Matrix<double, EKF_STATE_SIZE,1> kf_state = kf.get_state();
    Eigen::Quaterniond kf_orientation = kf.get_orientation();

     // 发布滤波后的 path
    filterd_path.header.stamp = ros::Time::now();
    filterd_path.header.frame_id = "world";


    estimated_pose.header.stamp = filterd_path.header.stamp;

    estimated_pose.pose.position.x = kf_state[0]; 
    estimated_pose.pose.position.y = kf_state[1]; 
    estimated_pose.pose.position.z = kf_state[2]; 
    estimated_pose.pose.orientation.x = kf_orientation.x();
    estimated_pose.pose.orientation.y = kf_orientation.y();
    estimated_pose.pose.orientation.z = kf_orientation.z();
    estimated_pose.pose.orientation.w = kf_orientation.w();

    filterd_path.poses.push_back(estimated_pose);

    filterd_path_pub.publish(filterd_path);


    estimated_twist.twist.linear.x = kf_state[3];
    estimated_twist.twist.linear.y = kf_state[4];
    estimated_twist.twist.linear.z = kf_state[5];
    
    // 发布滤波后的 odometry
    filterd_odom_msg.header.stamp = filterd_path.header.stamp;
    filterd_odom_msg.header.frame_id = "world";
    filterd_odom_msg.child_frame_id = "world";

    filterd_odom_msg.twist.twist= estimated_twist.twist;

    filterd_odom_msg.pose.pose = estimated_pose.pose;

    filterd_odom_msg.pose.pose.orientation.x = kf_orientation.x();
    filterd_odom_msg.pose.pose.orientation.y = kf_orientation.y();
    filterd_odom_msg.pose.pose.orientation.z = kf_orientation.z();
    filterd_odom_msg.pose.pose.orientation.w = kf_orientation.w();

    filterd_odom_pub.publish(filterd_odom_msg);

    return;
}

// 处理pose和twist消息的回调函数
void geometry_callback(const geometry_msgs::PoseStamped::ConstPtr& poseMsg, const geometry_msgs::TwistStamped::ConstPtr& twistMsg)
{ 
    
    // 初始化对齐坐标系
    if ( !kf.is_inited() && first_sensor_received == true) {
        Eigen::Matrix<double, 3, 1> opti_pos = Eigen::Matrix<double, 3, 1> (poseMsg->pose.position.x, poseMsg->pose.position.y, poseMsg->pose.position.z);
        Eigen::Quaterniond opti_orientation = Eigen::Quaterniond(poseMsg->pose.orientation.w, poseMsg->pose.orientation.x, poseMsg->pose.orientation.y, poseMsg->pose.orientation.z);
        kf.init_filter(sensor_data, opti_pos,opti_orientation);
    }

    // 计算并打印位置误差、姿态误差和速度误差
    PositionError position_error = Utility::calculatePositionError(*poseMsg, estimated_pose);
    OrientationError orientation_error = Utility::calculateOrientationError(*poseMsg, estimated_pose);
    double velocity_error = Utility::calculateVelocityError(*twistMsg, estimated_twist);

    ROS_INFO("position error (X, Y, Z) m: %.4f, %.4f, %.4f", position_error.x, position_error.y, position_error.z);
    ROS_INFO("attitude error (Roll, Pitch, Yaw) ard: %.4f, %.4f, %.4f", orientation_error.roll, orientation_error.pitch, orientation_error.yaw);
    ROS_INFO("speed error: %.4f m/s", velocity_error);

    // 发布path消息到pub_path话题
    truth_path.header = poseMsg->header;
    truth_path.header.frame_id = "world";
    truth_path.poses.push_back(*poseMsg);
    pub_truth_path.publish(truth_path);

    // 发布odometry消息到pub_odometry话题
    truth_odometry.header = poseMsg->header;
    truth_odometry.child_frame_id = "world";
    truth_odometry.pose.pose = poseMsg->pose;
    truth_odometry.twist.twist = twistMsg->twist;
    pub_truth_odometry.publish(truth_odometry);
}