#include "yaml-cpp/yaml.h"
#include "parameters.h"
#include "utility.h"


std::string IMU_TOPIC;
std::string POSE_TOPIC;
std::string TWIST_TOPIC;
std::string LEG_TOPIC;
// 运动参数
double LEG_OFFSET_X;
double LEG_OFFSET_Y;
double THIGH_OFFSET;
double THIGH_LENGTH;
double CALF_LENGTH;

template <typename T>
T readParam(ros::NodeHandle &n, std::string name)
{
    T ans;
    if (n.getParam(name, ans))
    {
        ROS_INFO_STREAM("Loaded " << name << ": " << ans);
    }
    else
    {
        ROS_ERROR_STREAM("Failed to load " << name);
        n.shutdown();
    }
    return ans;
}

/******************从配置文件读取参数************************/
void readParameters(std::string config_file)
{
    FILE *fh = fopen(config_file.c_str(), "r");
    if (fh == NULL)
    {
        ROS_WARN("config_file dosen't exist; wrong config_file path");
        ROS_BREAK();
        return;
    }
    fclose(fh);

    // 创建一个 YAML 对象
    YAML::Node fsSettings;

    try{
        fsSettings = YAML::LoadFile(config_file);
    } catch(YAML::BadFile &e) {
        std::cerr << "ERROR: Wrong path to settings" << std::endl;
    }

    POSE_TOPIC = fsSettings["pose_topic"].as<std::string>();
    TWIST_TOPIC = fsSettings["twist_topic"].as<std::string>();
    IMU_TOPIC = fsSettings["imu_topic"].as<std::string>();
    LEG_TOPIC = fsSettings["leg_topic"].as<std::string>();

    LEG_OFFSET_X = fsSettings["leg_offset_x"].as<double>();
    LEG_OFFSET_Y = fsSettings["leg_offset_y"].as<double>();
    THIGH_OFFSET = fsSettings["thigh_offset"].as<double>();
    THIGH_LENGTH = fsSettings["thigh_length"].as<double>();
    CALF_LENGTH = fsSettings["calf_length"].as<double>();
}
