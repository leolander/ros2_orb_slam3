/*
* Originally adapted from ORB-SLAM3: Examples/ROS/src/ros_stereo.cc
* Author: Azmyin Md. Kamal
* Version: 1.1
* Date: 01/01/2024
* Compatible for ROS2 Humble
*/

#include "ros2_orb_slam3/common.hpp"

//* Constructor
StereoMode::StereoMode() : Node("stereo_node_cpp")
{
    homeDir = getenv("HOME");
    RCLCPP_INFO(this->get_logger(), "\nORB-SLAM3-V1 STEREO NODE STARTED");

    // Declare parameters with defaults
    this->declare_parameter("node_name_arg", "stereo_node");
    this->declare_parameter("voc_file_arg", homeDir + "/" + packagePath + "orb_slam3/Vocabulary/ORBvoc.txt.bin");
    this->declare_parameter("settings_file_path_arg", homeDir + "/" + packagePath + "orb_slam3/config/Stereo/");
    this->declare_parameter("config_file_name", "EuRoC"); // Added parameter for config file
    this->declare_parameter("sub_img_left_topic", "/stereo_py_driver/img_left_msg");
    this->declare_parameter("sub_img_right_topic", "/stereo_py_driver/img_right_msg");
    this->declare_parameter("sub_timestep_topic", "/stereo_py_driver/timestep_msg");
    
    // Get parameters
    nodeName = this->get_parameter("node_name_arg").as_string();
    vocFilePath = this->get_parameter("voc_file_arg").as_string();
    settingsFilePath = this->get_parameter("settings_file_path_arg").as_string();
    std::string configFileName = this->get_parameter("config_file_name").as_string();
    subImgLeftMsgName = this->get_parameter("sub_img_left_topic").as_string();
    subImgRightMsgName = this->get_parameter("sub_img_right_topic").as_string();
    subTimestepMsgName = this->get_parameter("sub_timestep_topic").as_string();

    RCLCPP_INFO(this->get_logger(), "nodeName %s", nodeName.c_str());
    RCLCPP_INFO(this->get_logger(), "voc_file %s", vocFilePath.c_str());

    // Initialize VSLAM directly using parameters
    initializeVSLAM(configFileName);

    subImgLeftMsg_subscription_ = this->create_subscription<sensor_msgs::msg::Image>(subImgLeftMsgName, 1, std::bind(&StereoMode::ImgLeft_callback, this, _1));
    subImgRightMsg_subscription_ = this->create_subscription<sensor_msgs::msg::Image>(subImgRightMsgName, 1, std::bind(&StereoMode::ImgRight_callback, this, _1));
    subTimestepMsg_subscription_ = this->create_subscription<std_msgs::msg::Float64>(subTimestepMsgName, 1, std::bind(&StereoMode::Timestep_callback, this, _1));

    RCLCPP_INFO(this->get_logger(), "Waiting for image data ......");
}

StereoMode::~StereoMode()
{   
    if(pAgent) pAgent->Shutdown();
}

void StereoMode::initializeVSLAM(std::string configName){
    if (vocFilePath == "file_not_set" || settingsFilePath == "file_not_set")
    {
        RCLCPP_ERROR(get_logger(), "Please provide valid voc_file and settings_file paths");       
        rclcpp::shutdown();
    } 
    
    std::string fullSettingsPath = settingsFilePath + configName + ".yaml";

    RCLCPP_INFO(this->get_logger(), "Path to settings file: %s", fullSettingsPath.c_str());
    
    sensorType = ORB_SLAM3::System::STEREO;
    enablePangolinWindow = true;
    
    pAgent = new ORB_SLAM3::System(vocFilePath, fullSettingsPath, sensorType, enablePangolinWindow);
    std::cout << "StereoMode node initialized" << std::endl;
}

void StereoMode::Timestep_callback(const std_msgs::msg::Float64& time_msg){
    timeStep = time_msg.data;
}

void StereoMode::ImgLeft_callback(const sensor_msgs::msg::Image& msg)
{
    cv_bridge::CvImagePtr cv_ptr;
    try {
        cv_ptr = cv_bridge::toCvCopy(msg);
        std::lock_guard<std::mutex> lock(mImu);
        imLeft = cv_ptr->image.clone();
        bLeftReceived = true;
        if (bRightReceived) {
            pAgent->TrackStereo(imLeft, imRight, timeStep);
            bLeftReceived = false;
            bRightReceived = false;
        }
    }
    catch (cv_bridge::Exception& e) {
        RCLCPP_ERROR(this->get_logger(),"Error reading image");
    }
}

void StereoMode::ImgRight_callback(const sensor_msgs::msg::Image& msg)
{
    cv_bridge::CvImagePtr cv_ptr;
    try {
        cv_ptr = cv_bridge::toCvCopy(msg);
        std::lock_guard<std::mutex> lock(mImu);
        imRight = cv_ptr->image.clone();
        bRightReceived = true;
        if (bLeftReceived) {
            pAgent->TrackStereo(imLeft, imRight, timeStep);
            bLeftReceived = false;
            bRightReceived = false;
        }
    }
    catch (cv_bridge::Exception& e) {
        RCLCPP_ERROR(this->get_logger(),"Error reading image");
    }
}

int main(int argc, char **argv){
    rclcpp::init(argc, argv);
    auto node = std::make_shared<StereoMode>(); 
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
