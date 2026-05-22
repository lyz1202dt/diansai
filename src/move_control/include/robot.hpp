#pragma once

#include "leg_calc.hpp"

#include <Eigen/Dense>
#include <Eigen/src/Core/Matrix.h>
#include <chrono>
#include <ctime>
#include <functional>
#include <memory>
#include <rclcpp/parameter.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp/subscription.hpp>
#include <tuple>

#include <kdl/chain.hpp>
#include <kdl/frames.hpp>
#include <kdl/jntarray.hpp>
#include <kdl/tree.hpp>
#include <kdl_parser/kdl_parser.hpp>

#include <tf2/LinearMath/Matrix3x3.hpp>
#include <tf2/LinearMath/Quaternion.hpp>
#include <tf2_ros/transform_broadcaster.h>

#include <map>
#include <vector>
#include <visualization_msgs/msg/marker.hpp>
#include <visualization_msgs/msg/marker_array.hpp>
#include <sensor_msgs/msg/joint_state.hpp>

#define driver true
#define sim    false

class Estimater;

class Robot {
public:
    Robot(const std::shared_ptr<rclcpp::Node> node);
    ~Robot();

    // ROS2通信相关话题
    rclcpp::Node::SharedPtr node_;
    rclcpp::TimerBase::SharedPtr control_timer;
    rclcpp::node_interfaces::OnSetParametersCallbackHandle::SharedPtr param_server_;
    rclcpp::SyncParametersClient::SharedPtr robot_description_param_;

    // 可视化相关
    rclcpp::TimerBase::SharedPtr ui_update_timer;
    rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr rviz_joint_publisher;
    std::vector<std::string> joint_names = {"joint1", "joint2", "joint3"};
    sensor_msgs::msg::JointState joint_display_msg;


    // 解算部分
    KDL::Tree tree;
    std::string urdf_xml;
    KDL::Chain arm_chain;
    std::shared_ptr<LegCalc> arm_calc;

    //数据缓存
    Eigen::Vector3d arm_joint_pos,arm_joint_vel;
};
