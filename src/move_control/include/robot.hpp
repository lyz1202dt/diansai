#pragma once

#include "leg_calc.hpp"
#include "cdc_trans.hpp"

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
#include <array>
#include <thread>
#include <vector>
#include <visualization_msgs/msg/marker.hpp>
#include <visualization_msgs/msg/marker_array.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include "data_pack.h"


class Estimater;

class Robot {
public:
    struct QuinticTrajectory3D {
        std::array<LegStep::QuinticLineParam_t, 3> axis;
        double duration{0.0};
        bool active{false};
    };

    Robot(const std::shared_ptr<rclcpp::Node> node);
    ~Robot();

    void arm_control();

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

    std::unique_ptr<CDCTrans> cdc_trans;    //CDC传输类
    std::unique_ptr<std::thread> usb_event_handle_thread;
    bool exit_cdc_thread{false};

    Eigen::Vector3d final_target_pos;
    Eigen::Vector3d final_target_rad;

private:
    void sync_targets_from_parameters();
    void start_cartesian_motion(double execute_time);
    void start_joint_motion(double execute_time);
    void stop_cartesian_motion();
    void stop_joint_motion();

    QuinticTrajectory3D cartesian_traj_;
    QuinticTrajectory3D joint_traj_;
    std::chrono::steady_clock::time_point cartesian_start_time_;
    std::chrono::steady_clock::time_point joint_start_time_;
    bool cartesian_executing_{false};
    bool joint_executing_{false};
    bool last_cart_trigger_{false};
    bool last_joint_trigger_{false};
    Eigen::Vector3d last_planned_terminal_joint_{Eigen::Vector3d::Zero()};
    bool has_last_planned_terminal_joint_{false};

public:

    bool first_run{true};
    int state{0};
};
