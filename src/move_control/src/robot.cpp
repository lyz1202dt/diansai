#include "robot.hpp"
#include "step.hpp"

#include <chrono>
#include <memory>
#include <rclcpp/create_timer.hpp>
#include <rclcpp/logging.hpp>

using namespace std::chrono_literals;

Robot::Robot(const std::shared_ptr<rclcpp::Node> node){
    node_ = node;

    joint_display_msg.position.resize(3);
    joint_display_msg.name = joint_names;

    param_server_ = node_->add_on_set_parameters_callback([this](const std::vector<rclcpp::Parameter>& params) {
        rcl_interfaces::msg::SetParametersResult result;
        result.successful = true;
        RCLCPP_INFO(node_->get_logger(), "更新参数");

        return result;
    });


    // RVIZ2可视化
    rviz_joint_publisher = node_->create_publisher<sensor_msgs::msg::JointState>("joint_states", 10);

    // 订阅机器人URDF描述文件
    robot_description_param_ = std::make_shared<rclcpp::SyncParametersClient>(node_, "/robot_state_publisher");
    auto params              = robot_description_param_->get_parameters({"robot_description"});
    urdf_xml                 = params[0].as_string();
    if (urdf_xml.empty()) {
        RCLCPP_ERROR(node_->get_logger(), "无法读取URDF文件，不能进行动力学计算");
        return;
    }

    kdl_parser::treeFromString(urdf_xml, tree); // 解析四条腿的KDL树结构
    tree.getChain("base_link", "link4", arm_chain);
    
    // 初始化狗腿解算器，定义足端中性点位置
    arm_calc             = std::make_shared<LegCalc>(arm_chain);
    
    control_timer   = node->create_wall_timer(10ms, [this]() {

    });

    ui_update_timer = node_->create_wall_timer(50ms, [this](){
    joint_display_msg.position[0] = arm_joint_pos[0];
    joint_display_msg.position[1] = arm_joint_pos[1];
    joint_display_msg.position[2] = arm_joint_pos[2];

    joint_display_msg.header.stamp = node_->get_clock()->now();
    rviz_joint_publisher->publish(joint_display_msg);
    });
}

Robot::~Robot() {}
