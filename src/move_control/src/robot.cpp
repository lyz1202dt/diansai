#include "robot.hpp"
#include "step.hpp"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <iostream>
#include <memory>
#include <rclcpp/create_timer.hpp>
#include <rclcpp/logging.hpp>

using namespace std::chrono_literals;

namespace {
using QuinticParam = LegStep::QuinticLineParam_t;

void set_quintic(
    QuinticParam& seg,
    double p0,
    double v0,
    double a0,
    double pT,
    double vT,
    double aT,
    double dt)
{
    const double T = std::max(dt, 1e-3);
    const double T2 = T * T;
    const double T3 = T2 * T;
    const double T4 = T3 * T;
    const double T5 = T4 * T;

    seg.a = p0;
    seg.b = v0;
    seg.c = 0.5 * a0;
    seg.d = (10.0 * (pT - p0) - (6.0 * v0 + 4.0 * vT) * T - (1.5 * a0 - 0.5 * aT) * T2) / T3;
    seg.e = (-15.0 * (pT - p0) + (8.0 * v0 + 7.0 * vT) * T + (1.5 * a0 - aT) * T2) / T4;
    seg.f = (6.0 * (pT - p0) - (3.0 * v0 + 3.0 * vT) * T - (0.5 * a0 - 0.5 * aT) * T2) / T5;
}

double get_quintic_value(const QuinticParam& line, double time)
{
    return line.a + line.b * time + line.c * time * time + line.d * time * time * time + line.e * time * time * time * time
         + line.f * time * time * time * time * time;
}

double get_quintic_dt(const QuinticParam& line, double time)
{
    return line.b + 2.0 * line.c * time + 3.0 * line.d * time * time + 4.0 * line.e * time * time * time
         + 5.0 * line.f * time * time * time * time;
}

double get_quintic_dtdt(const QuinticParam& line, double time)
{
    return 2.0 * line.c + 6.0 * line.d * time + 12.0 * line.e * time * time + 20.0 * line.f * time * time * time;
}

Robot::QuinticTrajectory3D plan_quintic_trajectory(
    const Eigen::Vector3d& start_pos,
    const Eigen::Vector3d& start_vel,
    const Eigen::Vector3d& target_pos,
    const Eigen::Vector3d& target_vel,
    double duration)
{
    Robot::QuinticTrajectory3D traj;
    traj.duration = std::max(duration, 1e-3);
    traj.active = true;

    for (int i = 0; i < 3; ++i) {
        set_quintic(traj.axis[i], start_pos[i], start_vel[i], 0.0, target_pos[i], target_vel[i], 0.0, traj.duration);
    }
    return traj;
}

void sample_quintic_trajectory(
    const Robot::QuinticTrajectory3D& traj,
    double time,
    Eigen::Vector3d& pos,
    Eigen::Vector3d& vel,
    Eigen::Vector3d& acc)
{
    const double clamped_time = std::clamp(time, 0.0, traj.duration);
    for (int i = 0; i < 3; ++i) {
        pos[i] = get_quintic_value(traj.axis[i], clamped_time);
        vel[i] = get_quintic_dt(traj.axis[i], clamped_time);
        acc[i] = get_quintic_dtdt(traj.axis[i], clamped_time);
    }
}
} // namespace

Robot::Robot(const std::shared_ptr<rclcpp::Node> node) {
    node_ = node;

    joint_display_msg.position.resize(3);
    joint_display_msg.name = joint_names;

    final_target_pos.setZero();
    final_target_rad.setZero();
    arm_joint_pos.setZero();
    arm_joint_vel.setZero();

    node->declare_parameter<double>("target_x", 0.1);
    node->declare_parameter<double>("target_y", 0.0);
    node->declare_parameter<double>("target_z", 0.1);
    node->declare_parameter<double>("joint1", 0.1);
    node->declare_parameter<double>("joint2", 0.0);
    node->declare_parameter<double>("join3", 0.1);
    node->declare_parameter<bool>("start_execut_cart", false);
    node->declare_parameter<bool>("start_execut_joint", false);
    node->declare_parameter<double>("execut_time", 5.0);

    sync_targets_from_parameters();

    param_server_ = node_->add_on_set_parameters_callback([this](const std::vector<rclcpp::Parameter>& params) {
        rcl_interfaces::msg::SetParametersResult result;
        result.successful = true;

        for (const auto& param : params) {
            const auto& name = param.get_name();
            if (name == "execut_time" && param.as_double() <= 0.0) {
                result.successful = false;
                result.reason = "execut_time must be > 0";
                return result;
            }
        }

        for (const auto& param : params) {
            const auto& name = param.get_name();
            if (name == "target_x") {
                final_target_pos[0] = param.as_double();
            } else if (name == "target_y") {
                final_target_pos[1] = param.as_double();
            } else if (name == "target_z") {
                final_target_pos[2] = param.as_double();
            } else if (name == "joint1") {
                final_target_rad[0] = param.as_double();
            } else if (name == "joint2") {
                final_target_rad[1] = param.as_double();
            } else if (name == "join3") {
                final_target_rad[2] = param.as_double();
            }
        }
        return result;
    });

    rviz_joint_publisher = node_->create_publisher<sensor_msgs::msg::JointState>("joint_states", 10);

    robot_description_param_ = std::make_shared<rclcpp::SyncParametersClient>(node_, "/robot_state_publisher");
    auto params = robot_description_param_->get_parameters({"robot_description"});
    urdf_xml = params[0].as_string();
    if (urdf_xml.empty()) {
        RCLCPP_ERROR(node_->get_logger(), "无法读取URDF文件，不能进行动力学计算");
        exit_cdc_thread = true;
        return;
    }

    kdl_parser::treeFromString(urdf_xml, tree);
    tree.getChain("base_link", "link4", arm_chain);
    arm_calc = std::make_shared<LegCalc>(arm_chain);

    control_timer = node->create_wall_timer(10ms, [this]() {
        if (exit_cdc_thread || !cdc_trans) {
            return;
        }
        arm_control();
    });

    ui_update_timer = node_->create_wall_timer(50ms, [this]() {
        joint_display_msg.position[0] = arm_joint_pos[0];
        joint_display_msg.position[1] = arm_joint_pos[1];
        joint_display_msg.position[2] = arm_joint_pos[2];
        joint_display_msg.header.stamp = node_->get_clock()->now();
        rviz_joint_publisher->publish(joint_display_msg);
    });

    cdc_trans = std::make_unique<CDCTrans>();
    cdc_trans->regeiser_recv_cb([this](const uint8_t* data, int size) {
        if (data == nullptr || size != static_cast<int>(sizeof(StatePackage))) {
            return;
        }
        StatePackage* pack = (StatePackage*)(data);
        if (pack->head != 0x02) {
            return;
        }
        arm_joint_pos[0] = pack->joint[0].pos;
        arm_joint_pos[1] = pack->joint[1].pos;
        arm_joint_pos[2] = pack->joint[2].pos;

        arm_joint_vel[0] = pack->joint[0].vel;
        arm_joint_vel[1] = pack->joint[1].vel;
        arm_joint_vel[2] = pack->joint[2].vel;
    });

    if (!cdc_trans->open(0x0483, 0x5740)) {
        exit_cdc_thread = true;
        std::cerr << "[LegDriver] Failed to open USB CDC device\n";
        return;
    }

    usb_event_handle_thread = std::make_unique<std::thread>([this]() {
        while (!exit_cdc_thread) {
            cdc_trans->process_once();
        }
    });
}

Robot::~Robot() {
    exit_cdc_thread = true;
    if (usb_event_handle_thread && usb_event_handle_thread->joinable()) {
        usb_event_handle_thread->join();
    }
}

void Robot::sync_targets_from_parameters()
{
    final_target_pos[0] = node_->get_parameter("target_x").as_double();
    final_target_pos[1] = node_->get_parameter("target_y").as_double();
    final_target_pos[2] = node_->get_parameter("target_z").as_double();

    final_target_rad[0] = node_->get_parameter("joint1").as_double();
    final_target_rad[1] = node_->get_parameter("joint2").as_double();

    final_target_rad[2] = node_->get_parameter("join3").as_double();
}

void Robot::start_cartesian_motion(double execute_time)
{
    if (!arm_calc) {
        return;
    }

    const Eigen::Vector3d ik_seed = has_last_planned_terminal_joint_ ? last_planned_terminal_joint_ : arm_joint_pos;
    arm_calc->set_init_joint_pos(ik_seed);

    const Eigen::Vector3d cur_pos = arm_calc->foot_pos(arm_joint_pos);
    const Eigen::Vector3d cur_vel = arm_calc->foot_vel(arm_joint_pos, arm_joint_vel);

    cartesian_traj_ = plan_quintic_trajectory(cur_pos, cur_vel, final_target_pos, Eigen::Vector3d::Zero(), execute_time);
    cartesian_start_time_ = std::chrono::steady_clock::now();
    cartesian_executing_ = true;

    RCLCPP_INFO(
        node_->get_logger(),
        "开始执行笛卡尔轨迹: start=[%.3f, %.3f, %.3f], target=[%.3f, %.3f, %.3f], T=%.3f",
        cur_pos[0],
        cur_pos[1],
        cur_pos[2],
        final_target_pos[0],
        final_target_pos[1],
        final_target_pos[2],
        execute_time);

    RCLCPP_INFO(
        node_->get_logger(),
        "笛卡尔轨迹IK初值: seed=[%.3f, %.3f, %.3f]",
        ik_seed[0],
        ik_seed[1],
        ik_seed[2]);
}

void Robot::start_joint_motion(double execute_time)
{
    joint_traj_ = plan_quintic_trajectory(arm_joint_pos, arm_joint_vel, final_target_rad, Eigen::Vector3d::Zero(), execute_time);
    joint_start_time_ = std::chrono::steady_clock::now();
    joint_executing_ = true;
    last_planned_terminal_joint_ = final_target_rad;
    has_last_planned_terminal_joint_ = true;

    RCLCPP_INFO(
        node_->get_logger(),
        "开始执行关节轨迹: start=[%.3f, %.3f, %.3f], target=[%.3f, %.3f, %.3f], T=%.3f",
        arm_joint_pos[0],
        arm_joint_pos[1],
        arm_joint_pos[2],
        final_target_rad[0],
        final_target_rad[1],
        final_target_rad[2],
        execute_time);
}

void Robot::stop_cartesian_motion()
{
    cartesian_executing_ = false;
    cartesian_traj_.active = false;
    node_->set_parameter(rclcpp::Parameter("start_execut_cart", false));
    RCLCPP_INFO(node_->get_logger(), "笛卡尔轨迹执行完成");
}

void Robot::stop_joint_motion()
{
    joint_executing_ = false;
    joint_traj_.active = false;
    node_->set_parameter(rclcpp::Parameter("start_execut_joint", false));
    RCLCPP_INFO(node_->get_logger(), "关节轨迹执行完成");
}

void Robot::arm_control()
{
    if (first_run) {
        const Eigen::Vector3d default_init_joint_pos(0.0, -0.5, 1.7);
        arm_calc->set_init_joint_pos(default_init_joint_pos);
        last_planned_terminal_joint_ = default_init_joint_pos;
        has_last_planned_terminal_joint_ = true;
        first_run = false;
    }

    sync_targets_from_parameters();

    const bool start_cart = node_->get_parameter("start_execut_cart").as_bool();
    const bool start_joint = node_->get_parameter("start_execut_joint").as_bool();
    const double execute_time = std::max(node_->get_parameter("execut_time").as_double(), 1e-3);

    if (start_cart && !last_cart_trigger_ && !cartesian_executing_ && !joint_executing_) {
        start_cartesian_motion(execute_time);
    }
    if (start_joint && !last_joint_trigger_ && !joint_executing_ && !cartesian_executing_) {
        start_joint_motion(execute_time);
    }

    last_cart_trigger_ = start_cart;
    last_joint_trigger_ = start_joint;

    Eigen::Vector3d desired_joint_pos = arm_joint_pos;
    Eigen::Vector3d desired_joint_vel = Eigen::Vector3d::Zero();

    if (cartesian_executing_ && cartesian_traj_.active) {
        const double elapsed = std::chrono::duration<double>(std::chrono::steady_clock::now() - cartesian_start_time_).count();
        Eigen::Vector3d cart_pos = Eigen::Vector3d::Zero();
        Eigen::Vector3d cart_vel = Eigen::Vector3d::Zero();
        Eigen::Vector3d cart_acc = Eigen::Vector3d::Zero();
        sample_quintic_trajectory(cartesian_traj_, elapsed, cart_pos, cart_vel, cart_acc);

        int ik_result = -1;
        desired_joint_pos = arm_calc->joint_pos(cart_pos, &ik_result);
        if (ik_result != 0) {
            RCLCPP_ERROR(
                node_->get_logger(),
                "笛卡尔轨迹IK失败，停止执行。target=[%.3f, %.3f, %.3f], ik_result=%d",
                cart_pos[0],
                cart_pos[1],
                cart_pos[2],
                ik_result);
            stop_cartesian_motion();
            desired_joint_pos = arm_joint_pos;
            desired_joint_vel.setZero();
        } else {
            desired_joint_vel = arm_calc->joint_vel(desired_joint_pos, cart_vel);
            if (elapsed >= cartesian_traj_.duration) {
                last_planned_terminal_joint_ = desired_joint_pos;
                has_last_planned_terminal_joint_ = true;
                stop_cartesian_motion();
            }
        }
    } else if (joint_executing_ && joint_traj_.active) {
        const double elapsed = std::chrono::duration<double>(std::chrono::steady_clock::now() - joint_start_time_).count();
        Eigen::Vector3d joint_pos = Eigen::Vector3d::Zero();
        Eigen::Vector3d joint_vel = Eigen::Vector3d::Zero();
        Eigen::Vector3d joint_acc = Eigen::Vector3d::Zero();
        sample_quintic_trajectory(joint_traj_, elapsed, joint_pos, joint_vel, joint_acc);
        desired_joint_pos = joint_pos;
        desired_joint_vel = joint_vel;

        if (elapsed >= joint_traj_.duration) {
            stop_joint_motion();
        }
    }

    TargetPackage pack{};
    pack.head = 0x01;
    for (int i = 0; i < 3; ++i) {
        pack.joint[i].pos = static_cast<float>(desired_joint_pos[i]);
        pack.joint[i].vel = static_cast<float>(desired_joint_vel[i]);
    }
    cdc_trans->send_struct(pack);
}
