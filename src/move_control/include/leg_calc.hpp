#pragma once

#include "step.hpp"
#include <Eigen/Dense>
#include <Eigen/src/Core/Matrix.h>
#include <chrono>
#include <ctime>
#include <geometry_msgs/msg/point.hpp>
#include <kdl/jacobian.hpp>
#include <memory>
#include <rclcpp/parameter.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp/subscription.hpp>
#include <tuple>
#include <visualization_msgs/msg/marker.hpp>
#include <kdl/chain.hpp>
#include <kdl/chaindynparam.hpp>
#include <kdl/frames.hpp>
#include <kdl/jntarray.hpp>
#include <kdl/tree.hpp>
#include <kdl_parser/kdl_parser.hpp>
#include <kdl/chainfksolverpos_recursive.hpp>
#include <kdl/chainiksolvervel_pinv.hpp>   // ← 你缺的就是它
#include <kdl/chainiksolverpos_lma.hpp>
#include <kdl/chainjnttojacdotsolver.hpp>

class LegCalc{
public:
    LegCalc(KDL::Chain &chain);
    ~LegCalc();

    void set_init_joint_pos(const Eigen::Vector3d init_joint_pos);

    Eigen::Vector3d joint_pos(const Eigen::Vector3d &foot_pos,int  *result,const Eigen::Vector3d &cur_joint_pos); 

    //int joint_pos(KDL::JntArray &joint_rad, KDL::Vector &foot_pos,KDL::JntArray &result);
    Eigen::Vector3d joint_pos(const Eigen::Vector3d &foot_pos,int  *result);       //稍后需要在线安装IK求解器（手推的解析求解器或者数值迭代器）

    Eigen::Vector3d joint_vel(const Eigen::Vector3d &joint_rad, const Eigen::Vector3d &foot_vel);

    Eigen::Vector3d joint_torque_dynamic(const Eigen::Vector3d &joint_rad, const Eigen::Vector3d &joint_omega, const Eigen::Vector3d &foot_acc);

    Eigen::Vector3d joint_torque_foot_force(const Eigen::Vector3d &joint_rad,const Eigen::Vector3d &foot_force);    //由足端期望力计算的关节力矩

    Eigen::Vector3d foot_force(const Eigen::Vector3d &joint_rad,const Eigen::Vector3d &joint_torque,const Eigen::Vector3d &forward_torque=Vector3D(0.0,0.0,0.0));

    Eigen::Vector3d foot_vel(const Eigen::Vector3d &joint_rad, const Eigen::Vector3d &joint_omega);
    
    Eigen::Vector3d foot_pos(const Eigen::Vector3d& joint_rad);

    void set_joint_pd(int index,double kp,double kd);

    void get_joint_pd(int index,double &kp,double &kd);

private:

    Eigen::Vector3d joint_acc(const Eigen::Vector3d &joint_rad, const Eigen::Vector3d &joint_vel,Eigen::Vector3d foot_acc);

    Eigen::Matrix<double, 3, 3> get_3x3_jacobian_(const KDL::Jacobian &full_jacobian);    //从KDL库中求出我们感兴趣的3*3位置雅可比矩阵

    KDL::Chain chain;
    KDL::ChainFkSolverPos_recursive fk_solver;  //关节位置->足端位置
    KDL::ChainJntToJacSolver jacobain_solver;        //求解雅可比矩阵
    KDL::ChainJntToJacDotSolver jdot_solver;         //求解dJdq
    KDL::ChainIkSolverVel_pinv vel_solver;          //
    KDL::ChainIkSolverPos_LMA ik_pos_solver;    //计算期望关节位置
    KDL::ChainDynParam dynamin_solver;         //关节运动状态->关节力矩
    
    //计算数据缓存区
    KDL::JntSpaceInertiaMatrix M;
    KDL::JntArray C;
    KDL::JntArray G;
    KDL::Jacobian temp_jacobain;

    KDL::JntArray last_exp_joint_pos;

    KDL::JntArray _temp_joint3_array;
    KDL::JntArray _temp2_joint3_array;

    KDL::JntArrayVel _temp_joint3_vel_array;
    KDL::Twist _temp_jdot_qd;

    Eigen::Vector3d pos_offset{0.0,0.0,0.0};

    double wheel_radius{0.065};
};
