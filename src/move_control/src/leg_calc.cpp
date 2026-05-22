#include "leg_calc.hpp"
#include <chrono>
#include <kdl/chainiksolverpos_lma.hpp>
#include <kdl/frames.hpp>
#include <kdl/jntarray.hpp>
#include <rclcpp/logger.hpp>

using namespace std::chrono_literals;

LegCalc::LegCalc(KDL::Chain& chain)
    : chain(chain)
    , fk_solver(chain)
    , jacobain_solver(chain)
    , jdot_solver(chain)
    , vel_solver(chain)
    , ik_pos_solver(chain, Eigen::Vector<double, 6>(1.0, 1.0, 1.0, 0.0, 0.0, 0.0), 1e-6, 150, 1e-10)
    , dynamin_solver(chain, KDL::Vector(0, 0, -9.81)) {
    _temp_joint3_array.resize(3); // 提前resize需要用到的KDL::JntArray防止运行时频繁申请/释放内存
    _temp2_joint3_array.resize(3);
    last_exp_joint_pos.resize(3);
    temp_jacobain.resize(3);
    _temp_joint3_vel_array.resize(3);

    C.resize(3);
    G.resize(3);
    M.resize(3);
}


LegCalc::~LegCalc() {}

void LegCalc::set_init_joint_pos(const Eigen::Vector3d init_joint_pos)
{
    last_exp_joint_pos(0)=init_joint_pos[0];
    last_exp_joint_pos(1)=init_joint_pos[1];
    last_exp_joint_pos(2)=init_joint_pos[2];
}

Eigen::Matrix<double, 3, 3> LegCalc::get_3x3_jacobian_(const KDL::Jacobian& full_jacobian) // 只关心前三行的映射关系
{
    Eigen::Matrix<double, 3, 3> jacobian_3x3;
    for (int i = 0; i < 3; ++i) {
        for (int j = 0; j < 3; ++j) {
            jacobian_3x3(i, j) = full_jacobian(i, j);
        }
    }
    return jacobian_3x3;
}


Eigen::Vector3d LegCalc::joint_pos(const Eigen::Vector3d& foot_pos, int* result) {
    KDL::Frame frame;
    Eigen::Vector3d temp = foot_pos + pos_offset;
    frame.p.x(temp[0]);
    frame.p.y(temp[1]);
    frame.p.z(temp[2]);
    frame.M = KDL::Rotation::Identity();

    *result = ik_pos_solver.CartToJnt(last_exp_joint_pos, frame, _temp_joint3_array);
    if (*result == 0)                                                                      // 缓存本次计算结果,方便下一次迭代
        last_exp_joint_pos = _temp_joint3_array;
    return {_temp_joint3_array(0), _temp_joint3_array(1), _temp_joint3_array(2)};
}



Eigen::Vector3d LegCalc::joint_vel(const Eigen::Vector3d& joint_rad, const Eigen::Vector3d& foot_vel) {
    _temp_joint3_array(0) = joint_rad[0];
    _temp_joint3_array(1) = joint_rad[1];
    _temp_joint3_array(2) = joint_rad[2];
    jacobain_solver.JntToJac(_temp_joint3_array, temp_jacobain);
    Eigen::Matrix<double, 3, 3> jacobian = get_3x3_jacobian_(temp_jacobain);
    return jacobian.inverse() * foot_vel;
}


/**
    @brief 计算关节角加速度
    @param joint_rad 关节角度向量
    @param joint_vel 关节角速度
    @param foot_acc  期望的足端加速度
    @return 关节角加速度向量
 */
Eigen::Vector3d LegCalc::joint_acc(const Eigen::Vector3d& joint_rad, const Eigen::Vector3d& joint_vel, Eigen::Vector3d foot_acc) {
    _temp_joint3_array.data          = joint_rad;
    _temp2_joint3_array.data         = joint_vel;
    _temp_joint3_vel_array.q.data    = joint_rad;
    _temp_joint3_vel_array.qdot.data = joint_vel;

    // 计算雅可比矩阵J
    jacobain_solver.JntToJac(_temp_joint3_array, temp_jacobain);
    jdot_solver.JntToJacDot(_temp_joint3_vel_array, _temp_jdot_qd);

    Eigen::Matrix3d Jac = get_3x3_jacobian_(temp_jacobain);
    Vector3D jdot_dq_eigen;
    for (int i = 0; i < 3; i++) {
        jdot_dq_eigen[i] = _temp_jdot_qd(i);
    }
    return Jac.completeOrthogonalDecomposition().solve(foot_acc - jdot_dq_eigen);
}

Eigen::Vector3d
    LegCalc::joint_torque_dynamic(const Eigen::Vector3d& joint_rad, const Eigen::Vector3d& joint_omega, const Eigen::Vector3d& foot_acc) {
    _temp_joint3_array(0)  = joint_rad[0];
    _temp_joint3_array(1)  = joint_rad[1];
    _temp_joint3_array(2)  = joint_rad[2];
    _temp2_joint3_array(0) = joint_omega[0];
    _temp2_joint3_array(1) = joint_omega[1];
    _temp2_joint3_array(2)  = joint_omega[2];
    dynamin_solver.JntToGravity(_temp_joint3_array, G);
    dynamin_solver.JntToCoriolis(_temp_joint3_array, _temp2_joint3_array, C);
    dynamin_solver.JntToMass(_temp_joint3_array, M);

    // 6. 转换 KDL 输出到 Eigen，方便矩阵运算
    Eigen::Matrix<double, 3, 3> M_;
    Eigen::Matrix<double, 3, 1> C_, G_;

    for (int i = 0; i < 3; ++i) {
        C_(i) = C(i);
        G_(i) = G(i);
        for (int j = 0; j < 3; ++j) {
            M_(i, j) = M(i, j);
        }
    }
    // 7. 计算前馈力矩 tau
    return M_ * joint_acc(joint_rad, joint_omega, foot_acc) + C_ + G_;
}

/**
    @brief 足端期望力->计算关节力矩
    @param joint_rad 关节角度
    @param joint_force 关节末端期望力
    @return 关节空间下的力矩
 */
Eigen::Vector3d LegCalc::joint_torque_foot_force(const Eigen::Vector3d& joint_rad, const Eigen::Vector3d& foot_force) {
    _temp_joint3_array(0) = joint_rad[0];
    _temp_joint3_array(1) = joint_rad[1];
    _temp_joint3_array(2) = joint_rad[2];
    jacobain_solver.JntToJac(_temp_joint3_array, temp_jacobain);
    Eigen::Matrix<double, 3, 3> jacobian = get_3x3_jacobian_(temp_jacobain);
    Eigen::Vector3d torque(foot_force(0), foot_force(1), foot_force(2));
    return jacobian.transpose() * torque;
}

/**
    @brief 计算足端受力
    @param joint_rad 当前关节角度
    @param joint_torque 总力矩减去克服重力/科氏力/惯性力剩下的力矩（需要在外部计算）
    @return 笛卡尔坐标系下的足端受力
 */
Eigen::Vector3d
    LegCalc::foot_force(const Eigen::Vector3d& joint_rad, const Eigen::Vector3d& joint_torque, const Eigen::Vector3d& forward_torque) {
    _temp_joint3_array(0) = joint_rad[0];
    _temp_joint3_array(1) = joint_rad[1];
    _temp_joint3_array(2) = joint_rad[2];

    jacobain_solver.JntToJac(_temp_joint3_array, temp_jacobain);
    auto jacobian = get_3x3_jacobian_(temp_jacobain);

    return jacobian.transpose().inverse() * (joint_torque - forward_torque);
}

/**
    @brief 计算足端速度
    @param joint_rad 当前关节角度
    @param joint_omega 当前关节角速度
    @return 当前足端速度
 */
Eigen::Vector3d LegCalc::foot_vel(const Eigen::Vector3d& joint_rad, const Eigen::Vector3d& joint_omega) {
    _temp_joint3_array(0) = joint_rad[0];
    _temp_joint3_array(1) = joint_rad[1];
    _temp_joint3_array(2) = joint_rad[2];

    jacobain_solver.JntToJac(_temp_joint3_array, temp_jacobain);

    auto jacobian = get_3x3_jacobian_(temp_jacobain); // 提取雅可比矩阵中与位置相关的部分
    Eigen::Vector3d dq(joint_omega(0), joint_omega(1), joint_omega(2));
    return jacobian * dq;
}

/**
    @brief 计算足端位置
    @param joint_rad 关节角度向量
    @return 当前足端位置
 */
Eigen::Vector3d LegCalc::foot_pos(const Eigen::Vector3d& joint_rad) {
    KDL::Frame frame;
    _temp_joint3_array(0) = joint_rad[0]; // 避免运行时动态分配内存，提高效率
    _temp_joint3_array(1) = joint_rad[1];
    _temp_joint3_array(2) = joint_rad[2];

    int fk_result = fk_solver.JntToCart(_temp_joint3_array, frame);

    Eigen::Vector3d temp;
    temp[0] = frame.p.x();
    temp[1] = frame.p.y();
    temp[2] = frame.p.z();

    return temp - pos_offset; // temp是在机器人坐标系下的足端位置，要转换成支撑相中型点的坐标输出
}

