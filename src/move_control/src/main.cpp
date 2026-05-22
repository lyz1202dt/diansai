#include <memory>
#include <rclcpp/rclcpp.hpp>
#include "robot.hpp"


int main(int argc,char** argv)
{
    rclcpp::init(argc,argv);
    auto node=std::make_shared<rclcpp::Node>("arm_calc_node");
    Robot robot(node);
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
