#include "dynamics.hpp"

void Dynamics::SetConfig(rclcpp::Node* node) {
    // Declare parameters
    node->declare_parameter("NX", 4);
    node->declare_parameter("NU", 2);
    node->declare_parameter("ICR", 0.0);
    node->declare_parameter("maxLinVel", 1.5);
    node->declare_parameter("maxAngVel", 1.0);

    // Get parameters
    node->get_parameter("NX", _NX);
    node->get_parameter("NU", _NU);
    node->get_parameter("T", _T);
    node->get_parameter("DT", _DT);
    node->get_parameter("ICR", _ICR);
    node->get_parameter("maxLinVel", _maxLinVel);
    node->get_parameter("maxAngVel", _maxAngVel);
}

void Dynamics::PrintConfig() {
    std::cout << "Dynamics : " << std::endl;
    std::cout << "\tNX : " << _NX << std::endl;
    std::cout << "\tNU : " << _NU << std::endl;
    std::cout << "\tT : " << _T << std::endl;

    std::cout << "\tDT : " << _DT << std::endl;
    std::cout << "\tICR : " << _ICR << std::endl;
    std::cout << "\tmaxLinVel : " << _maxLinVel << std::endl;
    std::cout << "\tmaxAngVel : " << _maxAngVel << std::endl;
}

State Dynamics::UpdateState(const State& prevState, const Action& action) {
    State newState(0, 0, 0, 0);
    double cos_yaw = std::cos(prevState.yaw);
    double sin_yaw = std::sin(prevState.yaw);
    
    newState.x = prevState.x + (action.vx * cos_yaw + action.wz * _ICR * sin_yaw) * _DT;
    newState.y = prevState.y + (action.vx * sin_yaw - action.wz * _ICR * cos_yaw) * _DT;
    newState.yaw = prevState.yaw + action.wz * _DT;
    
    return newState;
}