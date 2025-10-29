#ifndef DYNAMICS_HPP
#define DYNAMICS_HPP

#include <memory>

#include "rclcpp/rclcpp.hpp"

#include "state.hpp"

class Dynamics {
    public:
        Dynamics() = default;
        ~Dynamics() = default;

        void SetConfig(rclcpp::Node* node);
        void PrintConfig();
        State UpdateState(const State& prevState, const Action& action);

        // Getters
        int GetNx() const { return _NX; }
        int GetNu() const { return _NU; }
        double GetT() const { return _T; }
        double GetDT() const { return _DT; }
        double GetMaxLinVel() const { return _maxLinVel; }
        double GetMaxAngVel() const { return _maxAngVel; }

        // Setters
        void SetT(const int& T) { _T = T; }
        void SetDT(const double& DT) { _DT = DT; }

    private:
        int _NX{4};
        int _NU{2};
        int _T{20};
        double _DT{0.1};
        double _ICR{0.0};
        double _maxLinVel{1.5};
        double _maxAngVel{1.0};
};

#endif