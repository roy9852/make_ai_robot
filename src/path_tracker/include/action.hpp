#ifndef ACTION_HPP
#define ACTION_HPP

#include <cmath>
#include <vector>
#include <iostream>

#include "misc.hpp"

class Action {
public:
    Action() = default;
    Action(double vx, double wz) : vx(vx), wz(wz) {}
    ~Action() = default;

    std::vector<double> toVector();
    std::vector<double> toVector() const;

    double vx{0}, wz{0};

    // Operator overloads with scalar
    Action operator+(const double& num) const;
    Action operator-(const double& num) const;
    Action operator*(const double& num) const;
    Action operator/(const double& num) const;

    // Operator overloads with Action
    Action operator+(const Action& action) const;
    Action operator-(const Action& action) const;
    Action operator*(const Action& action) const;
    Action operator/(const Action& action) const;

    // Assignment operators
    Action& operator=(const double& num);
    Action& operator=(const Action& action);

    // Compound assignment operators with scalar
    Action& operator+=(const double& num);
    Action& operator-=(const double& num);
    Action& operator*=(const double& num);
    Action& operator/=(const double& num);

    // Compound assignment operators with Action
    Action& operator+=(const Action& action);
    Action& operator-=(const Action& action);
    Action& operator*=(const Action& action);
    Action& operator/=(const Action& action);

    friend std::ostream& operator<<(std::ostream& os, const Action& action);
};

#endif