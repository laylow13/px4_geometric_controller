//
// Created by lay on 24-6-3.
//

#ifndef BUILD_DIRTY_DERIVATIVE_HPP
#define BUILD_DIRTY_DERIVATIVE_HPP

#include <iostream>
#include <type_traits>
#include <vector>
#include <Eigen/Dense>

// 判断是否为Eigen类型的辅助模板
template <typename T>
struct is_eigen_vector : std::false_type {};

template <typename Scalar, int Rows, int Cols, int Options, int MaxRows, int MaxCols>
struct is_eigen_vector<Eigen::Matrix<Scalar, Rows, Cols, Options, MaxRows, MaxCols>> : std::true_type {};

// 通用的赋值为0的函数模板
template <typename T>
void assign_zero(T& value) {
    value = 0;
}

template <typename Scalar, int Rows, int Cols, int Options, int MaxRows, int MaxCols>
void assign_zero(Eigen::Matrix<Scalar, Rows, Cols, Options, MaxRows, MaxCols>& value) {
    value.setZero();
}

template<typename T>
class Dirty_derivative {
public:
    Dirty_derivative(int order_, double tau_, double Ts_)
            : tau(tau_), Ts(Ts_), order(order_), it(1) {
        a1 = (2 * tau - Ts) / (2 * tau + Ts);
        a2 = 2 / (2 * tau + Ts);
    }

    T calculate(const T &x) {
        if (it == 1) {
            assign_zero(dot);
            assign_zero(x_d1);
        }

        if (it > order) {
            dot = a1 * dot + a2 * (x - x_d1);
        }

        ++it;
        x_d1 = x;

        return dot;
    }

private:
    double tau;
    double Ts;
    double a1;
    double a2;
    int order;
    int it;
    T dot;
    T x_d1;
};


#endif //BUILD_DIRTY_DERIVATIVE_HPP
