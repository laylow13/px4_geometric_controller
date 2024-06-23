#include "FTDO.h"
#include <algorithm>
#include <cmath>

template<int N, int Dim>
FTDO<N, Dim>::FTDO(array<double, N + 1> _lambda, double _L, double _dt)
        : lambda(_lambda), L(_L), dt(_dt) {
    for (int i = 0; i <= N; ++i) {
        z[i] = Vector<double, Dim>::Zero();
        v[i] = Vector<double, Dim>::Zero();
    }
}

template<int N, int Dim>
void FTDO<N, Dim>::update(Vector<double, Dim> _f_x_u, Vector<double, Dim> _x) {
    v[0] = -lambda[0] * std::pow(L, 1.0 / (N + 1)) *
           (z[0] - _x).array().abs().pow(N / (N + 1)).matrix().cwiseProduct(signum(z[0] - _x)) + z[1];
    for (int i = 1; i < N; ++i) {
        v[i] = -lambda[i] * std::pow(L, 1.0 / (N + 1 - i)) *
               (z[i] - v[i - 1]).array().abs().pow((N - i) / (N - i + 1)).matrix().cwiseProduct(signum(z[i] - v[i - 1]))
               + z[i + 1];
    }
    v[N] = -lambda[N] * L * signum(z[N] - v[N - 1]);

    for (int i = 0; i <= N; ++i) {
        z[i] += v[i] * dt;
    }
    z[0] += _f_x_u * dt;
}

template<int N, int Dim>
void FTDO<N, Dim>::set_initial_estimation(array<Vector<double, Dim>, N + 1> _z0) {
    std::copy(_z0.begin(), _z0.end(), z.begin());
}

template<int N, int Dim>
void FTDO<N, Dim>::get_estimation(array<Vector<double, Dim>, N + 1> &_z) const {
    std::copy(z.begin(), z.end(), _z.begin());
}

template<int N, int Dim>
Vector<double, Dim> FTDO<N, Dim>::signum(Vector<double, Dim> x) const {
    Vector<double, Dim> sign_x;
    for (int i = 0; i < Dim; ++i) {
        if (x(i) > 0) sign_x(i) = 1.0;
        else if (x(i) < 0) sign_x(i) = -1.0;
        else sign_x(i) = 0.0;
    }
    return sign_x;
}
