#ifndef LPF_1ST_HPP
#define LPF_1ST_HPP

class Lpf_1st
{
private:
    double T;
    double dt;
    double y_k;
    double x_k;

public:
    Lpf_1st(double const_T, double sample_T, double y_0 = 0.0f) : T(const_T), dt(sample_T), y_k(y_0){};
    ~Lpf_1st(){};
    double update(double _x_k)
    {
        x_k = _x_k;
        y_k = (1 - dt / T) * y_k + dt / T * x_k;
        return y_k;
    }
};

#endif