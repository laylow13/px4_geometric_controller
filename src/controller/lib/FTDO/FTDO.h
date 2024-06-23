// FTDO.h
#ifndef FTDO_H
#define FTDO_H

#include "eigen3/Eigen/Eigen"
#include "array"

using namespace Eigen;
using namespace std;

// Ref: Sharma, Manmohan, and Indrani Kar. "Finite time disturbance observer based geometric control of quadrotors." IFAC-PapersOnLine 53.1 (2020): 295-300.
// https://doi.org/10.1016/j.ifacol.2020.06.050

template<int N, int Dim>
class FTDO {
public:
    /* Constructor: Initializes the FTDO with specific parameters.
     * @param lambda: Array of gains for each level of the observer,length:order+1.
     * @param L: Lipschitz constant, affects the convergence rate.
     * @param dt: Sampling time or discretization time step.
     */
    FTDO(array<double, N + 1> _lambda, double _L, double _dt);

    // Sets the initial state estimates for the observer.
    // @param z0: Initial estimates of the observer states.
    void set_initial_estimation(array<Vector<double, Dim>, N + 1> _z0);

    // Updates the observer states based on the system output and current state.
    // @param f_x_u: The system output including control inputs.
    // @param x: The current measured or estimated state.
    void update(Vector<double, Dim> _f_x_u, Vector<double, Dim> _x);

    // Retrieves the current estimates of the observer states.
    // @return: Current state estimates.
    void get_estimation(array<Vector<double, Dim>, N + 1> &_z) const;

private:
//    int n;   // The order of the observer
    array<double, N + 1> lambda;  // Gains for each level of the observer
    double L;  // Lipschitz constant
    array<Vector<double, Dim>, N + 1> z;  // Observer states
    array<Vector<double, Dim>, N + 1> v;  // Auxiliary variables for observer dynamics
    double dt; // Discretization time step

    // Helper function to compute the sign of a number.
    Vector<double, Dim> signum(Vector<double, Dim> x) const;
};
#include "FTDO.tpp"
#endif // FTDO_H
