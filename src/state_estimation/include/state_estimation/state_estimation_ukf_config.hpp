#ifndef STATE_ESTIMATION_UKF_CONFIG_HPP
#define STATE_ESTIMATION_UKF_CONFIG_HPP

namespace UKF_CONFIG {
    constexpr int N = 15; // Dimension of state variables 
    constexpr int M = 9; // Dimension of measurement variables
    constexpr int U = 4; // Dimension of control inputs
    constexpr int L = 2 * N + 1; // Number of sigma points
}

#endif