#ifndef STATE_ESTIMATION_LUGRE_HPP
#define STATE_ESTIMATION_LUGRE_HPP

// Standart C++ libraries
#include <math.h>
#include <future>
#include <vector>
#include <Eigen/StdVector> 

// Custom Headers
#include "state_estimation/state_estimation_params.hpp" 
#include "state_estimation/state_estimation_ukf_config.hpp"

// Define aliases to keep the function signatures readable
using TireArray = std::vector<Tire, Eigen::aligned_allocator<Tire>>;
using TireInfoArray = std::vector<TireInfoStruct, Eigen::aligned_allocator<TireInfoStruct>>;

class Lugre {

    public:
        
        Eigen::Matrix<double,15,1> nonlinearDynamics(TireArray& tireArray, const Eigen::Matrix<double,4,1>& steerAngle, const Eigen::Matrix<double,15,1>& x, const Eigen::Matrix<double,4,1>& u, const Eigen::Vector4d& Fn, double Iz, double m, double dt);
    
        void extractTireInfo(const TireArray& tireArray, const Eigen::Matrix<double, 4, 1>& steerAngle, const Eigen::Matrix<double, 15, 1>& x, const Eigen::Vector4d& FnArray, TireInfoArray& tireInfoArray);

    private:

        double computeOx(const Tire& tire, const double re, const double w, const Eigen::Matrix<double,2,1>& vr, const double L, int tireId);        
        double computeOy(const Tire& tire, const double re, const double w, const Eigen::Matrix<double,2,1>& vr, const double L, int tireId);   
        double computeKi(const double w, const Eigen::Matrix<double,2,1>& vr, const double L, const double re, const double g, const double sig0_i);
        double computeG(const Tire &tire, const Eigen::Matrix<double,2,1> &vr);
        Eigen::Matrix<double,2,1> velocityTransform(const double delta, const double rx, const double ry, const double vx_car, const double vy_car, const double wz);

        //Atributes
        const double _eps = __DBL_EPSILON__;
        uint8_t _currentTireId = 0;

};

#endif