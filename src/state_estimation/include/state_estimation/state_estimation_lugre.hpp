#ifndef STATE_ESTIMATION_LUGRE_HPP
#define STATE_ESTIMATION_LUGRE_HPP

#include "state_estimation/state_estimation_params.hpp"
#include <math.h>

struct LinModelMatrixTire {
    Eigen::Matrix<double,6,6> A;
    Eigen::Matrix<double,6,1> B;
    Eigen::Matrix<double,4,6> C;
};

struct LinModelMatrix {
    Eigen::Matrix<double,15,15> A;
    Eigen::Matrix<double,15,4>  B;
    Eigen::Matrix<double,9,15>  C;
};
class Lugre {

    public:
        Lugre() {
            _tireInfoArray.resize(4);
        }
        LinModelMatrix fullCarDynamics( std::vector<Tire> &tireArray, const Eigen::Matrix<double,4,1> &steerAngle, const Eigen::Matrix<double,4,1> &wheelSpeedArray, 
                                      const double vx, const double vy, const double wz, const Eigen::Vector4d &FnArray, const double Iz, const double m);

        TireInfoStruct const & getTireInfo(uint8_t id)const;
    private:
        LinModelMatrixTire tireDynamics( Tire &tire, const double theta, const double w, const double vx, 
                                        const double vy, const double wz, double Fn, const double Iz, const double m, int tireId );
        double computeOx(const Tire &tire, const double rl, const double w, const Eigen::Matrix<double,2,1> &vr, const double L, int tireId );
        double computeOy(const Tire &tire, const double re, const double w, const Eigen::Matrix<double,2,1> &vr, const double L , int tireId );
        double computeKi(const double w, const Eigen::Matrix<double,2,1> &vr, const double L, const double re, const double g, const double sig0_i);
        double computeG(const Tire &tire, const Eigen::Matrix<double,2,1> &vr);
        Eigen::Matrix<double,2,1> velocityTransform(const double delta, const double rx, const double ry, const double vx_car, const double vy_car, const double wz);

        //Atributes
        const double _eps = __DBL_EPSILON__;
        std::vector<TireInfoStruct> _tireInfoArray;
        uint8_t _currentTireId = 0;

};


#endif