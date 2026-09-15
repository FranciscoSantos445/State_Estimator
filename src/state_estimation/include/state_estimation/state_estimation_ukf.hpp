#ifndef STATE_ESTIMATION_EKF_HPP
#define STATE_ESTIMATION_EKF_HPP

// C++ standard library
#include <ros/ros.h>
#include <eigen3/Eigen/Eigen>
#include <Eigen/Dense>
#include <Eigen/StdVector>  
#include "deque"

// Custom messages
#include "common_msgs/StateVector.h"

// Custom headers
#include "state_estimation/state_estimation_params.hpp"
#include "state_estimation/state_estimation_lugre.hpp"
#include "state_estimation/state_estimation_ukf_config.hpp"

class KalmanFilter {
    public:
        KalmanFilter();
        void loadParameters(Params params);
        void loadCovarianceMatrices(Eigen::MatrixXd P, Eigen::MatrixXd Q, Eigen::MatrixXd R);
        void update();

        EIGEN_MAKE_ALIGNED_OPERATOR_NEW;

        //Getters
        Eigen::Matrix<double,UKF_CONFIG::N,1> const & getStateVector() const;
        Eigen::Matrix<double,UKF_CONFIG::N,UKF_CONFIG::N> const & getStateCovariance() const;
        Eigen::Matrix<double,UKF_CONFIG::M,UKF_CONFIG::N> const & getCMatrix() const;
        TireInfoStruct const  getTireInfo(uint8_t id) const;
        Eigen::Matrix<double,UKF_CONFIG::M,1> getMeasurementVector() const;
        Eigen::Matrix<double,UKF_CONFIG::M,1> getPredictedVector() const;
        Eigen::Matrix<double,UKF_CONFIG::M,1> getPredictedVectorPost() const;
        Eigen::Matrix<double,UKF_CONFIG::M,1> const & getInnovation() const;
        Eigen::Matrix<double,UKF_CONFIG::M,UKF_CONFIG::M> const & getInnovationCovariance() const;

        //Setters
        void updateWheelSpeedsRL(const double w_rl);
        void updateWheelSpeedsRR(const double w_rr);
        void updateWheelSpeedsFR(const double w_fr);
        void updateWheelSpeedsFL(const double w_fl);
        void updateInertia(const double accx, const double accy, const double wz);
        void updateSteeringAngle(const double steering_fl, const double steering_fr);
        void updateMotorTorque(const double u_fl, const double u_fr, const double u_rl, const double u_rr);
        void updateGpsVelocity(const double vx, const double vy);

        ros::Time _currentTimeStamp;

    private:
        //Methods
        void computeInitialGuess();
        void calculate_weights();

        // UKF methods
        Eigen::Matrix<double,UKF_CONFIG::N,UKF_CONFIG::N> safeSqrt(const Eigen::Matrix<double,UKF_CONFIG::N,UKF_CONFIG::N>& M);

        void generateSigmaPoints(const Eigen::Matrix<double,UKF_CONFIG::N,1>& x, Eigen::Matrix<double,UKF_CONFIG::N,UKF_CONFIG::N>& P, Eigen::Matrix<double,UKF_CONFIG::N,UKF_CONFIG::L>& Xi);

        Eigen::Matrix<double,UKF_CONFIG::N,1> propagateSigmaPoint(const Eigen::Matrix<double,UKF_CONFIG::N,1>& xi, const Eigen::Vector4d& Fn);

        Eigen::Matrix<double, UKF_CONFIG::M, 1> measurementModel(const Eigen::Matrix<double, UKF_CONFIG::N, 1>& state, const Eigen::Vector4d& Fn) ;

        void updateDynamicLoads();

        //Attributes
        Params _params;
        Lugre _lugre;
        Eigen::Matrix<double,UKF_CONFIG::N,1> _x;
        Eigen::Matrix<double,UKF_CONFIG::N,UKF_CONFIG::N> _P;
        Eigen::Matrix<double,UKF_CONFIG::N,UKF_CONFIG::N> _Q;
        Eigen::Matrix<double,UKF_CONFIG::M,UKF_CONFIG::M> _R;
        Eigen::Matrix<double,4,1> _u;
        Eigen::Matrix<double,UKF_CONFIG::M,1> _y;
        Eigen::Matrix<double,UKF_CONFIG::M,1> _y_hat;
        Eigen::Matrix<double,UKF_CONFIG::M,1> _y_hat_post;
        Eigen::Matrix<double,UKF_CONFIG::M,1> _innovation;
        Eigen::Matrix<double,UKF_CONFIG::M,UKF_CONFIG::M> _Pyy;
        Eigen::Matrix<double,4,1> _st;
        std::vector<TireInfoStruct, Eigen::aligned_allocator<TireInfoStruct>> _tireInfoArray;

        const double _ts = 0.01;

        // UKF tuning
        double _alpha = 1e-1; // must be values between 1 and  1e-4
        double _beta  = 2.0; // beta = 2 is optimal for Gaussian distributions
        double _kappa = 0.0; // normally set to 0 or 3-N, where N is the state dimension

        double _eps = __DBL_EPSILON__;

        double _lambda = 0.0;
        std::array<double, UKF_CONFIG::L> _Wm{};
        std::array<double, UKF_CONFIG::L> _Wc{};
        Eigen::Vector4d _Fn;

        // https://www.google.com/url?sa=t&rct=j&q=&esrc=s&source=web&cd=&ved=2ahUKEwiU94rd9ZqTAxXY2QIHHenOLQoQFnoECBoQAQ&url=https%3A%2F%2Fwww.researchgate.net%2Fprofile%2FMohamed-Mourad-Lafifi%2Fpost%2FHow_to_create_state_transition_function_of_a_AR2_model_for_unscentedKalmanFilter_object_in_MATLAB%2Fattachment%2F5ede7a2a39f1f300016271da%2FAS%253A900211564089344%25401591638570015%2Fdownload%2Fukf.wan_.chapt7_.pdf&usg=AOvVaw0aZ5LuR5sQ4rCkelzwgsVE&opi=89978449
};

#endif