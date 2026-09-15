#ifndef STATE_ESTIMATION_PIPELINE_HPP
#define STATE_ESTIMATION_PIPELINE_HPP

// C++ standart libraries
#include <ros/ros.h>
#include <eigen3/Eigen/Eigen>
#include <math.h>

// Custom Headers
#include "state_estimation/state_estimation_pipeline.hpp"
#include "state_estimation/state_estimation_params.hpp"
#include "state_estimation/state_estimation_ukf.hpp"
#include "state_estimation_ukf_config.hpp"

// Custom Messages
#include "common_msgs/CarMotor.h"
#include "common_msgs/CarTireInfo.h"
#include "common_msgs/StateVector.h"
#include "sensor_msgs/Imu.h"
#include "common_msgs/CarVelocity.h"
#include "common_msgs/CarAcceleration.h"
#include "common_msgs/ControlCmd.h"
#include "common_msgs/StaPositionInfo.h"

// ROS messages
#include "std_msgs/Float32.h"
#include "std_msgs/Int16.h"
#include "std_msgs/Header.h"
#include "std_msgs/Float64MultiArray.h"
#include "nav_msgs/Odometry.h"
#include "sensor_msgs/NavSatFix.h"
#include "geometry_msgs/Vector3Stamped.h"
#include "geometry_msgs/TwistWithCovarianceStamped.h"
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <tf2_ros/transform_listener.h>


class StateEstimation {

    public:
        StateEstimation(ros::NodeHandle &nh);
        void run();

        //Setters
        void setImuMeasure(const sensor_msgs::Imu &odom3d);
        void setWheelSpeedsMeasure(const common_msgs::CarMotor &wheel_speeds);
        void setTorqueMeasure(const common_msgs::CarMotor &torque);
        void setSteeringMeasure(const common_msgs::ControlCmd &steering);
        void setGpsPosition(const sensor_msgs::NavSatFix &gpsPosition);
        void setGpsVelocity(const geometry_msgs::TwistWithCovarianceStamped &gpsVelocity);
        void setStaSteering(const common_msgs::StaPositionInfo &steering);

        EIGEN_MAKE_ALIGNED_OPERATOR_NEW;

        //Getters
        common_msgs::CarVelocity getCarVelocity();
        common_msgs::CarTireInfo getCarTireInfo();
        common_msgs::TireInfo convertToTireInfoMsg(TireInfoStruct tire);
        std_msgs::Float32 getPMatrixTrace();
        std_msgs::Float32 getPMatrixDet();
        geometry_msgs::Vector3Stamped getWheelSpeeds();
        geometry_msgs::Vector3Stamped getSteering();
        common_msgs::StateVector getTorque();
        geometry_msgs::Vector3Stamped getStaSteering();
        nav_msgs::Odometry getGpsPositionOdometry();
        common_msgs::StateVector getMeasuredValues();
        common_msgs::StateVector getPredictedValues();
        common_msgs::StateVector get_pipeline_StateVector();
        common_msgs::CarVelocity getCalculatedGpsVelocity();
        sensor_msgs::NavSatFix getPureNavSatFix();

        // NEES/NIS consistency debug topics
        std_msgs::Float64MultiArray getStateCovarianceFull();
        std_msgs::Float64MultiArray getInnovation();
        std_msgs::Float64MultiArray getInnovationCovarianceFull();

    private:
        void loadCarParameters(ros::NodeHandle &nh);
        void loadEkfParameters(ros::NodeHandle &nh);
        void convertGpsCoordinates();
        double median_filter_scalar(std::deque<double> &buffer, double new_value, int window_size);

        // Yaw observer (gyro + Xsens + GPS-COG)
        void predictYaw(double wz, double dt);
        void correctYawXsens(double xsensYaw);
        void correctYawGpsCog(double vMapX, double vMapY, double speed);

        Params _params;
        KalmanFilter _kalmanFilter;
        ros::Time _lastUpdateTime; // last _kalmanFilter timestamp seen, used to integrate _distanceIncrement
        Eigen::Vector2d _gpsInitial;
        bool _use_gps_velocities = false;
        bool _init = true;
        double _wz = 0;
        double _initialYaw = 0;
        double _currentXsensYaw = 0;

        // Yaw observer state (relative to the _initialYaw latch)
        double _yawHat = 0.0;        // fast, driven by gyro + Xsens
        double _yawOffsetHat = 0.0;  // slow, GPS-COG only, kept separate so it doesnt fight _yawHat
        double _gyroBiasHat = 0.0;
        bool _yawFilterInitialized = false;
        ros::Time _lastYawPredictTime;

        // Yaw observer gains, no RTK so gps is noisy -> low gain and gated to a higher speed
        double _kXsensYaw = 0.02;
        double _kXsensBias = 0.0005;
        double _kYawOffset = 0.08;
        double _cogSpeedThresh = 5.0;
        double _lastDx;
        double _lastDy;
        float _distanceIncrement = 0;
        std::string _velocityFrameId;
        std_msgs::Header _ahrsHeader;
        std_msgs::Float32 _velocityKph;
        std_msgs::Float32 _yawRate;

        Eigen::Matrix<double,UKF_CONFIG::N,1> _x;
        Eigen::Matrix<double,UKF_CONFIG::M,1>  _y;
        Eigen::Matrix<double,UKF_CONFIG::M,1> _y_hat;
        Eigen::Matrix<double,UKF_CONFIG::N,UKF_CONFIG::N> _P;

        //Messages to publish
        common_msgs::CarVelocity _carVelocity;
        common_msgs::CarAcceleration _carAcc;
        common_msgs::CarTireInfo _carTireInfo;
        common_msgs::CarVelocity _PcarVelocity;
        std_msgs::Float32 _traceP;
        std_msgs::Float32 _detP;
        common_msgs::CarMotor _PWheelSpeeds;

        bool _use_median_filters;

        // Per-value buffers for IMU
        std::deque<double> _IMU_BUFFER_ac_x;
        std::deque<double> _IMU_BUFFER_ac_y;
        std::deque<double> _IMU_BUFFER_wz;
        int _IMU_window_size;

        // Per-value buffers for wheel speeds
        std::deque<double> _WheelSpeeds0_BUFFER;
        std::deque<double> _WheelSpeeds1_BUFFER;
        std::deque<double> _WheelSpeeds2_BUFFER;
        std::deque<double> _WheelSpeeds3_BUFFER;
        int _WheelSpeeds_window_size;

        // Per-value buffers for torque
        std::deque<double> _Torque0_BUFFER;
        std::deque<double> _Torque1_BUFFER;
        std::deque<double> _Torque2_BUFFER;
        std::deque<double> _Torque3_BUFFER;
        int _Torque_window_size;


        //Debug Messages
        Eigen::Matrix<double,4,1> _wheelSpeedsVector;
        Eigen::Matrix<double,4,1> _torqueVector;
        Eigen::Matrix<double,2,1> _steeringVector;
        
        geometry_msgs::Vector3Stamped _wheelSpeeds;
        geometry_msgs::Vector3Stamped _torque;
        geometry_msgs::Vector3Stamped _steering;
        geometry_msgs::Vector3Stamped _staSteering;
        nav_msgs::Odometry _gpsOdom;

        // GPS Velocity Calculation States
        ros::Time _lastGpsStamp;
        double _lastGpsX = 0.0;
        double _lastGpsY = 0.0;
        bool _isGpsInitForVelocity = false;
        
        // Filter State & Tuning
        double _filteredGpsVx = 0.0;
        double _filteredGpsVy = 0.0;
        const double _minGpsDt = 0.05;       // Minimum 50ms temporal throttling

        // GPS longitude and latitude frame -- position is dead-reckoned from the UKF's
        // vx/vy states, rotated into the global frame with the corrected yaw observer
        // (_yawHat + _yawOffsetHat), not a free-running integration of raw wz.
        bool _pureNavInitialized = false;
        double _anchorLat = 0.0;
        double _anchorLon = 0.0;
        double _rawX = 0.0;
        double _rawY = 0.0;
        ros::Time _lastKekfTime;

        Eigen::Matrix<double,3,1> _gpsCoordinates;
        Eigen::Matrix<double,3,1> _gpsCoordinatesConverted;

        ros::NodeHandle _nh;
};

#endif