#ifndef STATE_ESTIMATION_HANDLE_HPP
#define STATE_ESTIMATION_HANDLE_HPP

#include <ros/ros.h>
#include "state_estimation/state_estimation_pipeline.hpp"


class StateEstimationHandle {

    public:

        // Constructor
        StateEstimationHandle(ros::NodeHandle &nodeHandle);

        // Methods
        void advertiseToTopics(); 
        void subscribeToTopics();
        void publishToTopics();
        void runAlgorithm();

    private:

        ros::NodeHandle _nodeHandle;
        StateEstimation _stateEstimation;
        bool _debug;

        //Subscribers
        ros::Subscriber _subXsensAccel;
        ros::Subscriber _subDashSteering;
        ros::Subscriber _subWheelSpeeds;
        ros::Subscriber _subMotorTorque;
        ros::Subscriber _subGpsPosition;
        ros::Subscriber _subGpsVelocity;
        ros::Subscriber _subStaSteering;

        //Publishers
        ros::Publisher _pubStateVector;
        ros::Publisher _pubMeasuredValues;
        ros::Publisher _pubPredictedValues;
        ros::Publisher _pubCarVelocity;
        ros::Publisher _pubCarAcceleration;
        ros::Publisher _pubTireInfo;
        ros::Publisher _pubCovarianceCarVelocity;
        ros::Publisher _pubCovarianceTrace;
        ros::Publisher _pubCovarianceWheelSpeeds;
        ros::Publisher _pubWheelSpeeds;
        ros::Publisher _pubSteering;
        ros::Publisher _pubTorque;
        ros::Publisher _pubGpsPositionConverted;
        ros::Publisher _pubStaSteering;
        ros::Publisher _pubGpsPositionOdometry;
        ros::Publisher _pubVelocityKph;
        ros::Publisher _pubEkfPureNavSatFix;

        //Consistency debug topics (NEES/NIS post-processing)
        ros::Publisher _pubStateCovariance;
        ros::Publisher _pubInnovation;
        ros::Publisher _pubInnovationCovariance;

        //Control EV
        ros::Publisher _pubVelocityX;
        ros::Publisher _pubVelocityY;
        ros::Publisher _pubVelocityTheta;

        void ImuCallback(const sensor_msgs::Imu &accel);
        void SteeringCallback(const common_msgs::ControlCmd &steering);
        void WheelSpeedCallback(const common_msgs::CarMotor &wheel_speeds);
        void MotorTorqueCallback(const common_msgs::CarMotor &torque);
        void GpsPositionCallback(const sensor_msgs::NavSatFix &gpsPosition);
        void GpsVelocityCallback(const geometry_msgs::TwistWithCovarianceStamped &gpsVelocity);
        void StaSteeringCallback(const common_msgs::StaPositionInfo &steering);
};

#endif