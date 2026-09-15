#include "state_estimation/state_estimation_handle.hpp"


static bool new_data = false;
StateEstimationHandle::StateEstimationHandle(ros::NodeHandle &nodeHandle): _nodeHandle(nodeHandle), _stateEstimation(nodeHandle) {
    advertiseToTopics();
    subscribeToTopics();
}

void StateEstimationHandle::advertiseToTopics(){
    _pubStateVector = _nodeHandle.advertise<common_msgs::StateVector>("/estimation/state_estimation/state_vector", 1);
    _pubMeasuredValues = _nodeHandle.advertise<common_msgs::StateVector>("/estimation/state_estimation/measured_values", 1);
    _pubPredictedValues = _nodeHandle.advertise<common_msgs::StateVector>("/estimation/state_estimation/predicted_values", 1);
    _pubCarVelocity = _nodeHandle.advertise<common_msgs::CarVelocity>("/estimation/state_estimation/velocity", 1);
    _pubTireInfo = _nodeHandle.advertise<common_msgs::CarTireInfo>("/estimation/state_estimation/Tire_Info", 1);
    _pubCovarianceTrace = _nodeHandle.advertise<std_msgs::Float32>("/estimation/state_estimation/P_trace", 1);
    _pubCovarianceDet = _nodeHandle.advertise<std_msgs::Float32>("/estimation/state_estimation/P_det", 1);
    _pubEkfPureNavSatFix = _nodeHandle.advertise<sensor_msgs::NavSatFix>("/estimation/state_estimation/ekf_pure_navsatfix", 1);

    //Consistency debug topics (NEES/NIS post-processing)
    _pubStateCovarianceFull = _nodeHandle.advertise<std_msgs::Float64MultiArray>("/estimation/state_estimation/debug/P_full", 1);
    _pubInnovation = _nodeHandle.advertise<std_msgs::Float64MultiArray>("/estimation/state_estimation/debug/innovation", 1);
    _pubInnovationCovarianceFull = _nodeHandle.advertise<std_msgs::Float64MultiArray>("/estimation/state_estimation/debug/Pyy_full", 1);

    //Topics published when debugging the package
    _pubSteering = _nodeHandle.advertise<geometry_msgs::Vector3Stamped>("/estimation/state_estimation/debug/steering", 1);
    _pubTorque = _nodeHandle.advertise<common_msgs::StateVector>("/estimation/state_estimation/debug/torque", 1);
    _pubGpsPositionOdometry = _nodeHandle.advertise<nav_msgs::Odometry>("/estimation/state_estimation/gps_odom", 1);
    // _pubStaSteering = _nodeHandle.advertise<geometry_msgs::Vector3Stamped>("/estimation/state_estimation/debug/sta_steering", 1);
}

void StateEstimationHandle::publishToTopics(){
    _pubStateVector.publish(_stateEstimation.get_pipeline_StateVector());
    _pubMeasuredValues.publish(_stateEstimation.getMeasuredValues());
    _pubPredictedValues.publish(_stateEstimation.getPredictedValues());
    _pubCarVelocity.publish(_stateEstimation.getCarVelocity());
    _pubTireInfo.publish(_stateEstimation.getCarTireInfo());
    _pubCovarianceTrace.publish(_stateEstimation.getPMatrixTrace());
    _pubCovarianceDet.publish(_stateEstimation.getPMatrixDet());

    _pubEkfPureNavSatFix.publish(_stateEstimation.getPureNavSatFix());

    //Consistency debug topics (NEES/NIS post-processing)
    _pubStateCovarianceFull.publish(_stateEstimation.getStateCovarianceFull());
    _pubInnovation.publish(_stateEstimation.getInnovation());
    _pubInnovationCovarianceFull.publish(_stateEstimation.getInnovationCovarianceFull());

    //Topics published when debugging the package
    _pubSteering.publish(_stateEstimation.getSteering());
    _pubTorque.publish(_stateEstimation.getTorque());
    _pubGpsPositionOdometry.publish(_stateEstimation.getGpsPositionOdometry());
    // _pubStaSteering.publish(_stateEstimation.getStaSteering());
}

void StateEstimationHandle::runAlgorithm(){
    if (new_data){
        _stateEstimation.run();
        publishToTopics();
        new_data = false;
    }
}

void StateEstimationHandle::subscribeToTopics(){
    _subXsensAccel = _nodeHandle.subscribe("/estimation/odom3D", 1, &StateEstimationHandle::ImuCallback, this);
    _subDashSteering = _nodeHandle.subscribe("/control/controller/steering_actual", 1, &StateEstimationHandle::SteeringCallback, this);
    _subWheelSpeeds = _nodeHandle.subscribe("/estimation/wheel_speeds", 1, &StateEstimationHandle::WheelSpeedCallback, this);
    _subMotorTorque = _nodeHandle.subscribe("/estimation/motor_torque", 1, &StateEstimationHandle::MotorTorqueCallback, this);
    _subGpsPosition = _nodeHandle.subscribe("/estimation/gps_position", 1, &StateEstimationHandle::GpsPositionCallback, this);
    // _subGpsVelocity = _nodeHandle.subscribe("/estimation/gps_velocity", 1, &StateEstimationHandle::GpsVelocityCallback, this);
}

void StateEstimationHandle::ImuCallback(const sensor_msgs::Imu &odom3d){
    _stateEstimation.setImuMeasure(odom3d);
}

void StateEstimationHandle::SteeringCallback(const common_msgs::ControlCmd &steering){
    _stateEstimation.setSteeringMeasure(steering);
}

void StateEstimationHandle::WheelSpeedCallback(const common_msgs::CarMotor &wheel_speeds){
    _stateEstimation.setWheelSpeedsMeasure(wheel_speeds);
    new_data = true;
}

void StateEstimationHandle::MotorTorqueCallback(const common_msgs::CarMotor &torque){
    _stateEstimation.setTorqueMeasure(torque);
}

void StateEstimationHandle::GpsPositionCallback(const sensor_msgs::NavSatFix &gpsPosition){
    _stateEstimation.setGpsPosition(gpsPosition);
}

void StateEstimationHandle::GpsVelocityCallback(const geometry_msgs::TwistWithCovarianceStamped &gpsVelocity) {
    _stateEstimation.setGpsVelocity(gpsVelocity);
}