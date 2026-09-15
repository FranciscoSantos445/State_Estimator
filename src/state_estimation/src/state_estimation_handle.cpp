#include "state_estimation/state_estimation_handle.hpp"

static bool new_data = false;
StateEstimationHandle::StateEstimationHandle(ros::NodeHandle &nodeHandle): _nodeHandle(nodeHandle), _stateEstimation(nodeHandle) {
    _debug = nodeHandle.param<bool>("/state_estimation/lugre_ekf/debug", false);
    advertiseToTopics();
    subscribeToTopics();
}

void StateEstimationHandle::advertiseToTopics(){

    _pubStateVector = _nodeHandle.advertise<common_msgs::StateVector>("/estimation/state_estimation/state_vector", 1);
    _pubMeasuredValues = _nodeHandle.advertise<common_msgs::StateVector>("/estimation/state_estimation/measured_values", 1);
    _pubPredictedValues = _nodeHandle.advertise<common_msgs::StateVector>("/estimation/state_estimation/predicted_values", 1);
    _pubCarVelocity = _nodeHandle.advertise<common_msgs::CarVelocity>("/estimation/state_estimation/velocity", 1);
    _pubCarAcceleration = _nodeHandle.advertise<common_msgs::CarAcceleration>("/estimation/state_estimation/acc", 1);

    //Debug topics (only published when lugre_ekf/debug is true)
    if (_debug) {
        _pubCovarianceCarVelocity = _nodeHandle.advertise<common_msgs::CarVelocity>("/estimation/state_estimation/P_velocity", 1);
        _pubCovarianceWheelSpeeds = _nodeHandle.advertise<common_msgs::CarMotor>("/estimation/state_estimation/P_wheel_speeds", 1);
        _pubCovarianceTrace = _nodeHandle.advertise<std_msgs::Float32>("/estimation/state_estimation/P_trace", 1);
        _pubStateCovariance = _nodeHandle.advertise<std_msgs::Float64MultiArray>("/estimation/state_estimation/debug/P_full", 1);
        _pubInnovation = _nodeHandle.advertise<std_msgs::Float64MultiArray>("/estimation/state_estimation/debug/innovation", 1);
        _pubInnovationCovariance = _nodeHandle.advertise<std_msgs::Float64MultiArray>("/estimation/state_estimation/debug/Pyy", 1);

        _pubVelocityKph = _nodeHandle.advertise<std_msgs::Float32>("/estimation/state_estimation/velocity_kph", 1);
        _pubEkfPureNavSatFix = _nodeHandle.advertise<sensor_msgs::NavSatFix>("/estimation/state_estimation/ekf_pure_navsatfix", 1);
        _pubTireInfo = _nodeHandle.advertise<common_msgs::CarTireInfo>("/estimation/state_estimation/Tire_Info", 1);

        _pubSteering = _nodeHandle.advertise<geometry_msgs::Vector3Stamped>("/estimation/state_estimation/debug/steering", 1);
        _pubTorque = _nodeHandle.advertise<common_msgs::StateVector>("/estimation/state_estimation/debug/torque", 1);
        // _pubStaSteering = _nodeHandle.advertise<geometry_msgs::Vector3Stamped>("/estimation/state_estimation/debug/sta_steering", 1);
    }

    //Control EV
    _pubVelocityX = _nodeHandle.advertise<std_msgs::Float64>("/velocity_x", 1);
    _pubVelocityY = _nodeHandle.advertise<std_msgs::Float64>("/velocity_y", 1);
    _pubVelocityTheta = _nodeHandle.advertise<std_msgs::Float64>("/velocity_theta", 1);
}

void StateEstimationHandle::publishToTopics(){
    _pubStateVector.publish(_stateEstimation.get_pipeline_StateVector());
    _pubMeasuredValues.publish(_stateEstimation.getMeasuredValues());
    _pubPredictedValues.publish(_stateEstimation.getPredictedValues());
    _pubCarVelocity.publish(_stateEstimation.getCarVelocity());
    _pubCarAcceleration.publish(_stateEstimation.getCarAcc());
    if (_debug) {
        _pubTireInfo.publish(_stateEstimation.getCarTireInfo());
        _pubCovarianceCarVelocity.publish(_stateEstimation.getPCarVelocity());
        _pubCovarianceTrace.publish(_stateEstimation.getPMatrixTrace());
        _pubCovarianceWheelSpeeds.publish(_stateEstimation.getPMatrixWheelSpeeds());
        _pubVelocityKph.publish(_stateEstimation.getVelocityKph());
        _pubEkfPureNavSatFix.publish(_stateEstimation.getPureNavSatFix());

        //Consistency debug topics (NEES/NIS post-processing)
        _pubStateCovariance.publish(_stateEstimation.getStateCovarianceFull());
        _pubInnovation.publish(_stateEstimation.getInnovation());
        _pubInnovationCovariance.publish(_stateEstimation.getInnovationCovarianceFull());

        //Topics published when debugging the package
        _pubSteering.publish(_stateEstimation.getSteering());
        _pubTorque.publish(_stateEstimation.getTorque());
        // _pubGpsPositionConverted.publish(_stateEstimation.getGpsPositionConverted());
        _pubGpsPositionOdometry.publish(_stateEstimation.getGpsPositionOdometry());
        // _pubStaSteering.publish(_stateEstimation.getStaSteering());
    }

    //Control EV
    _pubVelocityX.publish(_stateEstimation.getVelocityX());
    _pubVelocityY.publish(_stateEstimation.getVelocityY());
    _pubVelocityTheta.publish(_stateEstimation.getVelocityTheta());
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
    // _subGpsVelocity = _nodeHandle.subscribe("/estimation/gps_velocity", 1, &StateEstimationHandle::GpsVelocityCallback, this); commented in can sniffer
    _subStaSteering = _nodeHandle.subscribe("/common/can_sniffer/sta_position", 1, &StateEstimationHandle::StaSteeringCallback, this);
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

void StateEstimationHandle::StaSteeringCallback(const common_msgs::StaPositionInfo &steering) {
    _stateEstimation.setStaSteering(steering);
}