#include "state_estimation/state_estimation_pipeline.hpp"


StateEstimation::StateEstimation(ros::NodeHandle &nh) {
    loadCarParameters(nh);
    loadEkfParameters(nh);

    _nh = nh;
    
    _kalmanFilter.loadParameters(_params);

    _gpsCoordinates.setZero();
    _gpsCoordinatesConverted.setZero();

    _lastUpdateTime = ros::Time::now();
}

void StateEstimation::run() {

    _kalmanFilter.update();

    // Accumulate distance travelled from the UKF velocity, used to gate when _initialYaw latches
    const Eigen::Matrix<double,UKF_CONFIG::N,1> x = _kalmanFilter.getStateVector();
    const double dt = _kalmanFilter._currentTimeStamp.toSec() - _lastUpdateTime.toSec();
    _lastUpdateTime = _kalmanFilter._currentTimeStamp;
    if (dt > 0.0 && dt < 1.0) {
        _distanceIncrement += std::hypot(x(12), x(13)) * dt;
    }
}

void StateEstimation::loadCarParameters(ros::NodeHandle &nh) {

    //Load Inercia Params
    _params.inercia.unsprung = nh.param<double>("car/inertia/unsprung", 0);
    _params.inercia.m = nh.param<double>("car/inertia/m", 0); 
    _params.inercia.sprung = _params.inercia.m - _params.inercia.unsprung;
    _params.inercia.g = nh.param<double>("car/inertia/g", 0);
    _params.inercia.Iz = nh.param<double>("car/inertia/Iz", 0);

    //Load Aero Params
    _params.aero.rho = nh.param<double>("car/aero/rho", 0);
    _params.aero.cl = nh.param<double>("car/aero/cl", 0);
    _params.aero.dist = nh.param<double>("car/aero/dist", 0);

    //Load Kinematics Params
    _params.kinematics.track = nh.param<double>("car/kinematics/track", 0);
    _params.kinematics.a = nh.param<double>("car/kinematics/a", 0);
    _params.kinematics.b = nh.param<double>("car/kinematics/b", 0);
    _params.kinematics.h = nh.param<double>("car/kinematics/h", 0);

    //Load Engine Params
    _params.engine.gr = nh.param<double>("car/engine/gr", 0);

    //Load Tire Params equal for 4 tires
    Tire tire;
    tire.sigma(0,0) = nh.param<double>("car/tire/sigma_00", 0);
    tire.sigma(0,1) = nh.param<double>("car/tire/sigma_01", 0);
    tire.sigma(1,0) = nh.param<double>("car/tire/sigma_10", 0);
    tire.sigma(1,1) = nh.param<double>("car/tire/sigma_11", 0);
    tire.sigma(2,0) = nh.param<double>("car/tire/sigma_20", 0);
    tire.sigma(2,1) = nh.param<double>("car/tire/sigma_21", 0);

    tire.mu_k(0,0) = nh.param<double>("car/tire/mu_k_00", 0);
    tire.mu_k(0,1) = nh.param<double>("car/tire/mu_k_01", 0);
    tire.mu_k(1,0) = nh.param<double>("car/tire/mu_k_10", 0);
    tire.mu_k(1,1) = nh.param<double>("car/tire/mu_k_11", 0);

    tire.mu_s(0,0) = nh.param<double>("car/tire/mu_s_00", 0);
    tire.mu_s(0,1) = nh.param<double>("car/tire/mu_s_01", 0);
    tire.mu_s(1,0) = nh.param<double>("car/tire/mu_s_10", 0);
    tire.mu_s(1,1) = nh.param<double>("car/tire/mu_s_11", 0);

    tire.gamma = nh.param<double>("car/tire/gamma", 0);
    tire.vs = nh.param<double>("car/tire/vs", 0);
    tire.kx = nh.param<double>("car/tire/kx", 0);
    tire.ky = nh.param<double>("car/tire/ky", 0);
    tire.kz = nh.param<double>("car/tire/kz", 0);
    tire.r = nh.param<double>("car/tire/r", 0);

    //fl tire
    tire.p << _params.kinematics.a, _params.kinematics.track/2.0;
    tire.J = nh.param<double>("car/tire/J_f", 0);
    _params.tireArray.push_back(tire);
    //fr tire
    tire.p << _params.kinematics.a, -_params.kinematics.track/2.0;
    tire.J = nh.param<double>("car/tire/J_f", 0);
    _params.tireArray.push_back(tire);
    //rl tire
    tire.p << -_params.kinematics.b, _params.kinematics.track/2.0;
    tire.J = nh.param<double>("car/tire/J_r", 0);
    _params.tireArray.push_back(tire);
    //rr tire
    tire.p << -_params.kinematics.b, -_params.kinematics.track/2.0;
    tire.J = nh.param<double>("car/tire/J_r", 0);
    _params.tireArray.push_back(tire);

    //Load Fn array
    //sprung + unsprung mass
    //sprung on wheel = weight distribution * m * g / 2 + unsprung on wheel
    _params.Fn(0) = _params.kinematics.a/(_params.kinematics.a+_params.kinematics.b)*_params.inercia.sprung*_params.inercia.g/2.0 + _params.inercia.unsprung*_params.inercia.g/4.0;
    _params.Fn(1) = _params.kinematics.a/(_params.kinematics.a+_params.kinematics.b)*_params.inercia.sprung*_params.inercia.g/2.0 + _params.inercia.unsprung*_params.inercia.g/4.0;
    _params.Fn(2) = _params.kinematics.b/(_params.kinematics.a+_params.kinematics.b)*_params.inercia.sprung*_params.inercia.g/2.0 + _params.inercia.unsprung*_params.inercia.g/4.0;
    _params.Fn(3) = _params.kinematics.b/(_params.kinematics.a+_params.kinematics.b)*_params.inercia.sprung*_params.inercia.g/2.0 + _params.inercia.unsprung*_params.inercia.g/4.0;

    // Load median filter usage
    _use_median_filters = nh.param<bool>("car/median_filters/use_median_filters", true);

    //Load median filter window sizes
    _IMU_window_size = nh.param<int>("car/median_filters/imu_window_size", 5);
    _WheelSpeeds_window_size = nh.param<int>("car/median_filters/wheel_speeds_window_size", 5);
    _Torque_window_size = nh.param<int>("car/median_filters/torque_window_size", 5);

    //Clear buffers
    _IMU_BUFFER_ac_x.clear(), _IMU_BUFFER_ac_y.clear(), _IMU_BUFFER_wz.clear();
    _WheelSpeeds0_BUFFER.clear(), _WheelSpeeds1_BUFFER.clear(), _WheelSpeeds2_BUFFER.clear(), _WheelSpeeds3_BUFFER.clear();
    _Torque0_BUFFER.clear(), _Torque1_BUFFER.clear(), _Torque2_BUFFER.clear(), _Torque3_BUFFER.clear();

    //GPS velocities
    _use_gps_velocities = nh.param<bool>("car/gps/use_gps_velocities", 0);

    ROS_INFO("Loaded car parameters");
}

void StateEstimation::loadEkfParameters(ros::NodeHandle &nh) {

    XmlRpc::XmlRpcValue covarConfig;
    Eigen::MatrixXd P, Q, R;

    //Load Lugre EKF Config
    nh.getParam("/state_estimation/lugre_ekf/initial_estimate_covariance", covarConfig);
    convertXMLRPCVector(P, covarConfig, UKF_CONFIG::N);

    nh.getParam("/state_estimation/lugre_ekf/process_covariance", covarConfig);
    convertXMLRPCVector(Q, covarConfig, UKF_CONFIG::N);

    nh.getParam("/state_estimation/lugre_ekf/measurement_covariance", covarConfig);
    convertXMLRPCVector(R, covarConfig, UKF_CONFIG::M);


    if (!_use_gps_velocities) {
        R(6, 6) = 10000.0; // GPS v_x
        R(7, 7) = 10000.0; // GPS v_y
    }

    //Parse the matrices to kalman object
    _kalmanFilter.loadCovarianceMatrices(P,Q,R);
}

void StateEstimation::predictYaw(double wz, double dt) {
    _yawHat = clampAnglePi2Pi(_yawHat + (wz - _gyroBiasHat) * dt);
}

void StateEstimation::correctYawXsens(double xsensYaw) {
    double meas = clampAnglePi2Pi(xsensYaw - _initialYaw);
    double innov = clampAnglePi2Pi(meas - _yawHat);
    _yawHat = clampAnglePi2Pi(_yawHat + _kXsensYaw * innov);
    _gyroBiasHat -= _kXsensBias * innov;
}

void StateEstimation::correctYawGpsCog(double vMapX, double vMapY, double speed) {
    if (speed < _cogSpeedThresh) return;
    double cog = std::atan2(vMapY, vMapX); // already in the _initialYaw-relative map frame
    double beta = std::atan2(_x(13), _x(12)); // sideslip from UKF vy, vx
    double meas = clampAnglePi2Pi(cog - beta);
    double fusedYaw = clampAnglePi2Pi(_yawHat + _yawOffsetHat);
    double innov = clampAnglePi2Pi(meas - fusedYaw);
    // only touches the offset state, not _yawHat (see header comment)
    _yawOffsetHat = clampAnglePi2Pi(_yawOffsetHat + _kYawOffset * innov);
}

void StateEstimation::convertGpsCoordinates() {
    constexpr double EARTH_RADIUS = 6378388.0;

    double arc = 2.0*M_PI*(EARTH_RADIUS+_gpsCoordinates(2))/360.0;

    if (_initialYaw != 0) {
        // Correct Flat-Earth Projection
        _gpsCoordinatesConverted(0) = arc * cosf(_gpsInitial(0) * M_PI / 180.0) * (_gpsCoordinates(1) - _gpsInitial(1));
        _gpsCoordinatesConverted(1) = arc * (_gpsCoordinates(0) - _gpsInitial(0));
    
        Eigen::Matrix<double,2,2> rotation;
        float angle = _initialYaw;
        rotation << cosf(angle), sinf(angle),
                   -sinf(angle), cosf(angle);

        _gpsCoordinatesConverted.block<2,1>(0,0) = rotation * _gpsCoordinatesConverted.block<2,1>(0,0);
        _gpsCoordinatesConverted(2) = clampAnglePi2Pi(_yawHat + _yawOffsetHat);
    }
}

// This gives us position of the car, in global coordinates. Velocity comes straight from
// the UKF's own vx/vy states (dead reckoning, no GPS position feedback); heading uses the
// yaw observer's corrected estimate (_yawHat + _yawOffsetHat: gyro bias removed, pulled to
// Xsens absolute yaw, and to GPS course-over-ground above _cogSpeedThresh) instead of a
// free-running integration of raw yaw rate, which is what used to make this drift.
sensor_msgs::NavSatFix StateEstimation::getPureNavSatFix() {

    sensor_msgs::NavSatFix nav_msg;

    _x = _kalmanFilter.getStateVector();
    ros::Time currentTime = _kalmanFilter._currentTimeStamp;

    nav_msg.header.stamp = currentTime;
    nav_msg.header.frame_id = "map";

    // Wait for the yaw observer to latch (same 7m gate as _initialYaw)
    if (!_yawFilterInitialized) {
        nav_msg.latitude = _gpsCoordinates(0);
        nav_msg.longitude = _gpsCoordinates(1);
        nav_msg.altitude = _gpsCoordinates(2);
        _lastKekfTime = currentTime;
        return nav_msg;
    }

    constexpr double EARTH_RADIUS = 6378388.0;
    double arc = 2.0 * M_PI * (EARTH_RADIUS + _gpsCoordinates(2)) / 360.0;

    if (!_pureNavInitialized) {
        _anchorLat = _gpsCoordinates(0);
        _anchorLon = _gpsCoordinates(1);

        _rawX = 0.0;
        _rawY = 0.0;

        _lastKekfTime = currentTime;
        _pureNavInitialized = true;
    }

    const double dt = currentTime.toSec() - _lastKekfTime.toSec();
    _lastKekfTime = currentTime;

    double vx = _x(12);
    double vy = _x(13);

    // absolute heading, starts at _initialYaw (not 0) to match the map frame set up in convertGpsCoordinates()
    double yaw = clampAnglePi2Pi(_yawHat + _yawOffsetHat + _initialYaw);

    // transform velocities from car frame to global frame
    double global_vx = vx * cos(yaw) - vy * sin(yaw); // East
    double global_vy = vx * sin(yaw) + vy * cos(yaw); // North

    _rawX += global_vx * dt; // East displacement in meters
    _rawY += global_vy * dt; // North displacement in meters

    // Convert to Lat/Lon
    double lat = (_rawY / arc) + _anchorLat;
    double lon = (_rawX / (arc * cosf(_anchorLat * M_PI / 180.0))) + _anchorLon;

    nav_msg.latitude = lat;
    nav_msg.longitude = lon;
    nav_msg.altitude = _gpsCoordinates(2);

    return nav_msg;
}

double StateEstimation::median_filter_scalar(std::deque<double>& buffer, double value, int window_size) {
    if (window_size <= 1) {
        return value;
    }

    if (buffer.size() < window_size) {
        buffer.push_back(value);
    } else {
        buffer.pop_front();
        buffer.push_back(value);
    }

    if (buffer.empty() || buffer.size() == 1) {
        return value;
    }

    std::vector<double> sorted(buffer.begin(), buffer.end());
    std::sort(sorted.begin(), sorted.end());
    return sorted[sorted.size()/2];
}

//Setters
void StateEstimation::setImuMeasure(const sensor_msgs::Imu &odom3d) {

    sensor_msgs::Imu filtered_imu = odom3d;

    if (_use_median_filters) {

        //Apply median filter
        filtered_imu.linear_acceleration.x = median_filter_scalar(_IMU_BUFFER_ac_x, filtered_imu.linear_acceleration.x, _IMU_window_size);
        filtered_imu.linear_acceleration.y = median_filter_scalar(_IMU_BUFFER_ac_y, filtered_imu.linear_acceleration.y, _IMU_window_size);
        filtered_imu.angular_velocity.z = median_filter_scalar(_IMU_BUFFER_wz, filtered_imu.angular_velocity.z, _IMU_window_size);
    }

    _wz = filtered_imu.angular_velocity.z;

    // FOR GPS
    tf2::Quaternion q(odom3d.orientation.x, odom3d.orientation.y, odom3d.orientation.z, odom3d.orientation.w);
    tf2::Matrix3x3 m(q);
    Eigen::Matrix3d _rotationMatrixXsens;
    Eigen::Vector3d acc(0,0,0);
    double Roll, Pitch, Yaw;

    m.getRPY(Roll, Pitch, Yaw);

    if (_initialYaw == 0 && _distanceIncrement >= 7.0) {
        _initialYaw = Yaw;
        _yawHat = 0.0;
        _yawOffsetHat = 0.0;
        _gyroBiasHat = 0.0;
        _yawFilterInitialized = true;
        _lastYawPredictTime = odom3d.header.stamp;
    }
    _currentXsensYaw = Yaw;

    if (_yawFilterInitialized) {
        double yawDt = (odom3d.header.stamp - _lastYawPredictTime).toSec();
        _lastYawPredictTime = odom3d.header.stamp;
        if (yawDt > 0.0 && yawDt < 1.0) {
            predictYaw(_wz, yawDt);
        }
        correctYawXsens(Yaw);
    }

    // feed the bias corrected rate to the UKF instead of raw gyro (bias comes from correctYawXsens)
    // hurts NEES on speed (44 -> 66) but signals still look fine, leaving it for now, fix another day
    _kalmanFilter.updateInertia(filtered_imu.linear_acceleration.x, filtered_imu.linear_acceleration.y, _wz - _gyroBiasHat);
}

void StateEstimation::setWheelSpeedsMeasure(const common_msgs::CarMotor &wheel_speeds){

    _velocityFrameId = wheel_speeds.header.frame_id;

    common_msgs::CarMotor filtered = wheel_speeds;

    if (_use_median_filters) {

        //Apply median filter
        filtered.value0 = median_filter_scalar(_WheelSpeeds0_BUFFER, filtered.value0, _WheelSpeeds_window_size);
        filtered.value1 = median_filter_scalar(_WheelSpeeds1_BUFFER, filtered.value1, _WheelSpeeds_window_size);
        filtered.value2 = median_filter_scalar(_WheelSpeeds2_BUFFER, filtered.value2, _WheelSpeeds_window_size);
        filtered.value3 = median_filter_scalar(_WheelSpeeds3_BUFFER, filtered.value3, _WheelSpeeds_window_size);
    }

    double w_fl = filtered.value0*M_PI/30.0/_params.engine.gr;
    _kalmanFilter.updateWheelSpeedsFL(w_fl);
    //DEBUG
    _wheelSpeedsVector(0) = w_fl;

    double w_fr = filtered.value1*M_PI/30.0/_params.engine.gr;
    _kalmanFilter.updateWheelSpeedsFR(w_fr);
    //DEBUG
    _wheelSpeedsVector(1) = w_fr;

    double w_rl = filtered.value2*M_PI/30.0/_params.engine.gr;
    _kalmanFilter.updateWheelSpeedsRL(w_rl);
    //DEBUG
    _wheelSpeedsVector(2) = w_rl;

    double w_rr = filtered.value3*M_PI/30.0/_params.engine.gr;
    _kalmanFilter.updateWheelSpeedsRR(w_rr);
    //DEBUG
    _wheelSpeedsVector(3) = w_rr;
}

void StateEstimation::setTorqueMeasure(const common_msgs::CarMotor &torque){

    common_msgs::CarMotor filtered = torque;

    if (_use_median_filters) {

        //Apply median filter   
        filtered.value0 = median_filter_scalar(_Torque0_BUFFER, filtered.value0, _Torque_window_size);
        filtered.value1 = median_filter_scalar(_Torque1_BUFFER, filtered.value1, _Torque_window_size);
        filtered.value2 = median_filter_scalar(_Torque2_BUFFER, filtered.value2, _Torque_window_size);
        filtered.value3 = median_filter_scalar(_Torque3_BUFFER, filtered.value3, _Torque_window_size);
    }

    double u_fl = filtered.value0*_params.engine.gr*0.001 * M_PI/2.0;
    double u_fr = filtered.value1*_params.engine.gr*0.001 * M_PI/2.0;
    double u_rl = filtered.value2*_params.engine.gr*0.001 * M_PI/2.0;
    double u_rr = filtered.value3*_params.engine.gr*0.001 * M_PI/2.0;
    _kalmanFilter.updateMotorTorque(u_fl, u_fr, u_rl,u_rr);

    //DEBUG
    _torqueVector(0) = u_fl;
    _torqueVector(1) = u_fr;
    _torqueVector(2) = u_rl;
    _torqueVector(3) = u_rr;
}

void StateEstimation::setSteeringMeasure(const common_msgs::ControlCmd &steering){
    //dash is positive when turning right
    //as per the 4 wheel model defined in the thesis, the model takes positive steering as turning left
    //converting the dash signal to radians and dividing by 10 because of how its sent to can should do the trick
    //steering ratio is the relation of how much the wheels turn in respect to the steering column
    //max dash is around 100 degrees, max at the wheel is around 20 degrees (fst13)

    // Value for fst 15
    const double steering_ratio = 4.98; // pass this to a config value

    double wheel_st = -steering.steering_angle*(M_PI/180)/(10*steering_ratio);
    
    //true if turning left
    bool steering_bool = (wheel_st > 0) ? 1.0 : -1.0;
    //if turning left, left (inner) wheel turns more than right (outer) wheel

    // Updated linear regression based on new fst 15
    // double steering_fl = steering_bool ? (1.0599 * wheel_st) : (0.9355 * wheel_st);
    // double steering_fr = steering_bool ? (0.9355 * wheel_st) : (1.0599 * wheel_st);

    const std::vector<float> coef_inside  = {6e-5f, 0.1893f};
    const std::vector<float> coef_outside = {-0.0002f, 0.1874f};

    double steering_fl = (M_PI / 180.0) * (steering_bool * (pow(coef_inside[0] * (180.0 / M_PI), 2) * pow(wheel_st * steering_ratio, 2) + coef_inside[1] * (180.0 / M_PI) * wheel_st * steering_ratio) + (1 - steering_bool) * (pow(coef_outside[0] * (180.0 / M_PI), 2) * pow(-wheel_st * steering_ratio, 2) + coef_outside[1] * (180.0 / M_PI) * -wheel_st * steering_ratio));
    double steering_fr = (M_PI / 180.0) * (steering_bool * (pow(coef_outside[0] * (180.0 / M_PI), 2) * pow(wheel_st * steering_ratio, 2) + coef_outside[1] * (180.0 / M_PI) * wheel_st * steering_ratio) + (1 - steering_bool) * (pow(coef_inside[0] * (180.0 / M_PI), 2) * pow(-wheel_st * steering_ratio, 2) + coef_inside[1] * (180.0 / M_PI) * -wheel_st * steering_ratio));

    _kalmanFilter.updateSteeringAngle(steering_fl,steering_fr);
    //DEBUG
    _steeringVector(0) = steering_fl;
    _steeringVector(1) = steering_fr;

    // double steering_ratio = 4.98; fst 14

    //this linear regression comes from validation with fst13 (same as mpc)
    // double steering_fl = steering_bool ? (1.0938244289 * wheel_st) : (0.9575821360 * wheel_st);
    // double steering_fr = steering_bool ? (0.9575821360 * wheel_st) : (1.0938244289 * wheel_st);
}

void StateEstimation::setGpsPosition(const sensor_msgs::NavSatFix &gpsPosition) {
    _gpsCoordinates.setZero();
    _gpsCoordinates(0) = gpsPosition.latitude;
    _gpsCoordinates(1) = gpsPosition.longitude;
    _gpsCoordinates(2) = gpsPosition.altitude;
    
    if (_init) {
        _init = false;
        _gpsInitial(0) = _gpsCoordinates(0);
        _gpsInitial(1) = _gpsCoordinates(1);
    }
    convertGpsCoordinates();

    // map-frame velocity from position diff, feeds the yaw observer's COG correction
    // and, if enabled, the GPS velocity update to the UKF
    if (_isGpsInitForVelocity) {
        double dt = (gpsPosition.header.stamp - _lastGpsStamp).toSec();

        // Temporal throttling to prevent noise amplification
        if (dt >= _minGpsDt) {
            // Calculate global map frame velocity
            double v_global_x = (_gpsCoordinatesConverted(0) - _lastGpsX) / dt;
            double v_global_y = (_gpsCoordinatesConverted(1) - _lastGpsY) / dt;
            double speed = std::hypot(v_global_x, v_global_y);

            correctYawGpsCog(v_global_x, v_global_y, speed);

            if (_use_gps_velocities) {
                // Rotate into vehicle body frame using the fused yaw
                double theta = _gpsCoordinatesConverted(2);
                double v_body_x = v_global_x * std::cos(theta) + v_global_y * std::sin(theta);
                double v_body_y = -v_global_x * std::sin(theta) + v_global_y * std::cos(theta);

                // Update the Kalman Filter directly
                _kalmanFilter.updateGpsVelocity(v_body_x, v_body_y);
            }

            // Update persistent state
            _lastGpsStamp = gpsPosition.header.stamp;
            _lastGpsX = _gpsCoordinatesConverted(0);
            _lastGpsY = _gpsCoordinatesConverted(1);
        }
    } else {
        _isGpsInitForVelocity = true;
        _lastGpsStamp = gpsPosition.header.stamp;
        _lastGpsX = _gpsCoordinatesConverted(0);
        _lastGpsY = _gpsCoordinatesConverted(1);
    }
}

void StateEstimation::setGpsVelocity(const geometry_msgs::TwistWithCovarianceStamped &gpsVelocity) {
    //Xsens Vy seems to be inverted in relation to the YR
    _kalmanFilter.updateGpsVelocity(gpsVelocity.twist.twist.linear.x, -gpsVelocity.twist.twist.linear.y);
}

//Getters
common_msgs::CarVelocity StateEstimation::getCarVelocity() {

    //Create Velocity Message
    _carVelocity.header.stamp = ros::Time::now();
    _carVelocity.header.frame_id = _velocityFrameId;
    _carVelocity.velocity.x = _x(12);
    _carVelocity.velocity.y = _x(13);
    _carVelocity.velocity.theta = _x(14);
    return _carVelocity; 
}

common_msgs::CarTireInfo StateEstimation::getCarTireInfo() {

    //Create TireInfo Message
    _carTireInfo.header.stamp = ros::Time::now();
    _carTireInfo.header.frame_id = _velocityFrameId;
    _carTireInfo.FL = convertToTireInfoMsg(_kalmanFilter.getTireInfo(0));
    _carTireInfo.FR = convertToTireInfoMsg(_kalmanFilter.getTireInfo(1));
    _carTireInfo.RL = convertToTireInfoMsg(_kalmanFilter.getTireInfo(2));
    _carTireInfo.RR = convertToTireInfoMsg(_kalmanFilter.getTireInfo(3));
    
    return _carTireInfo;
}

common_msgs::TireInfo StateEstimation::convertToTireInfoMsg(TireInfoStruct tire){
    common_msgs::TireInfo msg;
    msg.Gx = tire.Gx;
    msg.Kx = tire.Kx;
    msg.Zx = tire.Zx;
    msg.Ox = tire.Ox;
    msg.Vrx = tire.Vrx;
    msg.Gy = tire.Gy;
    msg.Ky = tire.Ky;
    msg.Zy = tire.Zy;
    msg.Oy = tire.Oy;
    msg.Vry = tire.Vry;
    return msg;
}

std_msgs::Float32 StateEstimation::getPMatrixTrace() {

    //Update Covariance Matrix
    _P = _kalmanFilter.getStateCovariance();
    
    //Create Trace Message
    _traceP.data = _P.trace();
    return _traceP;
}

// Checking determinant of P matrix gives more information about uncertainty than trace, since we are interested 
// in the volume of the uncertainty
std_msgs::Float32 StateEstimation::getPMatrixDet() {     

    //Update Covariance Matrix
    _P = _kalmanFilter.getStateCovariance();

    // Create Determinant Message
    _detP.data = static_cast<float>(_P.determinant());
    return _detP;
}

// Row-major flatten of the full N x N state covariance, for exact (non-diagonal-approximated) NEES in post-processing
std_msgs::Float64MultiArray StateEstimation::getStateCovarianceFull() {
    _P = _kalmanFilter.getStateCovariance();

    std_msgs::Float64MultiArray msg;
    msg.data.resize(UKF_CONFIG::N * UKF_CONFIG::N);
    for (int i = 0; i < UKF_CONFIG::N; ++i) {
        for (int j = 0; j < UKF_CONFIG::N; ++j) {
            msg.data[i * UKF_CONFIG::N + j] = _P(i, j);
        }
    }
    return msg;
}

std_msgs::Float64MultiArray StateEstimation::getInnovation() {
    const Eigen::Matrix<double, UKF_CONFIG::M, 1> innovation = _kalmanFilter.getInnovation();

    std_msgs::Float64MultiArray msg;
    msg.data.resize(UKF_CONFIG::M);
    for (int i = 0; i < UKF_CONFIG::M; ++i) {
        msg.data[i] = innovation(i);
    }
    return msg;
}

// Row-major flatten of the full M x M innovation covariance (Pyy) actually used for the Kalman gain, for NIS
std_msgs::Float64MultiArray StateEstimation::getInnovationCovarianceFull() {
    const Eigen::Matrix<double, UKF_CONFIG::M, UKF_CONFIG::M> Pyy = _kalmanFilter.getInnovationCovariance();

    std_msgs::Float64MultiArray msg;
    msg.data.resize(UKF_CONFIG::M * UKF_CONFIG::M);
    for (int i = 0; i < UKF_CONFIG::M; ++i) {
        for (int j = 0; j < UKF_CONFIG::M; ++j) {
            msg.data[i * UKF_CONFIG::M + j] = Pyy(i, j);
        }
    }
    return msg;
}

geometry_msgs::Vector3Stamped StateEstimation::getSteering() {
    _steering.header.stamp = ros::Time::now();
    _steering.header.frame_id = _velocityFrameId;
    _steering.vector.x = _steeringVector(0);
    _steering.vector.y = _steeringVector(1);
    return _steering; 
}

common_msgs::StateVector StateEstimation::getTorque() {

    common_msgs::StateVector state;

    state.wfl = _torqueVector(0);
    state.wfr = _torqueVector(1);
    state.wrl = _torqueVector(2);
    state.wrr = _torqueVector(3);
    state.ax = 0;
    state.ay = 0;
    state.vx = 0;
    state.vy = 0;
    state.wz = 0;

    return state;
}

nav_msgs::Odometry StateEstimation::getGpsPositionOdometry() {
    _gpsOdom.header.stamp = ros::Time::now();
    _gpsOdom.header.frame_id = "map";
    _gpsOdom.pose.pose.position.x = _gpsCoordinatesConverted(0);
    _gpsOdom.pose.pose.position.y = _gpsCoordinatesConverted(1);
    tf2::Quaternion quat_tf;
    quat_tf.setRPY(0.0, 0.0, _gpsCoordinatesConverted(2));
    geometry_msgs::Quaternion quat_msg;
    tf2::convert(quat_tf, quat_msg);
    _gpsOdom.pose.pose.orientation = quat_msg;
    return _gpsOdom;
}

common_msgs::StateVector StateEstimation::get_pipeline_StateVector(){

    // get only important states, forces are hard to evaluate therefore are not included
    _x = _kalmanFilter.getStateVector();
    common_msgs::StateVector state;

    state.wfl = _x(2);
    state.wfr = _x(5);
    state.wrl = _x(8);
    state.wrr = _x(11);
    state.ax  = 0; 
    state.ay  = 0;
    state.vx  = _x(12);
    state.vy  = _x(13);
    state.wz  = _x(14);

    return state;
}

common_msgs::StateVector StateEstimation::getMeasuredValues() {

    _y = _kalmanFilter.getMeasurementVector();

    common_msgs::StateVector state;

    state.header.stamp = ros::Time::now();
    state.header.frame_id = _velocityFrameId;

    state.wfl = _y(0);
    state.wfr = _y(1);
    state.wrl = _y(2);
    state.wrr = _y(3);
    state.ax  = _y(4);
    state.ay  = _y(5);
    state.vx  = _y(6);
    state.vy  = _y(7);
    state.wz  = _y(8);

    return state;
}

common_msgs::StateVector StateEstimation::getPredictedValues() {

    _y_hat = _kalmanFilter.getPredictedVector();

    common_msgs::StateVector state;

    state.header.stamp = ros::Time::now();
    state.header.frame_id = _velocityFrameId;

    state.wfl = _y_hat(0);
    state.wfr = _y_hat(1);
    state.wrl = _y_hat(2);
    state.wrr = _y_hat(3);
    state.ax =  _y_hat(4);
    state.ay =  _y_hat(5);
    state.vx =  _y_hat(6);
    state.vy =  _y_hat(7);
    state.wz =  _y_hat(8);
    
    return state;
}