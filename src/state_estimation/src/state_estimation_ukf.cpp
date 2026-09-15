#include "state_estimation/state_estimation_ukf.hpp"

KalmanFilter::KalmanFilter() {
    computeInitialGuess();
    calculate_weights();
}

void KalmanFilter::loadParameters(Params params) {
    _params = params;
    _Fn = _params.Fn;
}

void KalmanFilter::loadCovarianceMatrices(Eigen::MatrixXd P, Eigen::MatrixXd Q, Eigen::MatrixXd R) {
    _P = P;
    _Q = Q;
    _R = R;
}

void KalmanFilter::computeInitialGuess() {
    _x.setConstant(_eps);
    _Q.setZero();
    _R.setZero();
    _u.setZero();
    _y.setConstant(_eps);
    _y_hat.setConstant(_eps);
    _st.setZero();
}

void KalmanFilter::calculate_weights(){

    // Calculate weights for the Unscented Kalman Filter on node initialization 
    _lambda = _alpha * _alpha * (UKF_CONFIG::N + _kappa) - UKF_CONFIG::N; 

    _Wm[0] = _lambda / (UKF_CONFIG::N + _lambda);
    _Wc[0] = _lambda / (UKF_CONFIG::N + _lambda) + (1.0 - _alpha*_alpha + _beta);

    for (int i = 1; i < UKF_CONFIG::L; ++i){
        _Wm[i] = _Wc[i] = 1.0 / (2.0 * (UKF_CONFIG::N + _lambda));
    }
}

//Setters
void KalmanFilter::updateWheelSpeedsFL(const double w_fl) {
    _y(0) = w_fl;
}

void KalmanFilter::updateWheelSpeedsFR(const double w_fr) {
    _y(1) = w_fr;
}

void KalmanFilter::updateWheelSpeedsRL(const double w_rl) {
    _y(2) = w_rl;
}

void KalmanFilter::updateWheelSpeedsRR(const double w_rr) {
    _y(3) = w_rr;
}

void KalmanFilter::updateInertia(const double accx, const double accy, const double wz) {
    _y(4) = accx;
    _y(5) = accy;
    _y(8) = wz;
}

void KalmanFilter::updateSteeringAngle(const double steering_fl, const double steering_fr){
    _st(0) = steering_fl;
    _st(1) = steering_fr;
    _st(2) = 0;
    _st(3) = 0;
}

void KalmanFilter::updateMotorTorque(const double u_fl, const double u_fr, const double u_rl, const double u_rr){
    _u(0) = u_fl;
    _u(1) = u_fr;
    _u(2) = u_rl;
    _u(3) = u_rr;
}

void KalmanFilter::updateGpsVelocity(const double vx, const double vy) {
    _y(6) = vx;
    _y(7) = vy;
}

//Getters
Eigen::Matrix<double,UKF_CONFIG::N,1> const & KalmanFilter::getStateVector() const { return _x; }
Eigen::Matrix<double,UKF_CONFIG::N,UKF_CONFIG::N> const & KalmanFilter::getStateCovariance() const { return _P; }
Eigen::Matrix<double,UKF_CONFIG::M,1> KalmanFilter::getMeasurementVector() const {return _y;}

Eigen::Matrix<double,UKF_CONFIG::M,1> KalmanFilter::getPredictedVector() const {return _y_hat;}
Eigen::Matrix<double,UKF_CONFIG::M,1> KalmanFilter::getPredictedVectorPost() const {return _y_hat_post;}
Eigen::Matrix<double,UKF_CONFIG::M,1> const & KalmanFilter::getInnovation() const { return _innovation; }
Eigen::Matrix<double,UKF_CONFIG::M,UKF_CONFIG::M> const & KalmanFilter::getInnovationCovariance() const { return _Pyy; }

TireInfoStruct const KalmanFilter::getTireInfo(uint8_t id) const {
    TireInfoStruct tire = _tireInfoArray[id];
    tire.Zx = _x(id*3 + 0);
    tire.Zy = _x(id*3 + 1);
    return tire;
}

void KalmanFilter::generateSigmaPoints(const Eigen::Matrix<double,UKF_CONFIG::N,1>& x, Eigen::Matrix<double,UKF_CONFIG::N,UKF_CONFIG::N>& P, Eigen::Matrix<double,UKF_CONFIG::N,UKF_CONFIG::L>& Xi) {

    Eigen::Matrix<double, UKF_CONFIG::N, UKF_CONFIG::N> P_inside;
    P_inside = (UKF_CONFIG::N + _lambda) * P;

    // Cholesky Decomposition to calculate the square root term
    Eigen::LLT<Eigen::Matrix<double, UKF_CONFIG::N, UKF_CONFIG::N>> lltOfP(P_inside);

    // Retrieve lower triangular matrix L
    Eigen::Matrix<double, UKF_CONFIG::N, UKF_CONFIG::N> sqrtP = lltOfP.matrixL();

    // Generate Sigma Points
    Xi.col(0) = x;
    
    for (int i = 0; i < UKF_CONFIG::N; ++i) {
        Xi.col(i + 1)      = x + sqrtP.col(i);
        Xi.col(i + 1 + UKF_CONFIG::N) = x - sqrtP.col(i);
    }
}

Eigen::Matrix<double, UKF_CONFIG::N, 1> KalmanFilter::propagateSigmaPoint(const Eigen::Matrix<double, UKF_CONFIG::N, 1>& xi, const Eigen::Vector4d& Fn)
{
    // Predict the next state using the nonlinear dynamics model
    Eigen::Matrix<double, UKF_CONFIG::N, 1> xdot = _lugre.nonlinearDynamics(_params.tireArray, _st, xi, _u, Fn, _params.inercia.Iz, _params.inercia.m, _ts);
    
    Eigen::Matrix<double, UKF_CONFIG::N, 1> next_state = xi + _ts * xdot;

    // If any of them are non-finite, reject the step and return the original sigma point
    if (!next_state.allFinite()) {
        ROS_WARN_STREAM("Integration hit non-finite boundary. Rejecting step.");
        return xi; 
    }

    return next_state;
}

Eigen::Matrix<double, UKF_CONFIG::M, 1> KalmanFilter::measurementModel(const Eigen::Matrix<double, UKF_CONFIG::N, 1>& state, const Eigen::Vector4d& Fn) 
{
    Eigen::Matrix<double, UKF_CONFIG::M, 1> y_pred;

    y_pred(0) = state(2);   // w_fl
    y_pred(1) = state(5);   // w_fr
    y_pred(2) = state(8);   // w_rl
    y_pred(3) = state(11);  // w_rr
    
    y_pred(6) = state(12);  // vx
    y_pred(7) = state(13);  // vy
    y_pred(8) = state(14);  // wz

    Eigen::Matrix<double, UKF_CONFIG::N, 1> state_dot = _lugre.nonlinearDynamics( _params.tireArray, _st, state, _u, Fn, _params.inercia.Iz, _params.inercia.m, _ts);

    double vx     = state(12);
    double vy     = state(13);
    double wz     = state(14);
    double vx_dot = state_dot(12);
    double vy_dot = state_dot(13);

    // Acceleration including centripetal terms
    double ax_imu = vx_dot - (vy * wz);
    double ay_imu = vy_dot + (vx * wz);

    y_pred(4) = ax_imu; // ax
    y_pred(5) = ay_imu; // ay

    return y_pred;
}

void KalmanFilter::updateDynamicLoads() {

    const double vx = _x(12);
    const double vy = _x(13);
    const double wz = _x(14);

    // Use the predicted acceleration for the next iteration so the calculated forces match the correct time stamp
    const double ax = _y_hat_post(4);
    const double ay = _y_hat_post(5);

    // Calculate weight transfers
    const double lon_wt = _params.inercia.m * _params.kinematics.h * (ax/ (_params.kinematics.a + _params.kinematics.b));
    const double lat_wt = _params.inercia.m * _params.kinematics.h * (ay/ _params.kinematics.track);
    const double downforce = 0.5 * _params.aero.rho * _params.aero.cl * std::pow(vx, 2);

    // Calculate normal forces off each tire
    _Fn(0) = _params.Fn(0) - lon_wt / 2.0 - lat_wt / 2.0 + downforce * _params.aero.dist / 2.0;         // FL
    _Fn(1) = _params.Fn(1) - lon_wt / 2.0 + lat_wt / 2.0 + downforce * _params.aero.dist / 2.0;         // FR
    _Fn(2) = _params.Fn(2) + lon_wt / 2.0 - lat_wt / 2.0 + downforce * (1.0 - _params.aero.dist) / 2.0; // RL
    _Fn(3) = _params.Fn(3) + lon_wt / 2.0 + lat_wt / 2.0 + downforce * (1.0 - _params.aero.dist) / 2.0; // RR

    // Restrict negative forces
    _Fn = _Fn.cwiseMax(0.0);
}

void KalmanFilter::update() {

    // Generate Sigma Points
    Eigen::Matrix<double, UKF_CONFIG::N, UKF_CONFIG::L> Xi;
    generateSigmaPoints(_x, _P, Xi); 

    Eigen::Matrix<double, UKF_CONFIG::N, UKF_CONFIG::L> Xi_pred;

    // Sigma Points Propagation
    for (int i = 0; i < UKF_CONFIG::L; i++) {
        Xi_pred.col(i) = propagateSigmaPoint(Xi.col(i),_Fn);
    }

    // Calculate Predicted Mean
    Eigen::Matrix<double, UKF_CONFIG::N, 1> x_pred = Eigen::Matrix<double, UKF_CONFIG::N, 1>::Zero();
    for(int i = 0; i < UKF_CONFIG::L; i++) {
        x_pred += _Wm[i] * Xi_pred.col(i);
    }

    // === Predicted state covariance ===
    // P_pred = ∑ Wc_i * (Xi_pred_i - x_pred)(Xi_pred_i - x_pred)^T  +  Q
    Eigen::Matrix<double, UKF_CONFIG::N, UKF_CONFIG::N> Pxx = Eigen::Matrix<double, UKF_CONFIG::N, UKF_CONFIG::N>::Zero();
    for (int i = 0; i < UKF_CONFIG::L; i++) {
        Eigen::Matrix<double, UKF_CONFIG::N, 1> x_diff = Xi_pred.col(i) - x_pred;
        Pxx += _Wc[i] * (x_diff * x_diff.transpose());
    }

    Eigen::Matrix<double, UKF_CONFIG::N, UKF_CONFIG::N> P_pred = Pxx + _Q;

    // Generate new sigma points based on the predicted state and covariance for the measurement update
    Eigen::Matrix<double, UKF_CONFIG::N, UKF_CONFIG::L> Xi_new;
    generateSigmaPoints(x_pred, P_pred, Xi_new);

    // Measurement Update
    Eigen::Matrix<double, UKF_CONFIG::M, UKF_CONFIG::L> Yi;
    for (int i = 0; i < UKF_CONFIG::L; i++) {
        Yi.col(i) = measurementModel(Xi_new.col(i), _Fn);
    }

    // Calculate Predicted Measurement Mean
    _y_hat.setZero();
    for (int i = 0; i < UKF_CONFIG::L; i++) {
        _y_hat += _Wm[i] * Yi.col(i);
    }

    // === Innovation covariance and state/measurement cross-covariance ===
    //   Pyy = ∑ Wc_i * (Y_i - y_hat)(Y_i - y_hat)^T  +  R
    //   Pxy = ∑ Wc_i * (Xi_new_i - x_pred)(Y_i - y_hat)^T
    Eigen::Matrix<double, UKF_CONFIG::M, UKF_CONFIG::M>& Pyy = _Pyy;
    Pyy.setZero();
    Eigen::Matrix<double, UKF_CONFIG::N, UKF_CONFIG::M> Pxy = Eigen::Matrix<double, UKF_CONFIG::N, UKF_CONFIG::M>::Zero();

    for (int i = 0; i < UKF_CONFIG::L; i++) {
        Eigen::Matrix<double, UKF_CONFIG::N, 1> x_diff = Xi_new.col(i) - x_pred;
        Eigen::Matrix<double, UKF_CONFIG::M, 1> y_diff = Yi.col(i) - _y_hat;

        Pyy += _Wc[i] * (y_diff * y_diff.transpose());
        Pxy += _Wc[i] * (x_diff * y_diff.transpose());
    }
    Pyy += _R;

    const Eigen::Matrix<double, UKF_CONFIG::M, 1>& innovation = _y - _y_hat;

    _innovation = innovation;

    // Normalized innovation
    Eigen::Array<double, UKF_CONFIG::M, 1> norm_innov_abs = (innovation.array() / Pyy.diagonal().array().sqrt()).abs();

    const double outlier_threshold    = 3.0;    // standard deviations
    const double penalty_scale_factor = 1000.0; // how much to inflate R

    // Outlier rejection: any measurement channel whose innovation sits more than
    // 3 standard deviations away is treated as an outlier. 
    Eigen::Matrix<double, UKF_CONFIG::M, UKF_CONFIG::M> Lk = Eigen::Matrix<double, UKF_CONFIG::M, UKF_CONFIG::M>::Zero();
    for (int i = 0; i < UKF_CONFIG::M; i++) {
         // Apply the penalty to the innovation covariance matrix for detected outliers
        if (norm_innov_abs(i) > outlier_threshold) {
            Lk(i, i) = _R(i, i) * penalty_scale_factor;
        }
    }

    // Pyy = Pyy + Lk  (inflate the covariance of outlier channels only)
    Pyy += Lk;

    // === Kalman gain ===
    // K = Pxy * Pyy^-1
    Eigen::Matrix<double, UKF_CONFIG::M, UKF_CONFIG::M> Pyy_inv = Pyy.ldlt().solve(Eigen::Matrix<double, UKF_CONFIG::M, UKF_CONFIG::M>::Identity());

    Eigen::Matrix<double, UKF_CONFIG::N, UKF_CONFIG::M> K;
    K = Pxy * Pyy_inv;

    // === State and covariance update ===
    //   x = x_pred + K * (y - y_hat)
    //   P = P_pred - K * Pyy * K^T
    _x = x_pred + K * innovation;
    _P = P_pred - K * Pyy * K.transpose();

    // extract the predicted measurement vector after the state update for use in the next iteration
    _y_hat_post = measurementModel(_x, _Fn);

    // Ensure positive semi-definite 
    _P = 0.5 * (_P + _P.transpose());

    // calculate the average wheel speed to determine if the vehicle is practically stopped
    const double avg_wheel_speed = (_y(0) + _y(1) + _y(2) + _y(3)) / 4.0;

    // If wheel speeds indicate the car is practically stopped force the kinematic velocities to perfectly zero to protect from drift 
    if (std::abs(avg_wheel_speed) < 0.05) {

        _x(12) = 0.00001; // vx
        _x(13) = 0.00001; // vy
        _x(14) = 0.00001; // wz

        _y_hat(0) = 0.000001; //wfl
        _y_hat(1) = 0.000001; //wfr
        _y_hat(2) = 0.000001; //wrl
        _y_hat(3) = 0.000001; //wrr
        _y_hat(4) = 0.000001; //ax
        _y_hat(5) = 0.000001; //ay
        _y_hat(6) = 0.000001; //vx
        _y_hat(7) = 0.000001; //vy
        _y_hat(8) = 0.000001; //wz

        // Crush the covariance for these states so the filter is absolutely certain it's stopped
        _P(12, 12) = 1e-3;
        _P(13, 13) = 1e-3;
        _P(14, 14) = 1e-3;
    }

    // Update the dynamic loads 
    updateDynamicLoads();

    // Extract tire information for the current state for debug
    _lugre.extractTireInfo(_params.tireArray, _st, _x, _Fn, _tireInfoArray);

    _currentTimeStamp = ros::Time::now();
}