#include "state_estimation/state_estimation_ekf.hpp"

KalmanFilter::KalmanFilter() {
    computeInitialGuess();
    initializeContStateMatrices();
}

void KalmanFilter::loadParameters(Params params) {
    _params = params;
}

void KalmanFilter::loadCovarianceMatrices(Eigen::MatrixXd P, Eigen::MatrixXd Q, Eigen::MatrixXd R) {
    _P = P;
    _Q = Q;
    _R = R;
}

void KalmanFilter::computeInitialGuess() {
    _x.setZero();
    _Q.setZero();
    _R.setZero();
    _u.setZero();
    _y.setZero();
    _y_hat.setZero();
    _st.setZero();
    _innovation.setZero();
}

void KalmanFilter::initializeContStateMatrices() {
    _continousStateMatrices.A.setZero();
    _continousStateMatrices.B.setZero();
    _continousStateMatrices.C.setZero();
    _C.setZero();
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
Eigen::Matrix<double,15,1> const & KalmanFilter::getStateVector() const { return _x; }
Eigen::Matrix<double,15,15> const & KalmanFilter::getStateCovariance() const { return _P; }
Eigen::Matrix<double,9,15> const & KalmanFilter::getCMatrix() const { return _C; }

Eigen::Matrix<double,9,1> KalmanFilter::getMeasurementVector() const{return _y;}

Eigen::Matrix<double,9,1> KalmanFilter::getPredictedVector() const{return _y_hat;}

Eigen::Matrix<double,9,1> const & KalmanFilter::getInnovation() const { return _innovation; }
Eigen::Matrix<double,9,9> const & KalmanFilter::getInnovationCovariance() const { return _S; }

TireInfoStruct const KalmanFilter::getTireInfo(uint8_t id)const{
    TireInfoStruct tire = _lugre.getTireInfo(id);
    tire.Zx = _x(id*3);
    tire.Zy = _x(id*3+1);
    return tire;
}


void KalmanFilter::update() {

    Eigen::Matrix<double,15,15> A, A_d;
    Eigen::Matrix<double,15,4> B, B_d;
    Eigen::Matrix<double,9,1> y_hat;
    Eigen::Matrix<double,9,1> normalized_innovation = Eigen::Matrix<double,9,1>::Zero();
    Eigen::Matrix<double,9,1> ik = Eigen::Matrix<double,9,1>::Zero();

    // initiliaze C matrix on first step
    if (_C.isZero()) {
        //for this first step Fn is sent as static loads
        _continousStateMatrices = _lugre.fullCarDynamics(_params.tireArray, _st, Eigen::Matrix<double,4,1>(_x(2),_x(5),_x(8),_x(11)), _x(12), _x(13), _x(14), _params.Fn, _params.inercia.Iz, _params.inercia.m);
        _C = _continousStateMatrices.C; 
    }

    //calculate S with C and P from last step
    Eigen::MatrixXd S = _C*_P*_C.transpose() + _R; // S = C P K|K-1 C^T + R

    //current measured outputs and predicted state
    y_hat = _C*_x;

    _y_hat = y_hat;
    _innovation = _y - y_hat;

    // Normalize innovation
    normalized_innovation = (_y - y_hat).array() / sqrt(S.diagonal().array());
    
    // Define Threshold (3-sigma)
    Eigen::Matrix<double,9,1> thr = Eigen::Matrix<double,9,1>::Constant(3.0); 

    double penalty_scale_factor = 1000.0; 

    // Detect outliers
    for (int i = 0; i < normalized_innovation.size(); ++i) {

        ik[i] = (std::abs(normalized_innovation(i)) > thr(i)) ? 1.0 : 0.0;
    }
    
    // Scale R matrix based on outliers
    Eigen::Matrix<double,9,9> D = ik.asDiagonal();
    Eigen::Matrix<double,9,9> R_diag = _R.diagonal().asDiagonal(); 
    
    // This adds a penalty proportional to the sensor's noise
    Eigen::Matrix<double,9,9> Lk = D * (penalty_scale_factor * R_diag) * D;
    
    // Update Innovation Covariance
    S += Lk;
    _S = S;

    Eigen::MatrixXd K = _P*_C.transpose()*S.inverse(); // K = P K|K-1 C^T S^-1

    Eigen::Matrix<double,15,1> xk = _x + K*(_y - (y_hat));

    // Joseph form
    Eigen::Matrix<double,15,15> I_KC = Eigen::Matrix<double,15,15>::Identity() - K*_C;
    _P = I_KC * _P * I_KC.transpose() + K * _R * K.transpose();

    // This is the predicted state vector after the update step, which will be used to compute weight transfers and downforce for the next prediction step
    Eigen::Matrix<double,9,1> y_hat_post = _C * xk;

    //for all steps but the first we send Fn with more info
    //Fn = sprung_mass + unsprung_mass + lateral_weight_transfer + longitudinal_weight_transfer + aero
    //given that lugre is heavily based on normal loads all these terms were modelled 
    //weight transfers will take into consideration coriolis in the acceleration term
    //ax = y_hat(4), ay = y_hat(5), vx = xk(12), vy = xk(13), yawrate = xk(14), currently lat_wt doesnt include the centripetal term
    //since it is already included in the C matrix

    lon_wt = _params.inercia.m * _params.kinematics.h * y_hat_post(4) / (_params.kinematics.a + _params.kinematics.b);
    lat_wt = _params.inercia.m * _params.kinematics.h * y_hat_post(5) / _params.kinematics.track;
    downforce = 0.5 * _params.aero.rho * _params.aero.cl * pow(xk(12), 2); // 1/2 * cl * rho * v^2, W H = area = 1 m²

    Eigen::Vector4d Fn;

    //_params.Fn already has sprung and unsprung static
    Fn(0) = _params.Fn(0) - lon_wt/2 - lat_wt/2 + downforce*_params.aero.dist/2; // fl
    Fn(1) = _params.Fn(1) - lon_wt/2 + lat_wt/2 + downforce*_params.aero.dist/2; // fr
    Fn(2) = _params.Fn(2) + lon_wt/2 - lat_wt/2 + downforce*(1-_params.aero.dist)/2; // rl
    Fn(3) = _params.Fn(3) + lon_wt/2 + lat_wt/2 + downforce*(1-_params.aero.dist)/2; //rr

    //no negative forces though our car is so poorly designed that it does lift wheels irl
    Fn = Fn.cwiseMax(0.0);

    //calculate new linearizations for state matrices
    _continousStateMatrices = _lugre.fullCarDynamics(_params.tireArray, _st, Eigen::Matrix<double,4,1>(xk(2),xk(5),xk(8),xk(11)), xk(12), xk(13), xk(14), Fn, _params.inercia.Iz, _params.inercia.m);
    A = _continousStateMatrices.A;
    B = _continousStateMatrices.B;
    _C = _continousStateMatrices.C;

    StateMatrices discreteStateMatrices = discretizeModel(A, B);
    A_d = discreteStateMatrices.A;
    B_d = discreteStateMatrices.B;

    //new state vector
    _x = A_d*xk + B_d*_u; // x k+1 = Ak * xk + Bk + U
    _currentTimeStamp = ros::Time::now();
    _P = A_d*_P*A_d.transpose() + _Q; // P K|K-1 = A K|K-1 * P K-1|K-1 * A K|K-1 ^T + Q
}

StateMatrices KalmanFilter::discretizeModel(const Eigen::Matrix<double,15,15> &A_c, const Eigen::Matrix<double,15,4> &B_c) {
    
    // Exact ZOH discretization via augmented matrix exponential.
    constexpr int NA = 15;  //contStateMatrices.A.cols();
    constexpr int NB = 4;   //contStateMatrices.B.cols();
    constexpr int NC = 9;   //contStateMatrices.C.cols();
    
    Eigen::Matrix<double,NA,NA>  A_d;
    Eigen::Matrix<double,NA,NB>  B_d;
    Eigen::Matrix<double,NC,NA>  C_d = Eigen::Matrix<double,NC,NA>::Zero();

    Eigen::Matrix<double,NA+NB,NA+NB> temp = Eigen::Matrix<double,NA+NB,NA+NB>::Zero();

    // temp = [ A_c  B_c ] * Ts  ->  exp(temp) = [ A_d  B_d ]
    //        [  0    0  ]                       [  0    I  ]
    temp.block<NA,NA>(0,0) = A_c;
    temp.block<NA,NB>(0,NA) = B_c;
    temp = temp*_ts;

    const Eigen::Matrix<double,NA+NB,NA+NB> temp_res = temp.exp();

    A_d = temp_res.block<NA,NA>(0,0);
    B_d = temp_res.block<NA,NB>(0,NA);
    
    return {A_d, B_d, C_d};
}