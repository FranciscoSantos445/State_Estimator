#include "state_estimation/state_estimation_lugre.hpp"

using namespace Eigen;

Eigen::Matrix<double,15,1> Lugre::nonlinearDynamics(TireArray& tireArray, const Eigen::Matrix<double,4,1>& steerAngle, const Eigen::Matrix<double,15,1>& x, const Eigen::Matrix<double,4,1>& u, const Eigen::Vector4d& FnArray, double Iz, double m, const double dt)
{
    Matrix<double,15,1> xdot;
    xdot.setZero();

    const double vx = x(12), vy = x(13), wz = x(14);
    double sum_Fx_car = 0.0, sum_Fy_car = 0.0, sum_Mz = 0.0;

    for (int j = 0; j < 4; ++j) {
        Tire& tire = tireArray[j];
        const double delta = steerAngle(j);
        double zx = x(j*3 + 0);
        double zy = x(j*3 + 1);
        const double w = std::max(0.0, x(j*3 + 2)); // ensure wheelspeeds are never negative

        //Damping Constants
        const double sig0x = tire.sigma(0,0), sig1x = tire.sigma(1,0), sig2x = tire.sigma(2,0);
        const double sig0y = tire.sigma(0,1), sig1y = tire.sigma(1,1), sig2y = tire.sigma(2,1);

        const double rx = tire.p(0);
        const double ry = tire.p(1);
        const double J  = tire.J;

        const double Fn = fmax(FnArray(j), 1.0);
        const double rl = tire.r - Fn / tire.kz; //loaded radius metres
        const double L  = 2.0 * sqrtf(fmax(pow(tire.r,2) - pow(rl,2), _eps)); //contact patch length

        //Calculate ground velocity under tyre
        Matrix<double,2,1> v_tire = velocityTransform(delta, rx, ry, vx, vy, wz);

        //Relative Velocities Calculation
        const double vr_x = w * rl - v_tire(0);
        const double vr_y = -v_tire(1);
        Matrix<double,2,1> vr(vr_x, vr_y);

        //Rate of bristle restitution
        double Ox = computeOx(tire, rl, w, vr, L, j);
        double Oy = computeOy(tire, rl, w, vr, L, j);

        // Derivatives of the bristle states
        double dzx_dt = vr_x - Ox * zx;
        double dzy_dt = vr_y - Oy * zy;

        // Calculate the tire forces
        double Fx_tire = Fn * (sig0x * zx + sig1x * dzx_dt + sig2x * vr_x);
        double Fy_tire = Fn * (sig0y * zy + sig1y * dzy_dt + sig2y * vr_y);

        // derivative of wheel speed
        double dw_dt = (u(j) - rl * Fx_tire) / J;
        if (!std::isfinite(dw_dt)) dw_dt = 0.0;

        // Backward Implicit Euler
        xdot(j*3 + 0) = dzx_dt / (1.0 + dt * Ox);
        xdot(j*3 + 1) = dzy_dt / (1.0 + dt * Oy);
        xdot(j*3 + 2) = dw_dt;

        // Calculate the tire forces in the vehicle frame
        const double c = cos(delta), s = sin(delta);
        const double Fx_car = Fx_tire * c - Fy_tire * s;
        const double Fy_car = Fx_tire * s + Fy_tire * c;

        // Accumulate forces and moments for the vehicle dynamics
        sum_Fx_car += Fx_car;
        sum_Fy_car += Fy_car;
        sum_Mz += rx * Fy_car - ry * Fx_car;

    }

    // Vehicle dynamics equations, sum of all tire forces and moments
    xdot(12) = (sum_Fx_car / m); 
    xdot(13) = (sum_Fy_car / m);
    xdot(14) = sum_Mz / Iz;

    // Guarantee that no matter what happens, nonlinearDynamics returns finite numbers to the UKF integrator
    for (int i = 0; i < 15; ++i) {
        if (!std::isfinite(xdot(i))) {
            xdot(i) = 0.0;
        }
    }

    return xdot;
}

Matrix<double,2,1> Lugre::velocityTransform(const double delta, const double rx, const double ry, const double vx_car, const double vy_car, const double wz) {
    Matrix<double,2,1> v_car(vx_car, vy_car);
    Matrix<double,2,2> rot;
    Matrix<double,2,1> yaw_component(-ry*wz, rx*wz);

    rot << cosf(delta), -sinf(delta),
           sinf(delta), cosf(delta);

    return rot.transpose()*(v_car+yaw_component);
}

double Lugre::computeOx(const Tire &tire, const double re, const double w, const Matrix<double,2,1> &vr, const double L, int tireId ) {

    const double sig0_x = tire.sigma(0,0);

    //Calculate sliding friction
    const double g = computeG(tire,vr);

    //Steady state constant calculation
    const double kx = computeKi(w,vr,L,re,g,sig0_x);

    double Ox = vr.norm()*sig0_x/g + (kx/L)*abs(w*re);

    if (isnan(Ox) || isinf(Ox)) {
        Ox = 1e4;
    }

    return Ox;
}

double Lugre::computeOy(const Tire &tire, const double re, const double w, const Matrix<double,2,1> &vr, const double L, int tireId ) {

    const double sig0_y = tire.sigma(0,1);

    //Calculate sliding friction
    const double g = computeG(tire,vr);

    //Steady state constant calculation
    const double ky = computeKi(w,vr,L,re,g,sig0_y);

    double Oy = vr.norm()*sig0_y/g + (ky/L)*abs(w*re);

    if (isnan(Oy) || isinf(Oy)) {
        Oy = 1e4;
    }

    return Oy;
}


double Lugre::computeKi(const double w, const Matrix<double,2,1> &vr, const double L, const double re, const double g, const double sig0_i)  {
    
    // compute zi ( deformation )
    double zi = abs(w*re)*g/(vr.norm()*sig0_i);
    if(isnan(zi) || isinf(zi)){
        zi = _eps;
    }

    const double aux = L/zi+sqrtf(_eps);
    double ki = (1-exp(-aux))/(1-(1-exp(-aux))* zi/(L+_eps));

    if (isnan(ki) || isinf(ki)) {
        ki = _eps;
    }
    
    return ki;
}

double Lugre::computeG(const Tire &tire, const Matrix<double,2,1> &vr) {
    Matrix<double,2,2> mk = tire.mu_k;
    Matrix<double,2,2> ms = tire.mu_s;
    const double gamma = tire.gamma;
    const double vs = tire.vs;
    double g;

    //Sliding friction calculation
    if(vr.norm() < 1e-10) {
        JacobiSVD<Matrix2d> svd(ms);
        g = svd.singularValues().maxCoeff();
    }
    else {
        Matrix<double,2,2> _aux1 = mk.array().pow(2);
        Matrix<double,2,1> _aux2 = _aux1*vr;
        Matrix<double,2,1> _aux3 = mk*vr;
        Matrix<double,2,2> _aux4 = ms.array().pow(2);
        Matrix<double,2,1> _aux5 = _aux4*vr;
        Matrix<double,2,1> _aux6 = ms*vr;

        g = _aux2.norm()/_aux3.norm()+(_aux5.norm()/_aux6.norm()-_aux2.norm()/_aux3.norm())*expf(-pow((vr.norm()/vs), gamma));

        if (isnan(g) || isinf(g)) {
            JacobiSVD<Matrix2d> svd(ms);
            g = svd.singularValues().maxCoeff();
        }

        if (isnan(g) || isinf(g)) {
            JacobiSVD<Matrix2d> svd(ms);
            g = svd.singularValues().maxCoeff();
        }
    }
    return g;
}

void Lugre::extractTireInfo(const TireArray& tireArray, const Eigen::Matrix<double, 4, 1>& steerAngle, const Eigen::Matrix<double, 15, 1>& x, const Eigen::Vector4d& FnArray, TireInfoArray& tireInfoArray)
{

    tireInfoArray.resize(4);

    // Extract final velocities
    const double vx = x(12);
    const double vy = x(13);
    const double wz = x(14);

    for (int j = 0; j < 4; ++j) {
        const Tire& tire = tireArray[j];
        const double delta = steerAngle(j);
        const double w  = x(j*3 + 2);

        const double rx = tire.p(0);
        const double ry = tire.p(1);
        const double Fn = std::fmax(FnArray(j), 1.0);
        const double rl = tire.r - Fn / tire.kz;
        const double L  = 2.0 * std::sqrt(std::fmax(std::pow(tire.r, 2) - std::pow(rl, 2), _eps));

        // Calculate relative velocities based on final state
        Eigen::Vector2d v_tire = velocityTransform(delta, rx, ry, vx, vy, wz);
        Eigen::Vector2d vr(w * rl - v_tire(0), -v_tire(1));

        // Calculate friction base and stiffness once
        double g = computeG(tire, vr);
        double kx = computeKi(w, vr, L, rl, g, tire.sigma(0,0));
        double ky = computeKi(w, vr, L, rl, g, tire.sigma(0,1));

        // Populate your struct (assuming _tireInfoArray supports indexing [j])
        tireInfoArray[j].Gx = g;
        tireInfoArray[j].Gy = g;
        tireInfoArray[j].Kx = kx;
        tireInfoArray[j].Ky = ky;
        tireInfoArray[j].Vrx = vr(0);
        tireInfoArray[j].Vry = vr(1);
        
        // Use the helper functions for the final bristle parameters
        tireInfoArray[j].Ox = computeOx(tire, rl, w, vr, L, j);
        tireInfoArray[j].Oy = computeOy(tire, rl, w, vr, L, j);
    }
}