#include "measure.h"

Measure::Measure() {
    state.initialized = false;
}

Measure::~Measure() {}

// process a measurement
void Measure::process(const MeasureData md) {
    if (!state.initialized) {
        init(md);
        return;
    }
    #ifdef __DEBUG__
        printf("x : %.3f\n", md.x);
        printf("y : %.3f\n", md.y);
        printf("vx : %.3f\n", md.vx);
        printf("vy : %.3f\n", md.vy);
        printf("rho : %.3f\n", md.rho);
        printf("phi : %.3f\n", md.phi);
        printf("rhodot : %.3f\n", md.rhodot);
        printf("timestamp : %d\n", md.timestamp);
    #endif
    
    predict(md);
    VectorXd z;
    switch(md.type) {
        case LIDAR:
            z = VectorXd(2);
            z << md.x, md.y;
            updateLidar(z);
            break;
        case RADAR:
            z = VectorXd(3);
            z << md.rho, md.phi, md.rhodot;
            updateRadar(z);
            break;
    }
    
    state.timestamp = md.timestamp;
    #ifdef __DEBUG__
        cout << "x:" << endl << state.x << endl;
        cout << "P:" << endl << state.P << endl;
    #endif
}

// getStateVector returns the state vector x
VectorXd Measure::getStateVector() {
    return state.x;
}

// init initializes the state
void Measure::init(const MeasureData md) {
    state.x = VectorXd(4);
    if (md.type == LIDAR) {
        state.x << md.x, md.y, 1, 1;
    } else if (md.type == RADAR) {
        state.x << md.rho * cos(md.phi),
                   md.rho * sin(md.phi),
                   md.rhodot * cos(md.phi),
                   md.rhodot * sin(md.phi);
    }
    state.P = MatrixXd(4, 4);
    state.P << 1, 0, 0, 0,
               0, 1, 0, 0,
               0, 0, 100, 0,
               0, 0, 0, 100;
    state.timestamp = md.timestamp;
    state.initialized = true;
}

void Measure::predict(const MeasureData md) {
    float dt = (md.timestamp - state.timestamp) / 1000000.0; // unit : s
    MatrixXd F(4, 4);
    F << 1, 0, dt, 0,
         0, 1, 0, dt,
         0, 0, 1, 0,
         0, 0, 0, 1;
    MatrixXd Q(4, 4);

    const float dt44 = pow(dt, 4) / 4.0;
    const float dt32 = pow(dt, 3) / 2.0;
    const float dt2 = pow(dt, 2);

    const float noise_ax = 9;
    const float noise_ay = 9;
    Q << dt44 * noise_ax, 0, dt32 * noise_ax, 0,
         0, dt44 * noise_ay, 0, dt32 * noise_ay,
         dt32 * noise_ax, 0, dt2 * noise_ax, 0,
         0, dt32 * noise_ay, 0, dt2 * noise_ay;

    state.x = F * state.x;
    state.P = F * state.P * F.transpose() + Q;
}

void Measure::updateLidar(const VectorXd z) {
    MatrixXd H(2, 4), R(2, 2);
    H << 1, 0, 0, 0,
         0, 1, 0, 0;
    R << 0.0225, 0,
         0, 0.0225;
    
    const MatrixXd Ht = H.transpose();

    const VectorXd y = z - H * state.x;
    const MatrixXd S = H * state.P * Ht + R;
    const MatrixXd K = state.P * Ht * S.inverse();
    
    state.x = state.x + K * y;
    const MatrixXd I = MatrixXd::Identity(4, 4);
    state.P = (I - K * H) * state.P;
}

void Measure::updateRadar(const VectorXd z) {
    float px = state.x(0);
    float py = state.x(1);
    float vx = state.x(2);
    float vy = state.x(3);

    float rho = sqrt(pow(px, 2) + pow(py, 2));
    float phi = atan2(py, px);
    float rhodot = 0;
    
    if (rho != 0) {
        rhodot = (px * vx + py * vy) / rho;
    }

    VectorXd h(3);
    h << rho, phi, rhodot;
    VectorXd y = z - h;

    const float PI = 3.1415926;
    if (y(1) > PI) {
        y(1) -= 2 * PI;
    }
    if (y(1) < -PI) {
        y(1) += 2 * PI;
    }

    const MatrixXd H = jacobian(state.x);
    const MatrixXd Ht = H.transpose();
    MatrixXd R = MatrixXd(3, 3);
    R << 0.09, 0, 0,
         0, 0.0009, 0,
         0, 0, 0.09;
    const MatrixXd S = H * state.P * Ht + R;
    const MatrixXd K = state.P * Ht * S.inverse();

    state.x = state.x + K * y;
    const MatrixXd I = MatrixXd::Identity(4, 4);
    state.P = (I - K * H) * state.P;
}

MatrixXd jacobian(const VectorXd x) {
    MatrixXd H(3, 4);
    float px = x(0);
    float py = x(1);
    float vx = x(2);
    float vy = x(3);

    float rho2 = pow(px, 2) + pow(py, 2);
    if (rho2 == 0) {
        // division by 0, skipping
        return H;
    }
    float rho = sqrt(rho2);
    float rho32 = rho * rho2;
    float vxpy = vx * py;
    float vypx = vy * px;

    H << px / rho, py / rho, 0, 0,
         -py / rho2, px / rho2, 0, 0,
         py * (vxpy - vypx) / rho32, px * (vypx - vxpy) / rho32, px / rho, py / rho;
    return H;
}

