#include <iostream>
#include <Eigen/Dense>

class KalmanFilter {
public:
    // Constructor
    KalmanFilter() {
        // Initialize state and covariance matrices
        state.orientation.setZero();
        state.gyro_bias.setZero();
        P = Eigen::MatrixXf::Identity(6, 6);
    }

    // Update the filter with new gyro and accelerometer data
    void update(const Eigen::Vector3f& gyro, const Eigen::Vector3f& acc, float dt) {
        // Prediction step
        State predicted_state = predict(state, gyro, dt);
        Eigen::MatrixXf F = processModelJacobian(state, gyro, dt);
        Eigen::MatrixXf Q = processNoiseCovariance(dt);
        Eigen::MatrixXf P_pred = F * P * F.transpose() + Q;

        // Correction step
        Eigen::Vector3f y = acc - measure(predicted_state);
        Eigen::MatrixXf H = measurementModelJacobian(predicted_state);
        Eigen::MatrixXf R = measurementNoiseCovariance();
        Eigen::MatrixXf S = H * P_pred * H.transpose() + R;
        Eigen::MatrixXf K = P_pred * H.transpose() * S.inverse();

        state = predicted_state + K * y;
        P = (Eigen::MatrixXf::Identity(6, 6) - K * H) * P_pred;
    }

    // Get the current state
    const State& getState() const {
        return state;
    }

private:
    struct State {
        Eigen::Vector3f orientation;  // Orientation (e.g., Euler angles)
        Eigen::Vector3f gyro_bias;    // Gyroscope bias
    };

    State state;
    Eigen::MatrixXf P;  // Covariance matrix

    // Prediction step: predict the next state
    State predict(const State& state, const Eigen::Vector3f& gyro, float dt) const {
        State new_state;
        new_state.orientation = state.orientation + (gyro - state.gyro_bias) * dt;
        new_state.gyro_bias = state.gyro_bias;  // Assume bias is constant
        return new_state;
    }

    // Measurement model: compute the expected accelerometer measurement
    Eigen::Vector3f measure(const State& state) const {
        Eigen::Vector3f gravity_world(0, 0, -9.81);
        Eigen::Matrix3f R = rotationMatrix(state.orientation);
        return R.transpose() * gravity_world;
    }

    // Process model Jacobian
    Eigen::MatrixXf processModelJacobian(const State& state, const Eigen::Vector3f& gyro, float dt) const {
        Eigen::MatrixXf F = Eigen::MatrixXf::Identity(6, 6);
        F.block<3, 3>(0, 3) = -Eigen::Matrix3f::Identity() * dt;
        return F;
    }

    // Measurement model Jacobian
    Eigen::MatrixXf measurementModelJacobian(const State& state) const {
        Eigen::Matrix3f R = rotationMatrix(state.orientation);
        Eigen::MatrixXf H = Eigen::MatrixXf::Zero(3, 6);
        H.block<3, 3>(0, 0) = R.transpose() * skewSymmetric(Eigen::Vector3f(0, 0, -9.81));
        return H;
    }

    // Process noise covariance
    Eigen::MatrixXf processNoiseCovariance(float dt) const {
        Eigen::MatrixXf Q = Eigen::MatrixXf::Identity(6, 6);
        Q.block<3, 3>(0, 0) *= 0.01;  // Adjust these values as needed
        Q.block<3, 3>(3, 3) *= 0.001;
        return Q;
    }

    // Measurement noise covariance
    Eigen::MatrixXf measurementNoiseCovariance() const {
        Eigen::MatrixXf R = Eigen::MatrixXf::Identity(3, 3) * 0.1;  // Adjust this value as needed
        return R;
    }

    // Helper function to compute the rotation matrix from Euler angles
    Eigen::Matrix3f rotationMatrix(const Eigen::Vector3f& euler) const {
        float roll = euler(0);
        float pitch = euler(1);
        float yaw = euler(2);

        Eigen::Matrix3f Rx;
        Rx << 1, 0, 0,
              0, std::cos(roll), -std::sin(roll),
              0, std::sin(roll), std::cos(roll);

        Eigen::Matrix3f Ry;
        Ry << std::cos(pitch), 0, std::sin(pitch),
              0, 1, 0,
              -std::sin(pitch), 0, std::cos(pitch);

        Eigen::Matrix3f Rz;
        Rz << std::cos(yaw), -std::sin(yaw), 0,
              std::sin(yaw), std::cos(yaw), 0,
              0, 0, 1;

        return Rz * Ry * Rx;
    }

    // Helper function to compute the skew-symmetric matrix
    Eigen::Matrix3f skewSymmetric(const Eigen::Vector3f& v) const {
        Eigen::Matrix3f S;
        S << 0, -v(2), v(1),
             v(2), 0, -v(0),
             -v(1), v(0), 0;
        return S;
    }
};

int main() {
    KalmanFilter kf;

    // Simulate IMU data
    Eigen::Vector3f gyro(0.1, 0.2, 0.3);
    Eigen::Vector3f acc(0.0, 0.0, -9.81);
    float dt = 0.01;

    for (int i = 0; i < 100; ++i) {
        kf.update(gyro, acc, dt);
        const auto& state = kf.getState();
        std::cout << "Orientation: " << state.orientation.transpose() << std::endl;
    }

    return 0;
}