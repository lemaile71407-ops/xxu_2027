#pragma once

#include <Eigen/Core>
#include <Eigen/Geometry>
#include <string>
#include <vector>

namespace small_point_lio::initialization {

    // Independent batch initializer. All accelerations here are SI specific force;
    // points and IMU must describe the SAME rigid sensor, after gimbal compensation.
    struct Imu {
        double time = 0.0;
        Eigen::Vector3d gyro = Eigen::Vector3d::Zero();
        Eigen::Vector3d force = Eigen::Vector3d::Zero();
    };

    struct Point {
        double time = 0.0;
        Eigen::Vector3d body = Eigen::Vector3d::Zero();
    };

    struct Scan {
        double time = 0.0;// End of the scan, also the optimization knot.
        std::vector<Point> points;
    };

    struct Options {
        double min_duration = 2.0;
        double max_imu_gap = 0.25;
        double gravity = 9.81;
        double max_gyro = 35.0;
        double max_force = 30.0;
        int min_matches = 40;
        double match_distance = 0.8;
        double plane_sigma = 0.03;
        double gyro_noise_density = 0.01;
        double accel_noise_density = 0.1;
        double gyro_measurement_variance = 0.01;
        double accel_measurement_variance = 0.01;
    };

    enum class Method { None,
                        Static,
                        Motion };

    struct Result {
        bool success = false;
        Method method = Method::Motion;
        std::size_t frame_count = 0;
        std::string reason = "waiting_for_window";
        double time = 0.0;
        double plane_rms = 0.0;
        double imu_rms = 0.0;
        double information_ratio = 0.0;
        Eigen::Matrix3d rotation = Eigen::Matrix3d::Identity();
        Eigen::Vector3d position = Eigen::Vector3d::Zero();
        Eigen::Vector3d velocity = Eigen::Vector3d::Zero();
        Eigen::Vector3d gyro_bias = Eigen::Vector3d::Zero();
        Eigen::Vector3d accel_bias = Eigen::Vector3d::Zero();
        Eigen::Vector3d gravity = Eigen::Vector3d::Zero();
        Eigen::Vector3d omega = Eigen::Vector3d::Zero();
        Eigen::Vector3d force = Eigen::Vector3d::Zero();
        // Order matches eskf.h: p, R, R_LI, t_LI, v, omega, force, g, bg, ba.
        Eigen::Matrix<double, 30, 30> covariance = Eigen::Matrix<double, 30, 30>::Zero();
        std::vector<Eigen::Vector3f> map;
    };

    // Does not mutate the live filter/map. Failure is safe to retry on a newer window.
    Result initialize(const std::vector<Scan> &scans, const std::vector<Imu> &imu,
                      const Options &options, const Eigen::Vector3d &target_gravity);

    // Certify a stationary interval using BOTH inertial and lidar evidence,
    // then construct the usual stationary initial state. This does not claim
    // to identify accelerometer bias independently from gravity at rest.
    Result initialize_stationary(const std::vector<Scan> &scans, const std::vector<Imu> &imu,
                                 const Options &options, const Eigen::Vector3d &target_gravity);

}// namespace small_point_lio::initialization
