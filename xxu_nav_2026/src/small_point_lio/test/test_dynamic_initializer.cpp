#include "dynamic_initialization_fixture.h"

#include <Eigen/Eigenvalues>
#include <cmath>
#include <cstdlib>
#include <iostream>
#include <limits>
#include <random>

using namespace small_point_lio::initialization;
using namespace initialization_test;
using V3 = Eigen::Vector3d;
using M3 = Eigen::Matrix3d;

namespace {
    void require(bool condition, const std::string &message) {
        if (!condition) {
            std::cerr << "FAIL: " << message << '\n';
            std::exit(1);
        }
    }
    Result run(const Data &data) { return initialize(data.scans, data.imu, Options{}, V3(0, 0, -9.81)); }
    void print(const Result &r) {
        std::cout << r.reason << " plane=" << r.plane_rms << " imu=" << r.imu_rms
                  << " information=" << r.information_ratio << " bg=" << r.gyro_bias.transpose()
                  << " ba=" << r.accel_bias.transpose() << '\n';
    }
}// namespace

int main() {
    const auto moving = make_data(true);
    const auto result = run(moving);
    print(result);
    require(result.success, "moving startup must initialize without a stationary interval");
    const V3 bg(0.01, -0.015, 0.02), ba(0.08, -0.05, 0.1), g(0, 0, -9.81);
    const auto stationary = make_data(false);
    const auto certified = initialize_stationary(stationary.scans, stationary.imu, Options{}, g);
    print(certified);
    require(certified.success && certified.method == Method::Static, "IMU and lidar certify stationary startup");
    require(certified.velocity.norm() == 0 && certified.accel_bias.norm() == 0,
            "stationary branch retains the usual zero-velocity/bias assumptions");
    auto constant_accel = stationary;
    for (auto &scan: constant_accel.scans) {
        for (auto &p: scan.points) {
            p.body -= rotation(0).transpose() * V3(0.15 * p.time * p.time, 0, 0);
        }
    }
    for (auto &imu: constant_accel.imu) { imu.force += rotation(0).transpose() * V3(0.3, 0, 0); }
    const auto moving_but_quiet = initialize_stationary(constant_accel.scans, constant_accel.imu, Options{}, g);
    require(!moving_but_quiet.success && moving_but_quiet.reason == "lidar_motion_detected",
            "constant acceleration cannot pass the automatic stationary branch");
    auto constant_velocity = stationary;
    for (auto &scan: constant_velocity.scans) {
        for (auto &p: scan.points) { p.body -= rotation(0).transpose() * V3(0.1 * p.time, 0, 0); }
    }
    require(!initialize_stationary(constant_velocity.scans, constant_velocity.imu, Options{}, g).success,
            "constant velocity is rejected even with stationary-looking IMU");
    const auto floor_only = make_data(false, true);
    require(!initialize_stationary(floor_only.scans, floor_only.imu, Options{}, g).success,
            "one plane cannot certify stationary motion in all directions");
    const double start = moving.scans.front().time, end = moving.scans.back().time;
    const M3 r0 = rotation(start);
    const M3 align = Eigen::Quaterniond::FromTwoVectors(r0.transpose() * g, g).toRotationMatrix();
    const M3 expected_rotation = align * r0.transpose() * rotation(end);
    const V3 expected_position = align * r0.transpose() * (position(end) - position(start));
    const V3 expected_velocity = align * r0.transpose() * velocity(end);
    const double angle_error = Eigen::AngleAxisd(expected_rotation.transpose() * result.rotation).angle();
    std::cout << "errors rotation=" << angle_error << " position=" << (result.position - expected_position).norm()
              << " velocity=" << (result.velocity - expected_velocity).norm() << '\n';
    require(angle_error < 0.02, "terminal orientation error below 0.02 rad");
    require((result.position - expected_position).norm() < 0.05, "terminal position error below 5 cm");
    require((result.velocity - expected_velocity).norm() < 0.08, "terminal velocity error below 0.08 m/s");
    require((result.gyro_bias - bg).norm() < 0.01, "gyro bias recovered");
    require((result.accel_bias - ba).norm() < 0.1, "accelerometer bias recovered");
    require((result.gravity - g).norm() < 1e-8, "gravity aligned to configured frame");
    require(result.map.size() == 12 * 147, "all selected points initialize the map");
    const Eigen::SelfAdjointEigenSolver<Eigen::Matrix<double, 30, 30>> eigen(result.covariance);
    require(result.covariance.allFinite() && eigen.eigenvalues()[0] > -1e-9, "finite PSD filter covariance");

    auto bad = moving;
    bad.imu[100].time = bad.imu[99].time;
    require(run(bad).reason == "imu_time_discontinuity", "duplicate IMU timestamp rejected");
    bad = moving;
    bad.imu.erase(bad.imu.begin() + 100, bad.imu.begin() + 170);
    require(run(bad).reason == "imu_time_discontinuity", "IMU gap rejected");
    bad = moving;
    bad.imu[100].force.x() = std::numeric_limits<double>::quiet_NaN();
    require(run(bad).reason == "nonfinite_imu", "NaN rejected");
    bad = moving;
    bad.imu[100].gyro.x() = 40;
    require(run(bad).reason == "imu_saturated", "saturated gyro rejected");
    bad = moving;
    bad.imu[100].force.x() = 40;
    require(run(bad).reason == "imu_saturated", "saturated accelerometer rejected");
    bad = moving;
    bad.scans[0].points[0].time = -0.1;
    require(run(bad).reason == "unbracketed_or_invalid_point", "point before IMU coverage rejected");
    bad = moving;
    bad.scans.resize(4);
    require(!run(bad).success, "short window rejected");
    const auto static_result = run(make_data(false));
    print(static_result);
    require(!static_result.success, "unobservable gravity/bias separation rejected despite low residual");
    const auto planar_result = run(make_data(true, true));
    print(planar_result);
    require(planar_result.reason == "insufficient_plane_geometry", "single-plane geometry rejected");
    const auto instantaneous = run(make_data(true, false, 0.0));
    print(instantaneous);
    require(instantaneous.success, "instantaneous scans also supported");
    auto noisy = moving;
    std::mt19937 generator(20261026);
    std::normal_distribution<double> noise(0.0, 1.0);
    for (auto &scan: noisy.scans) {
        for (auto &p: scan.points) {
            for (int axis = 0; axis < 3; ++axis) { p.body[axis] += 0.003 * noise(generator); }
        }
    }
    for (auto &imu: noisy.imu) {
        for (int axis = 0; axis < 3; ++axis) {
            imu.gyro[axis] += 0.002 * noise(generator);
            imu.force[axis] += 0.02 * noise(generator);
        }
    }
    const auto noisy_result = run(noisy);
    print(noisy_result);
    require(noisy_result.success, "noisy moving window accepted");
    require((noisy_result.gyro_bias - bg).norm() < 0.02, "noisy gyro bias within 0.02 rad/s");
    require((noisy_result.accel_bias - ba).norm() < 0.2, "noisy accel bias within 0.2 m/s^2");
    // A turning gimbal with a stationary origin is the motivating restart case.
    auto yaw_only = make_data(false);
    for (auto &scan: yaw_only.scans) {
        for (auto &p: scan.points) {
            const V3 world = rotation(0) * p.body + position(0);
            p.body = Eigen::AngleAxisd(-0.6 * p.time, V3::UnitZ()) * world;
        }
    }
    for (auto &imu: yaw_only.imu) {
        imu.gyro = V3(0, 0, 0.6) + bg;
        imu.force = -g + ba;
    }
    const auto yaw_result = run(yaw_only);
    print(yaw_result);
    require(yaw_result.success, "yaw-only moving initialization supported");
    require((yaw_result.accel_bias - ba).norm() < 0.1, "yaw-only accelerometer bias recovered");
    auto translation_only = make_data(false);
    for (auto &scan: translation_only.scans) {
        for (auto &p: scan.points) {
            const V3 world = rotation(0) * p.body + position(0);
            p.body = rotation(0).transpose() * (world - position(p.time));
        }
    }
    for (auto &imu: translation_only.imu) {
        imu.force = rotation(0).transpose() * (acceleration(imu.time) - g) + ba;
    }
    const auto translation_result = run(translation_only);
    print(translation_result);
    require(!translation_result.success, "fixed-attitude translation cannot falsely separate gravity and bias");
    auto invalid_options = Options{};
    invalid_options.gravity = std::numeric_limits<double>::infinity();
    require(initialize(moving.scans, moving.imu, invalid_options, g).reason == "invalid_options", "infinite option rejected");
    auto epoch = moving;
    for (auto &imu: epoch.imu) { imu.time += 1700000000.0; }
    for (auto &scan: epoch.scans) {
        scan.time += 1700000000.0;
        for (auto &p: scan.points) { p.time += 1700000000.0; }
    }
    const auto epoch_result = run(epoch);
    print(epoch_result);
    require(epoch_result.success, "epoch-sized sensor timestamps supported");
    std::cout << "All dynamic initialization checks passed\n";
}
