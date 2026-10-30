#pragma once

#include "small_point_lio/dynamic_initializer.h"
#include <cmath>

namespace initialization_test {
    using namespace small_point_lio::initialization;
    using V3 = Eigen::Vector3d;
    using M3 = Eigen::Matrix3d;

    M3 rotation(double t) {
        return (Eigen::AngleAxisd(0.4 * t + 0.15 * std::sin(0.8 * t), V3::UnitZ()) *
                Eigen::AngleAxisd(0.25 * std::sin(1.1 * t) - 0.12, V3::UnitY()) *
                Eigen::AngleAxisd(0.3 * std::sin(1.5 * t) + 0.15, V3::UnitX()))
                .toRotationMatrix();
    }
    V3 position(double t) { return {0.3 * t + 0.1 * std::sin(2 * t), 0.15 * std::sin(1.3 * t), 0.1 * std::sin(t)}; }
    V3 velocity(double t) { return {0.3 + 0.2 * std::cos(2 * t), 0.195 * std::cos(1.3 * t), 0.1 * std::cos(t)}; }
    V3 acceleration(double t) { return {-0.4 * std::sin(2 * t), -0.2535 * std::sin(1.3 * t), -0.1 * std::sin(t)}; }
    V3 angular_velocity(double t) {
        constexpr double h = 1e-5;
        const Eigen::AngleAxisd delta(rotation(t - h).transpose() * rotation(t + h));
        return delta.axis() * (delta.angle() / (2 * h));
    }
    struct Data {
        std::vector<Scan> scans;
        std::vector<Imu> imu;
    };
    Data make_data(bool moving, bool one_plane = false, double scan_duration = 0.1) {
        Data data;
        const V3 bg(0.01, -0.015, 0.02), ba(0.08, -0.05, 0.1), g(0, 0, -9.81);
        for (int i = 0; i <= 500; ++i) {
            const double t = i * 0.005;
            const M3 r = moving ? rotation(t) : rotation(0);
            data.imu.push_back({t, (moving ? angular_velocity(t) : V3::Zero()) + bg,
                                r.transpose() * ((moving ? acceleration(t) : V3::Zero()) - g) + ba});
        }
        std::vector<V3> world;
        for (int i = -3; i <= 3; ++i) {
            for (int j = -3; j <= 3; ++j) {
                world.emplace_back(i * 0.3, j * 0.3, -2.0);
                if (!one_plane) {
                    world.emplace_back(3.0, i * 0.3, j * 0.3);
                    world.emplace_back(i * 0.3, 3.0, j * 0.3);
                }
            }
        }
        for (int i = 0; i < 12; ++i) {
            Scan scan;
            scan.time = 0.12 + 0.2 * i;
            for (std::size_t j = 0; j < world.size(); ++j) {
                const double t = scan.time - scan_duration + scan_duration * double(j) / double(world.size() - 1);
                const M3 r = moving ? rotation(t) : rotation(0);
                const V3 p = moving ? position(t) : position(0);
                scan.points.push_back({t, r.transpose() * (world[j] - p)});
            }
            data.scans.push_back(std::move(scan));
        }
        return data;
    }
}// namespace initialization_test
