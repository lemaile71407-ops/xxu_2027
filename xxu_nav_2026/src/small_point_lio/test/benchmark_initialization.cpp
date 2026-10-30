#include "dynamic_initialization_fixture.h"
#include "frontend_initialization_options.h"
#include "small_point_lio/small_point_lio.h"

#include <chrono>
#include <cmath>
#include <ctime>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <random>
#include <string>
#include <thread>

// Manual comparison, intentionally not in ctest: replay is paced in wall time
// so the asynchronous initializer receives new measurements while solving.
namespace {
    using namespace initialization_test;
    using Clock = std::chrono::steady_clock;
    constexpr double pi = 3.14159265358979323846;
    const V3 gravity(0, 0, -9.81), bg(0.01, -0.015, 0.02), ba(0.08, -0.05, 0.1);

    double yaw_angle(double t, bool stop) {
        if (!stop || t <= 2.0) { return 0.6 * t; }
        if (t >= 3.0) { return 1.5; }
        const double u = t - 2.0;
        return 1.2 + 0.3 * (u + std::sin(pi * u) / pi);
    }
    struct Scenario {
        std::string name;
        bool plane = false;
        M3 r(double t) const {
            if (name == "moving_room") { return rotation(t); }
            if (name == "yaw_room" || name == "single_plane_yaw" || name == "turn_then_stop") {
                return Eigen::AngleAxisd(yaw_angle(t, name == "turn_then_stop"), V3::UnitZ()).toRotationMatrix();
            }
            if (name == "stationary_then_move" && t > 2.0) { return rotation(t - 2.0); }
            return rotation(0);
        }
        V3 p(double t) const {
            if (name == "moving_room") { return position(t) - position(0); }
            if (name == "constant_accel") { return {0.15 * t * t, 0, 0}; }
            if (name == "stationary_then_move" && t > 2.0) {
                // Smoothly starts from zero velocity and acceleration.
                const double u = t - 2.0;
                return {0.03 * u * u * u, 0.03 * (1.0 - std::cos(u)), 0};
            }
            return V3::Zero();
        }
        V3 v(double t) const {
            constexpr double h = 1e-4;
            return (p(t + h) - p(t - h)) / (2 * h);
        }
        V3 a(double t) const {
            constexpr double h = 1e-4;
            return (p(t + h) - 2.0 * p(t) + p(t - h)) / (h * h);
        }
        V3 w(double t) const {
            constexpr double h = 1e-5;
            const Eigen::AngleAxisd delta(r(t - h).transpose() * r(t + h));
            return delta.axis() * (delta.angle() / (2 * h));
        }
    };

    Data generate(const Scenario &scenario, int seed, double duration) {
        Data data;
        std::mt19937 random(seed);
        std::normal_distribution<double> normal(0.0, 1.0);
        const auto noise = [&](double sigma) -> V3 {
            return {sigma * normal(random), sigma * normal(random), sigma * normal(random)};
        };
        for (int i = 0; i <= int(std::round(duration * 200)); ++i) {
            const double t = i / 200.0;
            data.imu.push_back({t, scenario.w(t) + bg + noise(0.002),
                                scenario.r(t).transpose() * (scenario.a(t) - gravity) + ba + noise(0.02)});
        }
        std::vector<V3> world;
        for (int i = -4; i <= 4; ++i) {
            for (int j = -4; j <= 4; ++j) {
                world.emplace_back(i * 0.25, j * 0.25, -2.0);
                if (!scenario.plane) {
                    world.emplace_back(3.0, i * 0.25, j * 0.25);
                    world.emplace_back(i * 0.25, 3.0, j * 0.25);
                }
            }
        }
        for (int i = 1; i <= int(std::round(duration * 10)); ++i) {
            Scan scan;
            scan.time = i / 10.0;
            for (std::size_t j = 0; j < world.size(); ++j) {
                const double t = scan.time - 0.1 + 0.1 * double(j) / double(world.size() - 1);
                scan.points.push_back({std::min(t, scan.time),
                                       scenario.r(t).transpose() * (world[j] - scenario.p(t)) + noise(0.003)});
            }
            data.scans.push_back(std::move(scan));
        }
        return data;
    }

    double yaw(const M3 &r) { return std::atan2(r(1, 0), r(0, 0)); }
    double tilt_error(const M3 &estimated, const M3 &truth) {
        return std::acos(std::clamp((estimated.transpose() * V3::UnitZ()).dot(truth.transpose() * V3::UnitZ()), -1.0, 1.0)) * 180.0 / pi;
    }
    struct Outcome {
        bool initialized = false, fault = false, invalid = false;
        std::string selected_path = "none";
        double first_stamp = NAN, first_delivery = NAN, wall_ready = NAN;
        double initial_tilt = NAN, initial_velocity_error = NAN;
        double drift2 = NAN, rmse2 = NAN, final_error = NAN, post_duration = NAN;
        double cpu_seconds = 0, max_callback_ms = 0, max_schedule_lag_ms = 0;
        std::vector<common::Odometry> poses;
    };

    Outcome replay(const Scenario &scenario, const Data &data, bool dynamic, const std::string &map_backend) {
        auto options = frontend_options(dynamic);
        auto overrides = options.parameter_overrides();
        overrides.emplace_back("map_backend", map_backend);
        options.parameter_overrides(overrides);
        rclcpp::Node node("initialization_comparison", options);
        node.get_logger().set_level(rclcpp::Logger::Level::Error);
        small_point_lio::SmallPointLio lio(node);
        Outcome out;
        double current_delivery = 0;
        const auto wall_start = Clock::now();
        const auto cpu_start = std::clock();
        lio.set_pose_history_callback([&](const common::Odometry &pose) {
            if (!pose.position.allFinite() || !pose.velocity.allFinite() || !pose.orientation.coeffs().allFinite() ||
                pose.position.norm() > 1000) {
                out.invalid = true;
                return;
            }
            if (!out.initialized) {
                out.initialized = true;
                out.selected_path = lio.initialization_method() == small_point_lio::initialization::Method::Static ? "static" : "motion";
                out.first_stamp = pose.timestamp;
                out.first_delivery = current_delivery;
                out.wall_ready = std::chrono::duration<double>(Clock::now() - wall_start).count();
                out.initial_tilt = tilt_error(pose.orientation.toRotationMatrix(), scenario.r(pose.timestamp));
            }
            if (!out.poses.empty() && std::abs(out.poses.back().timestamp - pose.timestamp) < 1e-9) {
                out.poses.back() = pose;
            } else {
                out.poses.push_back(pose);
            }
        });
        const auto deliver = [&](double timestamp, const auto &callback) {
            current_delivery = timestamp;
            const auto scheduled = wall_start + std::chrono::duration_cast<Clock::duration>(std::chrono::duration<double>(timestamp));
            std::this_thread::sleep_until(scheduled);
            out.max_schedule_lag_ms = std::max(out.max_schedule_lag_ms,
                                               std::chrono::duration<double, std::milli>(Clock::now() - scheduled).count());
            const auto start = Clock::now();
            callback();
            lio.handle_once();
            out.max_callback_ms = std::max(out.max_callback_ms,
                                           std::chrono::duration<double, std::milli>(Clock::now() - start).count());
            out.fault = out.fault || lio.has_imu_time_fault();
        };
        std::size_t m = 0, s = 0;
        while ((m < data.imu.size() || s < data.scans.size()) && !out.invalid && !out.fault) {
            if (m < data.imu.size() && (s == data.scans.size() || data.imu[m].time <= data.scans[s].time)) {
                const auto &imu = data.imu[m++];
                deliver(imu.time, [&] { lio.on_imu_callback({imu.time, imu.force, imu.gyro}); });
            } else {
                const auto &scan = data.scans[s++];
                std::vector<common::Point> points;
                for (const auto &p: scan.points) { points.push_back({p.time, p.body.cast<float>()}); }
                deliver(scan.time, [&] { lio.on_point_cloud_callback(points); });
            }
        }
        // No post-recording grace period: a solve finishing after the observation
        // deadline does not count as a startup within that deadline. Reset joins
        // and discards pending work; its CPU is included in total cost.
        lio.reset();
        out.cpu_seconds = double(std::clock() - cpu_start) / CLOCKS_PER_SEC;
        if (out.poses.empty()) { return out; }
        const auto &first = out.poses.front(), &last = out.poses.back();
        const M3 alignment = Eigen::AngleAxisd(yaw(first.orientation.toRotationMatrix()) - yaw(scenario.r(first.timestamp)), V3::UnitZ()).toRotationMatrix();
        const V3 translation = first.position - alignment * scenario.p(first.timestamp);
        const auto error = [&](const common::Odometry &pose) {
            return (pose.position - alignment * scenario.p(pose.timestamp) - translation).norm();
        };
        out.initial_velocity_error = (first.velocity - alignment * scenario.v(first.timestamp)).norm();
        out.final_error = error(last);
        out.post_duration = last.timestamp - first.timestamp;
        double squared = 0;
        int samples = 0;
        for (int k = 1; k <= 20; ++k) {
            const double t = first.timestamp + k * 0.1;
            const auto hi = std::lower_bound(out.poses.begin(), out.poses.end(), t,
                                             [](const auto &pose, double time) { return pose.timestamp < time; });
            if (hi == out.poses.end()) { break; }
            auto pose = *hi;
            if (hi != out.poses.begin()) {
                const auto &lo = *std::prev(hi);
                const double fraction = (t - lo.timestamp) / (hi->timestamp - lo.timestamp);
                pose.timestamp = t;
                pose.position = lo.position + fraction * (hi->position - lo.position);
            }
            const double e = error(pose);
            squared += e * e;
            ++samples;
            if (k == 20) { out.drift2 = e; }
        }
        if (samples == 20) { out.rmse2 = std::sqrt(squared / samples); }
        // Avoid carrying tens of thousands of poses into summary output.
        out.poses.clear();
        return out;
    }

    void number(std::ostream &out, double value) {
        if (std::isfinite(value)) { out << std::setprecision(10) << value; }
    }
    void write(std::ostream &csv, const Scenario &scenario, int seed, bool dynamic, double duration, const Outcome &out) {
        csv << scenario.name << ',' << seed << ',' << (dynamic ? "auto" : "static") << ',' << duration << ','
            << out.initialized << ',' << out.fault << ',' << out.invalid;
        for (double value: {out.first_stamp, out.first_delivery, out.wall_ready, out.initial_tilt,
                            out.initial_velocity_error, out.drift2, out.rmse2, out.final_error, out.post_duration,
                            out.cpu_seconds, out.max_callback_ms, out.max_schedule_lag_ms}) {
            csv << ',';
            number(csv, value);
        }
        csv << ',' << out.selected_path << '\n';
        csv.flush();
    }
}// namespace

int main(int argc, char **argv) {
    std::string csv_path, selected, map_backend = "ivox";
    int repeats = 3;
    double duration = 6.0;
    for (int i = 1; i < argc; ++i) {
        const std::string arg(argv[i]);
        if (arg == "--csv" && i + 1 < argc) {
            csv_path = argv[++i];
        } else if (arg == "--scenario" && i + 1 < argc) {
            selected = argv[++i];
        } else if (arg == "--map-backend" && i + 1 < argc) {
            map_backend = argv[++i];
        } else if (arg == "--repeats" && i + 1 < argc) {
            repeats = std::stoi(argv[++i]);
        } else if (arg == "--duration" && i + 1 < argc) {
            duration = std::stod(argv[++i]);
        } else {
            std::cerr << "Unknown argument " << arg << '\n';
            return 2;
        }
    }
    if ((map_backend != "ivox" && map_backend != "moment") || csv_path.empty() || repeats < 1 || repeats > 20 || duration < 4 || duration > 15) {
        std::cerr << "Usage: benchmark_initialization --csv PATH [--scenario NAME] [--repeats 3] [--duration 6] [--map-backend ivox|moment]\n";
        return 2;
    }
    const std::vector<Scenario> scenarios = {{"static_room"}, {"yaw_room"}, {"moving_room"}, {"turn_then_stop"}, {"stationary_then_move"}, {"constant_accel"}, {"single_plane_yaw", true}};
    if (!selected.empty() && std::none_of(scenarios.begin(), scenarios.end(), [&](const auto &s) { return s.name == selected; })) {
        std::cerr << "Unknown scenario " << selected << '\n';
        return 2;
    }
    std::ofstream csv(csv_path);
    if (!csv) {
        std::cerr << "Cannot create CSV\n";
        return 2;
    }
    csv << "scenario,seed,method,duration_s,initialized,imu_fault,invalid_state,initial_pose_stamp_s,first_delivery_s,wall_ready_s,initial_tilt_error_deg,initial_velocity_error_mps,drift_2s_m,position_rmse_2s_m,final_position_error_m,post_duration_s,cpu_s,max_callback_ms,max_schedule_lag_ms,selected_path\n";
    rclcpp::init(0, nullptr);
    int runs = 0;
    for (const auto &scenario: scenarios) {
        if (!selected.empty() && selected != scenario.name) { continue; }
        for (int repeat = 0; repeat < repeats; ++repeat) {
            const int seed = 20261027 + repeat;
            const auto data = generate(scenario, seed, duration);
            // Alternate order to reduce systematic temperature/cache ordering.
            for (int method = 0; method < 2; ++method) {
                const bool dynamic = (method + repeat) % 2 == 1;
                const auto out = replay(scenario, data, dynamic, map_backend);
                write(csv, scenario, seed, dynamic, duration, out);
                std::cout << "RESULT " << ++runs << ' ' << scenario.name << ' ' << seed << ' '
                          << (dynamic ? "auto" : "static") << " path=" << out.selected_path << " initialized=" << out.initialized
                          << " ready=" << out.first_delivery << " tilt=" << out.initial_tilt
                          << " drift2=" << out.drift2 << " cpu=" << out.cpu_seconds << std::endl;
            }
        }
    }
    rclcpp::shutdown();
    std::cout << "Completed " << runs << " runs\n";
}
