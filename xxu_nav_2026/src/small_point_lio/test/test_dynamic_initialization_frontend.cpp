#include "dynamic_initialization_fixture.h"
#include "frontend_initialization_options.h"
#include "small_point_lio/small_point_lio.h"

#include <cstdlib>
#include <iostream>
#include <thread>

using namespace initialization_test;
using small_point_lio::SmallPointLio;

namespace small_point_lio {
    struct SmallPointLioTestAccess {
        static const Preprocess &input(const SmallPointLio &lio) { return lio.preprocess; }
        static std::shared_ptr<SmallIVox> map(const SmallPointLio &lio) { return lio.estimator.ivox; }
        static bool pending(const SmallPointLio &lio) { return lio.dynamic_init_future.valid(); }
    };
}// namespace small_point_lio

namespace {
    void require(bool condition, const std::string &message) {
        if (!condition) {
            std::cerr << "FAIL: " << message << '\n';
            std::exit(1);
        }
    }
    void feed(SmallPointLio &lio, const Data &data) {
        // IMU leads the cloud stream. This exercises retaining future IMU across
        // the handoff, rather than clearing the entire input buffer on success.
        for (const auto &imu: data.imu) {
            lio.on_imu_callback({imu.time, imu.force, imu.gyro});
            lio.handle_once();
        }
        for (const auto &scan: data.scans) {
            std::vector<common::Point> points;
            for (const auto &p: scan.points) { points.push_back({p.time, p.body.cast<float>()}); }
            lio.on_point_cloud_callback(points);
            lio.handle_once();
        }
    }
    void finish_initializing(SmallPointLio &lio, const std::vector<common::Odometry> &poses) {
        const auto deadline = std::chrono::steady_clock::now() + std::chrono::seconds(10);
        while (poses.empty() && std::chrono::steady_clock::now() < deadline) {
            lio.handle_once();
            std::this_thread::sleep_for(std::chrono::milliseconds(5));
        }
        lio.handle_once();// Drain measurements retained beyond the solved knot.
    }
}// namespace

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    {
        rclcpp::Node node("test_static_startup_buffer_limit", frontend_options(false));
        SmallPointLio lio(node);
        for (int frame = 0; frame < 1000; ++frame) {
            std::vector<common::Point> points;
            for (int i = 0; i < 1000; ++i) {
                points.push_back({frame * 0.1 + i * 0.00001, Eigen::Vector3f(2, 0, 0)});
            }
            lio.on_point_cloud_callback(points);
            lio.handle_once();
        }
        const auto &input = small_point_lio::SmallPointLioTestAccess::input(lio);
        require(input.point_deque.size() <= 100000 && input.dense_point_deque.size() <= 300000,
                "missing IMU cannot create unbounded static startup queues");
        require(input.point_deque.back().timestamp - input.point_deque.front().timestamp <= 5.0,
                "static startup only retains a recent cloud window");
    }
    {
        rclcpp::Node node("test_motion_then_stationary_map", frontend_options(false));
        SmallPointLio lio(node);
        std::vector<common::Odometry> poses;
        lio.set_pose_history_callback([&](const auto &pose) { poses.push_back(pose); });
        for (int i = 0; i < 620 && poses.empty(); ++i) {
            const double time = 20.0 + i * 0.005;
            const bool moving = i < 400;
            lio.on_imu_callback({time, V3(0, 0, 9.81), moving ? V3(0, 0, 0.2) : V3::Zero()});
            if (i % 10 == 0) {
                std::vector<common::Point> points;
                for (int j = 0; j < 40; ++j) {
                    points.push_back({time, Eigen::Vector3f(moving ? 20.0f : 2.0f, j * 0.01f, 0)});
                }
                lio.on_point_cloud_callback(points);
            }
            lio.handle_once();
        }
        require(!poses.empty() && poses.front().timestamp >= 22.0, "startup waits for stationary evidence");
        std::vector<Eigen::Vector3f> neighbors;
        auto map = small_point_lio::SmallPointLioTestAccess::map(lio);
        map->get_closest_point(Eigen::Vector3f(20, 0, 0), neighbors);
        require(neighbors.empty(), "clouds collected during motion never seed the static map");
        map->get_closest_point(Eigen::Vector3f(2, 0, 0), neighbors);
        require(!neighbors.empty(), "stationary cloud seeds the map");
    }
    {
        rclcpp::Node node("test_static_frontier_handoff", frontend_options(false));
        SmallPointLio lio(node);
        std::vector<common::Odometry> poses;
        lio.set_pose_history_callback([&](const auto &pose) { poses.push_back(pose); });
        for (int i = 0; i < 200; ++i) {
            lio.on_imu_callback({1.0 + i * 0.005, V3(0, 0, 9.81), V3::Zero()});
            lio.handle_once();
        }
        std::vector<common::Point> points;
        for (int i = 0; i < 40; ++i) { points.push_back({1.9, Eigen::Vector3f(2, i * 0.01f, 0)}); }
        points.push_back({2.1, Eigen::Vector3f(3, 0, 0)});
        lio.on_point_cloud_callback(points);
        lio.handle_once();
        require(!poses.empty() && poses.front().timestamp <= 1.995, "initial pose is on the common sensor frontier");
        const auto &input = small_point_lio::SmallPointLioTestAccess::input(lio);
        require(input.point_deque.size() == 1 && input.point_deque.front().timestamp == 2.1,
                "future cloud is retained for the normal filter");
    }
    {
        auto opts = frontend_options(true);
        opts.append_parameter_override("planar_constraint_en", true);
        opts.append_parameter_override("planar_preserve_initial_tilt", false);
        rclcpp::Node node("test_incompatible_planar_projection", opts);
        bool rejected = false;
        try {
            SmallPointLio lio(node);
        } catch (const std::invalid_argument &) { rejected = true; }
        require(rejected, "hard level projection cannot silently overwrite solved orientation");
    }
    {
        rclcpp::Node node("test_dynamic_initialization", frontend_options(true));
        SmallPointLio lio(node);
        std::vector<common::Odometry> poses;
        lio.set_pose_history_callback([&](const auto &pose) { poses.push_back(pose); });
        const auto data = make_data(true);
        feed(lio, data);
        finish_initializing(lio, poses);
        require(!poses.empty(), "moving frontend initializes");
        require(lio.initialization_method() == small_point_lio::initialization::Method::Motion,
                "moving input selects the supplemental motion path");
        require(!small_point_lio::SmallPointLioTestAccess::pending(lio), "normal LIO does not keep running initialization");
        require(poses.front().timestamp >= 2.0 && poses.front().timestamp <= data.scans.back().time,
                "first pose uses a solved terminal knot");
        require(poses.back().timestamp > poses.front().timestamp,
                "queued measurements continue through the normal filter after handoff");
        require(poses.back().velocity.allFinite() && poses.back().position.allFinite(), "normal filter state remains finite");
        for (std::size_t i = 1; i < poses.size(); ++i) {
            require(poses[i].timestamp >= poses[i - 1].timestamp, "handoff never goes backwards in time");
        }
        const double t = poses.front().timestamp;
        const M3 r0 = rotation(data.scans.front().time);
        const V3 g(0, 0, -9.81);
        const M3 align = Eigen::Quaterniond::FromTwoVectors(r0.transpose() * g, g).toRotationMatrix();
        require((poses.front().velocity - align * r0.transpose() * velocity(t)).norm() < 0.08,
                "solved initial velocity reaches the live filter");
        require(!lio.has_imu_time_fault(), "valid handoff does not trip the IMU guard");
        const auto &last = data.imu.back();
        lio.on_imu_callback({last.time + 0.5, last.force, last.gyro});
        require(lio.has_imu_time_fault(), "post-initialization IMU gap still latches fault");
        const auto old_count = poses.size();
        lio.handle_once();
        require(poses.size() == old_count, "fault freezes output");
        lio.reset();
        require(lio.initialization_method() == small_point_lio::initialization::Method::None,
                "reset clears the selected initialization path");
        poses.clear();
        feed(lio, data);
        lio.reset();// Discard any in-flight solution from the previous epoch.
        poses.clear();
        lio.handle_once();
        require(poses.empty(), "reset discards a pending worker result");
        feed(lio, data);
        finish_initializing(lio, poses);
        require(!poses.empty() && !lio.has_imu_time_fault(), "explicit reset can initialize a new moving window");
    }
    {
        auto opts = frontend_options(false);
        auto parameters = opts.parameter_overrides();
        parameters.erase(std::remove_if(parameters.begin(), parameters.end(),
                                        [](const auto &p) { return p.get_name() == "dynamic_init_en"; }),
                         parameters.end());
        opts.parameter_overrides(parameters);
        rclcpp::Node node("test_default_static_initialization", opts);
        SmallPointLio lio(node);
        int outputs = 0;
        lio.set_pose_history_callback([&](const auto &) { ++outputs; });
        feed(lio, make_data(true));
        require(outputs == 0, "default path still waits for a stationary interval");
        lio.reset();
        feed(lio, make_data(false));
        require(outputs > 0, "default stationary startup still succeeds");
    }
    {
        rclcpp::Node node("test_automatic_stationary_initialization", frontend_options(true));
        SmallPointLio lio(node);
        std::vector<common::Odometry> poses;
        lio.set_pose_history_callback([&](const auto &pose) { poses.push_back(pose); });
        feed(lio, make_data(false));
        finish_initializing(lio, poses);
        require(!poses.empty(), "automatic mode also initializes when stationary");
        require(lio.initialization_method() == small_point_lio::initialization::Method::Static,
                "stationary input selects the conventional stationary state model");
        require(!small_point_lio::SmallPointLioTestAccess::pending(lio), "stationary handoff exits initialization");
    }
    rclcpp::shutdown();
    std::cout << "All frontend initialization checks passed\n";
}
