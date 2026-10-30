#include "dynamic_initialization_fixture.h"
#include "frontend_initialization_options.h"
#include "small_point_lio/small_point_lio.h"
#include <cstdlib>
#include <iostream>
#include <thread>
using namespace small_point_lio;
using namespace initialization_test;
namespace small_point_lio {
    struct SmallPointLioTestAccess {
        static const Estimator &estimator(const SmallPointLio &lio) { return lio.estimator; }
    };
}
static void require(bool ok, const char *message) {
    if (!ok) { std::cerr << "FAIL: " << message << '\n'; std::exit(1); }
}
static rclcpp::NodeOptions options(bool dynamic) {
    auto result = frontend_options(dynamic);
    auto overrides = result.parameter_overrides();
    overrides.emplace_back("map_backend", "moment");
    overrides.emplace_back("defer_map_insertion_by_scan", true);
    overrides.emplace_back("motion_diagnostics_en", true);
    result.parameter_overrides(overrides);
    return result;
}
int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    if (!MomentMap::available()) {
        bool rejected = false;
        try { rclcpp::Node node("test_missing_moment_build", options(false)); SmallPointLio lio(node); }
        catch (const std::invalid_argument &) { rejected = true; }
        require(rejected, "disabled optional backend rejects an explicit runtime request");
        rclcpp::shutdown(); return 0;
    }
    {
        Parameters p;
        p.map_backend = "moment"; p.moment_map_resolution = .5; p.laser_point_cov = .0001;
        p.extrinsic_est_en = false; p.motion_diagnostics_en = false; p.match_sqaured = 81;
        Estimator e; e.parameters = &p; e.reset();
        e.Lidar_R_wrt_IMU.setIdentity(); e.Lidar_T_wrt_IMU.setZero();
        e.kf.x.position = V3(13, -7, 2);
        e.kf.x.rotation = Eigen::AngleAxisd(.6, V3(1,2,3).normalized()).toRotationMatrix();
        const V3 body(.2, -.4, 1.3), world = e.kf.x.rotation * body + e.kf.x.position;
        Eigen::Matrix<double,3,6> right, left;
        right.leftCols<3>().setIdentity(); right.rightCols<3>() = -e.kf.x.rotation * hat<double>(body);
        left.leftCols<3>().setIdentity(); left.rightCols<3>() = -hat<double>(world);
        require((right*e.kf.P.topLeftCorner<6,6>()*right.transpose()-left*e.map_pose_covariance()*left.transpose()).norm()<1e-10,
                "right/body to left/world pose covariance preserves point uncertainty away from origin");
        e.kf.x.position.setZero(); e.kf.x.rotation.setIdentity();
        e.kf.P.setIdentity(); e.kf.P *= 1e-6;
        for (int x=0;x<31;++x) for (int y=0;y<31;++y) {
            e.add_map_point(Eigen::Vector3f(1.01f+.04f*x,1.02f+.04f*y,-.1f), true);
        }
        require(e.moment_map->size()==0, "scan deferred supports remain invisible until flush");
        e.moment_map->flush();
        require(e.moment_map->size()>0, "flush builds actual upstream map");
        e.point_lidar_frame = Eigen::Vector3f(1.3f, 1.3f, -.105f);
        point_measurement_result measurement;
        e.h_point(e.kf.x, measurement);
        require(measurement.valid, "upstream plane provides an ESKF measurement");
        require(std::abs(measurement.H(2))>.99, "map plane normal enters the position Jacobian");
        require(measurement.laser_point_cov>=p.laser_point_cov && std::isfinite(measurement.laser_point_cov),
                "plane uncertainty is added to sensor variance without duplicating prior pose variance");
        e.reset();
        require(e.moment_map->size()==0, "reset clears map and pending supports");
    }
    for (bool dynamic : {false,true}) {
        rclcpp::Node node(dynamic?"test_motion_moment_handoff":"test_static_moment_handoff",options(dynamic));
        SmallPointLio lio(node);
        std::vector<common::Odometry> poses;
        lio.set_pose_history_callback([&](const auto &pose){poses.push_back(pose);});
        auto data=make_data(dynamic);
        // Interleaved stream, including IMU coverage of each scan end.
        std::size_t next_scan=0;
        for(const auto &imu:data.imu) {
            lio.on_imu_callback({imu.time,imu.force,imu.gyro});
            while(next_scan<data.scans.size() && data.scans[next_scan].time<=imu.time) {
                std::vector<common::Point> points;
                for(const auto &point:data.scans[next_scan].points)points.push_back({point.time,point.body.cast<float>()});
                lio.on_point_cloud_callback(points); ++next_scan;
            }
            lio.handle_once();
            if(dynamic)std::this_thread::sleep_for(std::chrono::milliseconds(1));
        }
        const auto deadline=std::chrono::steady_clock::now()+std::chrono::seconds(10);
        while(poses.empty() && std::chrono::steady_clock::now()<deadline){lio.handle_once();std::this_thread::sleep_for(std::chrono::milliseconds(5));}
        lio.handle_once();
        require(!poses.empty(), "static and motion initialization both hand off to moment backend");
        const auto &e=SmallPointLioTestAccess::estimator(lio);
        require(e.moment_map && !e.ivox && e.moment_map->size()>0, "selected backend exclusively receives initial map");
        require(!lio.has_imu_time_fault(), "backend does not alter IMU continuity");
        for(const auto &pose:poses)require(pose.position.allFinite()&&pose.velocity.allFinite(),"finite post-handoff states");
        lio.reset();
        require(SmallPointLioTestAccess::estimator(lio).moment_map->size()==0,"frontend reset replaces moment map");
    }
    rclcpp::shutdown();
    std::cout << "moment map bridge/frontend checks passed\n";
}
