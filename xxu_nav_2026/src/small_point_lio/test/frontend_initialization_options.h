#pragma once

#include <rclcpp/rclcpp.hpp>

namespace initialization_test {
    inline rclcpp::NodeOptions frontend_options(bool dynamic) {
        rclcpp::NodeOptions opts;
        opts.start_parameter_services(false).start_parameter_event_publisher(false).enable_rosout(false);
        opts.parameter_overrides({{"point_filter_num", 1}, {"min_distance", 0.1}, {"max_distance", 100.0}, {"space_downsample", false}, {"space_downsample_leaf_size", 0.2}, {"gravity", std::vector<double>{0, 0, -9.81}}, {"check_satu", true}, {"fix_gravity_direction", true}, {"satu_acc", 100.0}, {"satu_gyro", 35.0}, {"acc_norm", 9.81}, {"map_resolution", 0.2}, {"init_map_size", 20}, {"extrinsic_est_en", false}, {"extrinsic_T", std::vector<double>{0, 0, 0}}, {"extrinsic_R", std::vector<double>{1, 0, 0, 0, 1, 0, 0, 0, 1}}, {"laser_point_cov", 0.01}, {"imu_meas_acc_cov", 0.01}, {"imu_meas_omg_cov", 0.01}, {"velocity_cov", 20.0}, {"acceleration_cov", 500.0}, {"omg_cov", 1000.0}, {"ba_cov", 0.0001}, {"bg_cov", 0.0001}, {"plane_threshold", 0.1}, {"match_sqaured", 81.0}, {"publish_odometry_without_downsample", false}, {"dynamic_init_en", dynamic}});
        return opts;
    }
}// namespace initialization_test
