/**
 * This file is part of Small Point-LIO, an advanced Point-LIO algorithm implementation.
 * Copyright (C) 2025  Yingjie Huang
 * Licensed under the MIT License. See License.txt in the project root for license information.
 */

#include "parameters.h"

namespace small_point_lio {

    void Parameters::read_parameters(rclcpp::Node &node) {
        // 点云过滤
        point_filter_num = static_cast<int>(node.declare_parameter<long>("point_filter_num"));
        auto min_distance = node.declare_parameter<float>("min_distance");
        auto max_distance = node.declare_parameter<float>("max_distance");
        min_distance_squared = min_distance * min_distance;
        max_distance_squared = max_distance * max_distance;
        space_downsample = node.declare_parameter<bool>("space_downsample");
        space_downsample_leaf_size = node.declare_parameter<float>("space_downsample_leaf_size");

        // IMU处理
        std::vector<double> gravity_temp = node.declare_parameter<std::vector<double>>("gravity");
        gravity << gravity_temp[0], gravity_temp[1], gravity_temp[2];
        check_satu = node.declare_parameter<bool>("check_satu");
        fix_gravity_direction = node.declare_parameter<bool>("fix_gravity_direction");
        satu_acc = node.declare_parameter<double>("satu_acc") * 0.99;
        satu_gyro = node.declare_parameter<double>("satu_gyro") * 0.99;
        acc_norm = node.declare_parameter<double>("acc_norm");
        max_imu_sample_gap = node.declare_parameter<double>("max_imu_sample_gap", 0.25);
        if (!std::isfinite(max_imu_sample_gap) || max_imu_sample_gap <= 0.0) {
            throw std::invalid_argument("max_imu_sample_gap must be finite and positive");
        }
        dynamic_init_en = node.declare_parameter<bool>("dynamic_init_en", false);
        dynamic_init_duration = node.declare_parameter<double>("dynamic_init_duration", 2.0);
        if (!std::isfinite(dynamic_init_duration) || dynamic_init_duration < 1.0 || dynamic_init_duration > 5.0) {
            throw std::invalid_argument("dynamic_init_duration must be within [1, 5] seconds");
        }
        if (dynamic_init_en && (!gravity.allFinite() || gravity.norm() < 1e-3 ||
                               !std::isfinite(acc_norm) || acc_norm <= 0.0)) {
            throw std::invalid_argument("dynamic initialization requires finite gravity and positive acc_norm");
        }

        // 地图
        map_resolution = node.declare_parameter<float>("map_resolution");
        map_backend = node.declare_parameter<std::string>("map_backend", "ivox");
        moment_map_resolution = node.declare_parameter<double>("moment_map_resolution", 0.5);
        if ((map_backend != "ivox" && map_backend != "moment") ||
            !std::isfinite(moment_map_resolution) || moment_map_resolution < 0.05 || moment_map_resolution > 5.0) {
            throw std::invalid_argument("map_backend must be ivox or moment; moment_map_resolution must be in [0.05, 5] m");
        }
        init_map_size = static_cast<size_t>(node.declare_parameter<long>("init_map_size"));
        defer_map_insertion_by_scan = node.declare_parameter<bool>(
                "defer_map_insertion_by_scan", false);
        planar_constraint_en = node.declare_parameter<bool>("planar_constraint_en", false);
        planar_z = node.declare_parameter<double>("planar_z", 0.0);
        planar_preserve_initial_tilt = node.declare_parameter<bool>(
                "planar_preserve_initial_tilt", true);
        if (dynamic_init_en && planar_constraint_en && !planar_preserve_initial_tilt) {
            throw std::invalid_argument(
                    "dynamic_init_en requires planar_constraint_en=false when planar_preserve_initial_tilt=false; "
                    "hard level projection is not part of the batch initialization model");
        }

        // 雷达与IMU相对位姿
        extrinsic_est_en = node.declare_parameter<bool>("extrinsic_est_en");
        std::vector<double> extrinsic_T_temp = node.declare_parameter<std::vector<double>>("extrinsic_T");
        extrinsic_T << extrinsic_T_temp[0], extrinsic_T_temp[1], extrinsic_T_temp[2];
        std::vector<double> extrinsic_R_temp = node.declare_parameter<std::vector<double>>("extrinsic_R");
        extrinsic_R << extrinsic_R_temp[0], extrinsic_R_temp[1], extrinsic_R_temp[2],
                extrinsic_R_temp[3], extrinsic_R_temp[4], extrinsic_R_temp[5],
                extrinsic_R_temp[6], extrinsic_R_temp[7], extrinsic_R_temp[8];

        // 滤波器参数
        laser_point_cov = node.declare_parameter<double>("laser_point_cov");
        if (map_backend == "moment" && (extrinsic_est_en || !std::isfinite(laser_point_cov) || laser_point_cov <= 0.0)) {
            throw std::invalid_argument("moment map requires fixed extrinsics and finite positive laser_point_cov");
        }
        imu_meas_acc_cov = node.declare_parameter<double>("imu_meas_acc_cov");
        imu_meas_omg_cov = node.declare_parameter<double>("imu_meas_omg_cov");
        if (dynamic_init_en && (!std::isfinite(imu_meas_acc_cov) || imu_meas_acc_cov <= 0 ||
                               !std::isfinite(imu_meas_omg_cov) || imu_meas_omg_cov <= 0 ||
                               !std::isfinite(satu_acc) || satu_acc <= 0 ||
                               !std::isfinite(satu_gyro) || satu_gyro <= 0)) {
            throw std::invalid_argument("dynamic initialization requires positive IMU covariance and saturation limits");
        }
        velocity_cov = node.declare_parameter<double>("velocity_cov");
        acceleration_cov = node.declare_parameter<double>("acceleration_cov");
        omg_cov = node.declare_parameter<double>("omg_cov");
        ba_cov = node.declare_parameter<double>("ba_cov");
        bg_cov = node.declare_parameter<double>("bg_cov");
        plane_threshold = node.declare_parameter<double>("plane_threshold");
        match_sqaured = node.declare_parameter<double>("match_sqaured");

        motion_diagnostics_en = node.declare_parameter<bool>("motion_diagnostics_en", false);

        // 数据发布
        publish_odometry_without_downsample = node.declare_parameter<bool>("publish_odometry_without_downsample");
        deskew_pose_history_duration = node.declare_parameter<double>(
                "deskew_pose_history_duration", 5.0);
        if (!std::isfinite(deskew_pose_history_duration) || deskew_pose_history_duration <= 0.0) {
            throw std::invalid_argument("deskew_pose_history_duration must be finite and positive");
        }
    }

}// namespace small_point_lio
