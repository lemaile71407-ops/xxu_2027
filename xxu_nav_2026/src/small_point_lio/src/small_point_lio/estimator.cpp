/**
 * This file is part of Small Point-LIO, an advanced Point-LIO algorithm implementation.
 * Copyright (C) 2025  Yingjie Huang
 * Licensed under the MIT License. See License.txt in the project root for license information.
 */

#include "estimator.h"

namespace small_point_lio {

    constexpr int NUM_MATCH_POINTS = 5;

    Estimator::Estimator() {// NOLINT(cppcoreguidelines-pro-type-member-init)
        kf.init(
                [this](auto &&s, auto &&measurement_result) {
                    return h_point(s, measurement_result);
                },
                [this](auto &&s, auto &&measurement_result) {
                    return h_imu(s, measurement_result);
                });
    }

    void Estimator::reset() {
        ivox.reset();
        moment_map.reset();
        if (parameters->map_backend == "moment") {
            moment_map = std::make_unique<MomentMap>(parameters->moment_map_resolution, 1000000);
        } else {
            ivox = std::make_shared<SmallIVox>(parameters->map_resolution, 1000000);
        }
        kf.P = Eigen::Matrix<state::value_type, state::DIM, state::DIM>::Identity() * 0.01;
        kf.P.block<3, 3>(state::gravity_index, state::gravity_index).diagonal().fill(0.0001);
        kf.P.block<3, 3>(state::bg_index, state::bg_index).diagonal().fill(0.001);
        kf.P.block<3, 3>(state::ba_index, state::ba_index).diagonal().fill(0.001);
    }

    MomentMap::Mat6 Estimator::map_pose_covariance() const {
        // Small Point-LIO: [world position, body right rotation].
        // Upstream: [world left translation, world left rotation].
        // delta_t_left = delta_p + [p]x R delta_theta_right.
        MomentMap::Mat6 transform = MomentMap::Mat6::Zero();
        transform.topLeftCorner<3, 3>().setIdentity();
        transform.topRightCorner<3, 3>() = hat<state::value_type>(kf.x.position) * kf.x.rotation;
        transform.bottomRightCorner<3, 3>() = kf.x.rotation;
        return transform * kf.P.topLeftCorner<6, 6>() * transform.transpose();
    }

    void Estimator::add_map_point(const Eigen::Vector3f &world, bool defer) {
        if (moment_map) {
            moment_map->insert(world.cast<double>(), Eigen::Matrix3d::Identity() * parameters->laser_point_cov,
                               map_pose_covariance(), defer);
        } else if (!defer) {
            ivox->add_point(world);
        }
    }

    [[nodiscard]] Eigen::Matrix<state::value_type, state::DIM, state::DIM> Estimator::process_noise_cov() const {
        Eigen::Matrix<state::value_type, state::DIM, state::DIM> cov = Eigen::Matrix<state::value_type, state::DIM, state::DIM>::Zero();
        cov.block<3, 3>(state::velocity_index, state::velocity_index).diagonal().fill(static_cast<state::value_type>(parameters->velocity_cov));
        cov.block<3, 3>(state::omg_index, state::omg_index).diagonal().fill(static_cast<state::value_type>(parameters->omg_cov));
        cov.block<3, 3>(state::acceleration_index, state::acceleration_index).diagonal().fill(static_cast<state::value_type>(parameters->acceleration_cov));
        cov.block<3, 3>(state::bg_index, state::bg_index).diagonal().fill(static_cast<state::value_type>(parameters->bg_cov));
        cov.block<3, 3>(state::ba_index, state::ba_index).diagonal().fill(static_cast<state::value_type>(parameters->ba_cov));
        return cov;
    }

    void Estimator::h_point(const state &s, point_measurement_result &measurement_result) {
        measurement_result.valid = false;
        if (parameters->motion_diagnostics_en) { ++diagnostics.attempted; }
        // get closest point
        Eigen::Matrix<state::value_type, 3, 1> point_imu_frame;
        if (parameters->extrinsic_est_en) {
            point_imu_frame = kf.x.offset_R_L_I * point_lidar_frame.cast<state::value_type>() + kf.x.offset_T_L_I;
        } else {
            point_imu_frame = Lidar_R_wrt_IMU * point_lidar_frame.cast<state::value_type>() + Lidar_T_wrt_IMU;
        }
        point_odom_frame = (kf.x.rotation * point_imu_frame + kf.x.position).cast<float>();
        Eigen::Vector3f normal;
        double point_distanace = 0.0, map_variance = 0.0;
        if (moment_map) {
            Eigen::Matrix<double, 3, 6> J;
            J.leftCols<3>().setIdentity();
            J.rightCols<3>() = -s.rotation * hat<state::value_type>(point_imu_frame);
            const Eigen::Matrix3d query_covariance = J * kf.P.topLeftCorner<6, 6>() * J.transpose() +
                    Eigen::Matrix3d::Identity() * parameters->laser_point_cov;
            const auto match = moment_map->match(point_odom_frame.cast<double>(), query_covariance);
            if (!match || !std::isfinite(match->variance) || match->variance < 0.0) {
                if (parameters->motion_diagnostics_en) { ++diagnostics.no_neighbors; }
                return;
            }
            normal = match->normal.cast<float>();
            point_distanace = match->residual;
            map_variance = match->variance;
        } else {
            ivox->get_closest_point(point_odom_frame, nearest_points, NUM_MATCH_POINTS);
            if (nearest_points.size() != NUM_MATCH_POINTS) {
                if (parameters->motion_diagnostics_en) { ++diagnostics.no_neighbors; }
                return;
            }
            // estimate plane
            Eigen::Vector3f centroid = Eigen::Vector3f::Zero();
            for (const auto &p: nearest_points) {
                centroid.noalias() += p;
            }
            centroid /= static_cast<float>(nearest_points.size());
            Eigen::Matrix3f covariance = Eigen::Matrix3f::Zero();
            for (const auto &p: nearest_points) {
                Eigen::Vector3f centered = p - centroid;
                covariance.noalias() += centered * centered.transpose();
            }
            covariance /= static_cast<float>(nearest_points.size() - 1);
            Eigen::SelfAdjointEigenSolver<Eigen::Matrix3f> solver(covariance);
            normal = solver.eigenvectors().col(0);
            float d = -normal.dot(centroid);
            for (int j = 0; j < NUM_MATCH_POINTS; j++) {
                float point_distanace = std::abs(normal.dot(nearest_points[j]) + d);
                if (point_distanace > parameters->plane_threshold) {
                    if (parameters->motion_diagnostics_en) { ++diagnostics.nonplanar; }
                    return;
                }
            }
            point_distanace = normal.dot(point_odom_frame) + d;
        }
        if (point_lidar_frame.norm() <= parameters->match_sqaured * point_distanace * point_distanace) {
            if (parameters->motion_diagnostics_en) { ++diagnostics.residual_rejected; }
            return;
        }
        // calculate residual and jacobian matrix
        measurement_result.laser_point_cov = static_cast<state::value_type>(parameters->laser_point_cov + map_variance);
        if (parameters->extrinsic_est_en) {
            Eigen::Matrix<state::value_type, 3, 1> normal0 = normal.cast<state::value_type>();
            Eigen::Matrix<state::value_type, 3, 1> C = s.rotation.transpose() * normal0;
            Eigen::Matrix<state::value_type, 3, 1> A, B;
            A.noalias() = point_imu_frame.cross(C);
            B.noalias() = point_lidar_frame.cast<state::value_type>().cross(s.offset_R_L_I.transpose() * C);
            measurement_result.H.segment<3>(0) = normal0.transpose();
            measurement_result.H.segment<3>(3) = A.transpose();
            measurement_result.H.segment<3>(6) = B.transpose();
            measurement_result.H.segment<3>(9) = C.transpose();
        } else {
            Eigen::Matrix<state::value_type, 3, 1> normal0 = normal.cast<state::value_type>();
            Eigen::Matrix<state::value_type, 3, 1> A;
            A.noalias() = point_imu_frame.cross(s.rotation.transpose() * normal0);
            measurement_result.H.setZero();
            measurement_result.H.segment<3>(0) = normal0.transpose();
            measurement_result.H.segment<3>(3) = A.transpose();
        }
        measurement_result.z = -point_distanace;
        measurement_result.valid = true;
        if (parameters->motion_diagnostics_en) {
            ++diagnostics.accepted;
            diagnostics.residual_squared_sum += static_cast<double>(point_distanace) * point_distanace;
            const Eigen::Vector2d n = normal.head<2>().cast<double>();
            diagnostics.normal_xy_sum.noalias() += n * n.transpose();
        }
    }

    void Estimator::h_imu(const state &s, imu_measurement_result &measurement_result) {
        std::memset(measurement_result.satu_check, false, 6);
        measurement_result.z.segment<3>(0) = angular_velocity - s.omg - s.bg;
        measurement_result.z.segment<3>(3) = linear_acceleration * imu_acceleration_scale - s.acceleration - s.ba;
        measurement_result.imu_meas_omg_cov = static_cast<state::value_type>(parameters->imu_meas_omg_cov);
        measurement_result.imu_meas_acc_cov = static_cast<state::value_type>(parameters->imu_meas_acc_cov);
        if (parameters->check_satu) {
            for (int i = 0; i < 3; i++) {
                if (std::abs(angular_velocity(i)) >= parameters->satu_gyro) {
                    measurement_result.satu_check[i] = true;
                    measurement_result.z(i) = 0.0;
                }
                if (std::abs(linear_acceleration(i)) >= parameters->satu_acc) {
                    measurement_result.satu_check[i + 3] = true;
                    measurement_result.z(i + 3) = 0.0;
                }
            }
        }
    }

}// namespace small_point_lio
