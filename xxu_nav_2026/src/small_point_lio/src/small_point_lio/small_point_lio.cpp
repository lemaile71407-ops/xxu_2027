/**
 * This file is part of Small Point-LIO, an advanced Point-LIO algorithm implementation.
 * Copyright (C) 2025  Yingjie Huang
 * Licensed under the MIT License. See License.txt in the project root for license information.
 */

#include "small_point_lio.h"
#include "dynamic_initializer.h"

namespace small_point_lio {

    SmallPointLio::SmallPointLio(rclcpp::Node &node) : logger(node.get_logger()) {
        // init param
        parameters.read_parameters(node);
        imu_time_guard = ImuTimeGuard(parameters.max_imu_sample_gap);
        preprocess.parameters = &parameters;
        estimator.parameters = &parameters;
        estimator.Lidar_T_wrt_IMU = parameters.extrinsic_T.cast<state::value_type>();
        estimator.Lidar_R_wrt_IMU = parameters.extrinsic_R.cast<state::value_type>();
        if (parameters.extrinsic_est_en) {
            estimator.kf.x.offset_T_L_I = parameters.extrinsic_T.cast<state::value_type>();
            estimator.kf.x.offset_R_L_I = parameters.extrinsic_R.cast<state::value_type>();
        }
        Q = estimator.process_noise_cov();
        estimator.imu_acceleration_scale = parameters.gravity.norm() / parameters.acc_norm;

        // init data
        reset();
    }

    void SmallPointLio::reset() {
        preprocess.reset();
        estimator.reset();
        estimator.diagnostics = {};
        is_init = false;
        last_dynamic_init_attempt = -std::numeric_limits<double>::infinity();
        last_static_init_attempt = -std::numeric_limits<double>::infinity();
        selected_initialization = initialization::Method::None;
        // The worker owns a snapshot only. Discard its result on reset and wait
        // for its bounded solve before destroying its future.
        dynamic_init_future = {};
        imu_time_guard.reset();
        active_map_scan_id = 0;
        pending_scan_map_points.clear();
        pose_history.clear();
        planar_reference_initialized = false;
        planar_tilt_rotation.setIdentity();
    }

    void SmallPointLio::on_point_cloud_callback(const std::vector<common::Point> &pointcloud) {
        if (has_imu_time_fault()) { return; }
        preprocess.on_point_cloud_callback(pointcloud);
    }

    void SmallPointLio::on_imu_callback(const common::ImuMsg &imu_msg) {
        if (has_imu_time_fault()) { return; }
        if (!std::isfinite(imu_msg.timestamp) || !imu_msg.angular_velocity.allFinite() ||
            !imu_msg.linear_acceleration.allFinite()) {
            RCLCPP_WARN_ONCE(logger, "Dropping non-finite IMU measurement");
            return;
        }
        if (is_init) {
            const auto result = imu_time_guard.observe(imu_msg.timestamp);
            if (result == ImuTimeGuard::Result::Stale) { return; }
            if (result == ImuTimeGuard::Result::Fault) {
                RCLCPP_ERROR(logger,
                        "IMU_TIME_FAULT gap=%.6f s last=%.6f next=%.6f: "
                        "stopping odometry and deskewed clouds; restart localization and reinitialize map pose",
                        imu_time_guard.gap(), imu_time_guard.last_timestamp(), imu_msg.timestamp);
                preprocess.reset();
                pending_scan_map_points.clear();
                pose_history.clear();
                return;
            }
        }
        preprocess.on_imu_callback(imu_msg);
    }

    void SmallPointLio::handle_once() {
        if (has_imu_time_fault()) { return; }
        // we need to init small point lio
        if (!is_init) {
            if (parameters.dynamic_init_en) {
                try_dynamic_initialize();
                return;
            }
            // Static startup must not keep an entire moving trajectory. Only
            // clouds covered by the latest calibration window may seed a map.
            auto &points = preprocess.point_deque;
            auto &dense = preprocess.dense_point_deque;
            auto &imus = preprocess.imu_deque;
            constexpr std::size_t init_imu_samples = 200;
            while (imus.size() > init_imu_samples) { imus.pop_front(); }
            const double latest = std::max(points.empty() ? -std::numeric_limits<double>::infinity() : points.back().timestamp,
                                           imus.empty() ? -std::numeric_limits<double>::infinity() : imus.back().timestamp);
            double cutoff = latest - 5.0;
            if (parameters.fix_gravity_direction && imus.size() == init_imu_samples) {
                cutoff = std::max(cutoff, imus.front().timestamp);
            }
            while (!points.empty() && (points.front().timestamp < cutoff || points.size() > 100000)) { points.pop_front(); }
            while (!dense.empty() && (dense.front().timestamp < cutoff || dense.size() > 300000)) { dense.pop_front(); }
            if (points.empty()) { return; }
            const double frontier = imus.empty() ? points.back().timestamp : std::min(points.back().timestamp, imus.back().timestamp);
            const auto map_end = std::upper_bound(points.begin(), points.end(), frontier,
                    [](double time, const common::Point &point) { return time < point.timestamp; });
            if (static_cast<std::size_t>(std::distance(points.begin(), map_end)) < parameters.init_map_size) { return; }
            if ((!preprocess.point_deque.empty() || !preprocess.imu_deque.empty()) &&
                preprocess.point_deque.size() >= parameters.init_map_size &&
                (!parameters.fix_gravity_direction || preprocess.imu_deque.size() >= 200)) {
                // Fix gravity direction only from a recent stationary window.
                // The robot can bounce while being spawned in Gazebo, and a
                // gimbal-mounted IMU can later contain large angular/centripetal
                // motion. Neither is a valid gravity calibration interval.
                if (parameters.fix_gravity_direction) {
                    const auto imu_begin = preprocess.imu_deque.end() - init_imu_samples;
                    for (auto it = std::next(imu_begin); it != imus.end(); ++it) {
                        const double gap = it->timestamp - std::prev(it)->timestamp;
                        if (gap <= 0.0 || gap > parameters.max_imu_sample_gap) { return; }
                    }
                    Eigen::Matrix<state::value_type, 3, 1> avg_acc = Eigen::Matrix<state::value_type, 3, 1>::Zero();
                    for (auto it = imu_begin; it != preprocess.imu_deque.end(); ++it) {
                        const auto &imu_msg = *it;
                        avg_acc += imu_msg.linear_acceleration.cast<state::value_type>();
                    }
                    avg_acc /= static_cast<state::value_type>(init_imu_samples);

                    state::value_type max_gyro_norm = 0.0;
                    state::value_type acc_squared_error = 0.0;
                    for (auto it = imu_begin; it != preprocess.imu_deque.end(); ++it) {
                        const auto &imu_msg = *it;
                        max_gyro_norm = std::max(
                                max_gyro_norm,
                                imu_msg.angular_velocity.cast<state::value_type>().norm());
                        acc_squared_error +=
                                (imu_msg.linear_acceleration.cast<state::value_type>() - avg_acc).squaredNorm();
                    }
                    const auto acc_rms = std::sqrt(
                            acc_squared_error / static_cast<state::value_type>(init_imu_samples));
                    if (max_gyro_norm > 0.05 || acc_rms > 0.15) {
                        return;
                    }

                    Eigen::Matrix<state::value_type, 3, 1> body_up = avg_acc.normalized();
                    Eigen::Matrix<state::value_type, 3, 1> world_up = -parameters.gravity.cast<state::value_type>().normalized();
                    Eigen::Matrix<state::value_type, 3, 1> axis = body_up.cross(world_up);
                    state::value_type axis_norm = axis.norm();
                    if (axis_norm > 0) {
                        axis /= axis_norm;
                        state::value_type angle = std::acos(body_up.dot(world_up));
                        estimator.kf.x.rotation = Eigen::AngleAxis<state::value_type>(angle, axis).toRotationMatrix();
                    }
                    estimator.kf.x.gravity = parameters.gravity.cast<state::value_type>();
                } else {
                    estimator.kf.x.gravity = parameters.gravity.cast<state::value_type>();
                }
                estimator.kf.x.acceleration = -estimator.kf.x.rotation.transpose() * estimator.kf.x.gravity;
                apply_planar_constraint();

                // init map: transform sparse measurement points to the LIO world frame.
                for (auto it = points.begin(); it != map_end; ++it) {
                    const auto &point = *it;
                    Eigen::Matrix<state::value_type, 3, 1> point_imu_frame;
                    if (parameters.extrinsic_est_en) {
                        point_imu_frame = estimator.kf.x.offset_R_L_I * point.position.cast<state::value_type>() + estimator.kf.x.offset_T_L_I;
                    } else {
                        point_imu_frame = estimator.Lidar_R_wrt_IMU * point.position.cast<state::value_type>() + estimator.Lidar_T_wrt_IMU;
                    }
                    estimator.add_map_point((estimator.kf.x.rotation * point_imu_frame + estimator.kf.x.position).cast<float>(), bool(estimator.moment_map));
                }
                if (estimator.moment_map) { estimator.moment_map->flush(); }
                // init time
                time_current = frontier;
                estimator.kf.init_timestamp(time_current);
                // Startup may wait for gravity calibration. Start gap checks
                // from the newest IMU only after the estimator is initialized.
                imu_time_guard.reset(preprocess.imu_deque.empty()
                        ? std::numeric_limits<double>::quiet_NaN()
                        : preprocess.imu_deque.back().timestamp);
                estimator.diagnostics = {};
                estimator.diagnostics.start = time_current;
                estimator.diagnostics.position_start = estimator.kf.x.position;
                // clear data
                while (!points.empty() && points.front().timestamp <= time_current) { points.pop_front(); }
                while (!dense.empty() && dense.front().timestamp <= time_current) { dense.pop_front(); }
                while (!imus.empty() && imus.front().timestamp <= time_current) { imus.pop_front(); }
                is_init = true;
                selected_initialization = initialization::Method::Static;
                record_pose_history(time_current);
            }
            return;
        }

        // Dense points are deliberately absent from this loop. They are
        // navigation observations, not filter measurements: feeding them to
        // the primary filter would make its trajectory depend on display
        // point density. The deskew path consumes them only after this loop,
        // using the pose history recorded below.
        bool is_publish_odometry = !preprocess.imu_deque.empty() && !preprocess.point_deque.empty() &&
                                   preprocess.imu_deque.front().timestamp < preprocess.point_deque.back().timestamp;
        // Keep the estimator on the common sensor-time frontier.  IMU data
        // that arrives before the next lidar frame must remain queued; if we
        // consume it here, a later frame can be older than time_current and
        // its lidar updates would be discarded as late measurements.
        while (!preprocess.imu_deque.empty() && !preprocess.point_deque.empty()) {
            const common::ImuMsg &imu_msg = preprocess.imu_deque.front();
            if (!preprocess.point_deque.empty() &&
                preprocess.point_deque.front().timestamp < imu_msg.timestamp) {
                const common::Point &point_lidar_frame = preprocess.point_deque.front();
                // point update
                if (point_lidar_frame.timestamp < time_current) {
                    if (parameters.motion_diagnostics_en) { ++estimator.diagnostics.late_points; }
                    preprocess.point_deque.pop_front();
                    continue;
                }
                time_current = point_lidar_frame.timestamp;

                if (parameters.defer_map_insertion_by_scan) {
                    begin_map_scan(point_lidar_frame.scan_id);
                }

                // predict
                predict_state_with_diagnostics(time_current);

                // update
                estimator.point_lidar_frame = point_lidar_frame.position;
                if (parameters.motion_diagnostics_en) {
                    const Eigen::Vector3d p = estimator.kf.x.position;
                    const Eigen::Vector3d v = estimator.kf.x.velocity;
                    estimator.kf.update_point();
                    estimator.diagnostics.point_dp += estimator.kf.x.position - p;
                    estimator.diagnostics.point_dv += estimator.kf.x.velocity - v;
                } else {
                    estimator.kf.update_point();
                }
                apply_planar_constraint();
                record_pose_history(time_current);

                // publish odometry
                if (parameters.publish_odometry_without_downsample) {
                    publish_odometry(time_current);
                }

                // map incremental
                if (parameters.defer_map_insertion_by_scan) {
                    pending_scan_map_points.push_back(estimator.point_odom_frame);
                    estimator.add_map_point(estimator.point_odom_frame, true);
                } else {
                    estimator.add_map_point(estimator.point_odom_frame);
                }

                preprocess.point_deque.pop_front();
            } else {
                // imu update
                if (imu_msg.timestamp < time_current) {
                    if (parameters.motion_diagnostics_en) { ++estimator.diagnostics.late_imus; }
                    preprocess.imu_deque.pop_front();
                    continue;
                }
                time_current = imu_msg.timestamp;

                // predict
                predict_state_with_diagnostics(time_current);
                estimator.kf.predict_cov(time_current, Q);

                // update
                estimator.angular_velocity = imu_msg.angular_velocity.cast<state::value_type>();
                estimator.linear_acceleration = imu_msg.linear_acceleration.cast<state::value_type>();
                if (parameters.motion_diagnostics_en) {
                    const Eigen::Vector3d p = estimator.kf.x.position;
                    const Eigen::Vector3d v = estimator.kf.x.velocity;
                    ++estimator.diagnostics.imu_updates;
                    if (!estimator.kf.update_imu()) { ++estimator.diagnostics.imu_failed; }
                    estimator.diagnostics.imu_dp += estimator.kf.x.position - p;
                    estimator.diagnostics.imu_dv += estimator.kf.x.velocity - v;
                } else {
                    estimator.kf.update_imu();
                }
                apply_planar_constraint();
                record_pose_history(time_current);

                preprocess.imu_deque.pop_front();
            }
        }

        log_motion_diagnostics();

        if (is_publish_odometry) {
            if (!parameters.publish_odometry_without_downsample) {
                publish_odometry(time_current);
            }
        }
        publish_ready_deskew_scans();
    }

    void SmallPointLio::set_deskew_scan_callback(const DeskewScanCallback &callback) {
        deskew_scan_callback = callback;
    }

    void SmallPointLio::set_pose_history_callback(const PoseHistoryCallback &callback) {
        pose_history_callback = callback;
    }

    void SmallPointLio::set_odometry_callback(const std::function<void(const common::Odometry &odometry)> &odometry_callback) {
        this->odometry_callback = odometry_callback;
    }

    void SmallPointLio::begin_map_scan(std::uint64_t scan_id) {
        if (active_map_scan_id == 0) {
            active_map_scan_id = scan_id;
            return;
        }
        if (scan_id != active_map_scan_id) {
            flush_pending_map_points();
            active_map_scan_id = scan_id;
        }
    }

    void SmallPointLio::flush_pending_map_points() {
        if (estimator.moment_map) {
            estimator.moment_map->flush();
        } else {
            for (const auto &point: pending_scan_map_points) {
                estimator.add_map_point(point);
            }
        }
        pending_scan_map_points.clear();
    }

    bool SmallPointLio::try_dynamic_initialize() {
        auto &points = preprocess.point_deque;
        auto &dense = preprocess.dense_point_deque;
        auto &imus = preprocess.imu_deque;
        if (points.empty() && imus.empty()) { return false; }
        const double latest = std::max(points.empty() ? -std::numeric_limits<double>::infinity() : points.back().timestamp,
                                       imus.empty() ? -std::numeric_limits<double>::infinity() : imus.back().timestamp);
        const double cutoff = latest - parameters.dynamic_init_duration - 0.75;
        // Bound all startup queues even if a sensor stalls or no solution is
        // observable. Keep one IMU before the interval for interpolation.
        while (!points.empty() && (points.front().timestamp < cutoff || points.size() > 100000)) { points.pop_front(); }
        while (!dense.empty() && (dense.front().timestamp < cutoff || dense.size() > 300000)) { dense.pop_front(); }
        while (imus.size() > 1 && (imus[1].timestamp < cutoff || imus.size() > 20000)) { imus.pop_front(); }
        if (dynamic_init_future.valid()) {
            if (dynamic_init_future.wait_for(std::chrono::seconds(0)) != std::future_status::ready) { return false; }
            initialization::Result result;
            try {
                result = dynamic_init_future.get();
            } catch (const std::exception &error) {
                RCLCPP_ERROR(logger, "DYNAMIC_INIT_WAIT reason=worker_exception detail=%s", error.what());
                return false;
            }
            return accept_dynamic_initialization(result);
        }
        if (points.empty() || imus.size() < 10) { return false; }
        const double frontier = std::min(points.back().timestamp, imus.back().timestamp);
        struct ScanRange { std::size_t begin, end; };
        std::vector<ScanRange> ranges;
        for (std::size_t begin = 0; begin < points.size();) {
            std::size_t end = begin + 1;
            while (end < points.size() && points[end].scan_id == points[begin].scan_id) { ++end; }
            if (points[begin].timestamp >= imus.front().timestamp && points[end - 1].timestamp <= frontier && end - begin >= 40) {
                ranges.push_back({begin, end});
            }
            begin = end;
        }
        if (ranges.empty()) { return false; }
        const bool motion_ready = ranges.size() >= 5 &&
                points[ranges.back().end - 1].timestamp - points[ranges.front().end - 1].timestamp + 1e-9 >= parameters.dynamic_init_duration &&
                frontier - last_dynamic_init_attempt >= 0.5;
        // This cheap IMU gate only selects whether a lidar stationarity check
        // is worth doing. It is NEVER sufficient evidence to install a state.
        bool imu_quiet = false;
        std::vector<ScanRange> stationary_ranges;
        if (imus.size() >= 200) {
            const auto start = imus.end() - 200;
            Eigen::Vector3d mean = Eigen::Vector3d::Zero();
            double max_gyro = 0, variance = 0;
            for (auto it = start; it != imus.end(); ++it) {
                mean += it->linear_acceleration * estimator.imu_acceleration_scale;
                max_gyro = std::max(max_gyro, it->angular_velocity.norm());
            }
            mean /= 200.0;
            for (auto it = start; it != imus.end(); ++it) {
                variance += (it->linear_acceleration * estimator.imu_acceleration_scale - mean).squaredNorm();
            }
            imu_quiet = max_gyro <= 0.05 && std::sqrt(variance / 200.0) <= 0.15;
            if (imu_quiet) {
                for (const auto &range : ranges) {
                    if (points[range.begin].timestamp >= start->timestamp) { stationary_ranges.push_back(range); }
                }
            }
        }
        const bool static_ready = stationary_ranges.size() >= 3 &&
                points[stationary_ranges.back().end - 1].timestamp -
                points[stationary_ranges.front().end - 1].timestamp + 1e-9 >= 0.6 &&
                frontier - last_static_init_attempt >= 0.1;
        if (!static_ready && !motion_ready) { return false; }
        struct Input {
            std::vector<initialization::Scan> scans;
            std::vector<initialization::Imu> imu;
        };
        const auto snapshot = [&](const std::vector<ScanRange> &selected_ranges, std::size_t max_frames) {
            Input input;
            const std::size_t count = std::min(max_frames, selected_ranges.size());
            for (std::size_t i = 0; i < count; ++i) {
                const auto &range = selected_ranges[i * (selected_ranges.size() - 1) / (count - 1)];
                initialization::Scan scan;
                scan.time = points[range.end - 1].timestamp;
                const std::size_t selected = std::min(std::size_t(160), range.end - range.begin);
                for (std::size_t j = 0; j < selected; ++j) {
                    const auto &point = points[range.begin + j * (range.end - range.begin - 1) / (selected - 1)];
                    const Eigen::Vector3d body = parameters.extrinsic_R * point.position.cast<double>() + parameters.extrinsic_T;
                    scan.points.push_back({point.timestamp, body});
                }
                input.scans.push_back(std::move(scan));
            }
            auto first = std::upper_bound(imus.begin(), imus.end(), input.scans.front().points.front().time,
                    [](double time, const common::ImuMsg &imu) { return time < imu.timestamp; });
            if (first != imus.begin()) { --first; }
            auto last = std::lower_bound(first, imus.end(), input.scans.back().time,
                    [](const common::ImuMsg &imu, double time) { return imu.timestamp < time; });
            if (last != imus.end()) { ++last; }
            for (auto it = first; it != last; ++it) {
                input.imu.push_back({it->timestamp, it->angular_velocity,
                                     it->linear_acceleration * estimator.imu_acceleration_scale});
            }
            return input;
        };
        Input stationary, motion;
        if (static_ready) {
            stationary = snapshot(stationary_ranges, 5);
            last_static_init_attempt = frontier;
        }
        if (motion_ready) {
            motion = snapshot(ranges, 12);
            last_dynamic_init_attempt = frontier;
        }
        initialization::Options options;
        options.min_duration = parameters.dynamic_init_duration;
        options.max_imu_gap = parameters.max_imu_sample_gap;
        options.gravity = parameters.gravity.norm();
        options.max_gyro = parameters.satu_gyro;
        options.max_force = parameters.satu_acc * estimator.imu_acceleration_scale;
        options.gyro_measurement_variance = parameters.imu_meas_omg_cov;
        options.accel_measurement_variance = parameters.imu_meas_acc_cov;
        dynamic_init_future = std::async(std::launch::async,
                [stationary = std::move(stationary), motion = std::move(motion), options,
                 gravity = parameters.gravity]() {
#ifdef _OPENMP
                    // OpenMP's thread-count ICV is local to this worker. Keep
                    // small window products from spawning a CPU-wide team.
                    omp_set_num_threads(1);
#endif
                    initialization::Result result;
                    if (!stationary.scans.empty()) {
                        result = initialization::initialize_stationary(stationary.scans, stationary.imu, options, gravity);
                        if (result.success) { return result; }
                    }
                    if (!motion.scans.empty()) {
                        return initialization::initialize(motion.scans, motion.imu, options, gravity);
                    }
                    return result;
                });
        return false;
    }

    bool SmallPointLio::accept_dynamic_initialization(const initialization::Result &result) {
        if (!result.success) {
            RCLCPP_INFO(logger, "INIT_WAIT reason=%s plane_rms=%.5f imu_rms=%.3f information=%.3g",
                        result.reason.c_str(), result.plane_rms, result.imu_rms, result.information_ratio);
            return false;
        }
        auto &points = preprocess.point_deque;
        auto &dense = preprocess.dense_point_deque;
        auto &imus = preprocess.imu_deque;
        if (imus.empty() || imus.front().timestamp > result.time) {
            RCLCPP_WARN(logger, "DYNAMIC_INIT_WAIT reason=solution_outside_retained_window");
            return false;
        }
        ImuTimeGuard pending_guard(parameters.max_imu_sample_gap);
        pending_guard.reset(result.time);
        for (const auto &imu : imus) {
            if (imu.timestamp <= result.time) { continue; }
            if (pending_guard.observe(imu.timestamp) != ImuTimeGuard::Result::Accepted) {
                RCLCPP_WARN(logger, "DYNAMIC_INIT_WAIT reason=handoff_imu_discontinuity");
                return false;
            }
        }
        auto &x = estimator.kf.x;
        x.position = result.position;
        x.rotation = result.rotation;
        x.velocity = result.velocity;
        x.omg = result.omega;
        x.acceleration = result.force;
        x.gravity = result.gravity;
        x.bg = result.gyro_bias;
        x.ba = result.accel_bias;
        if (parameters.extrinsic_est_en) {
            x.offset_R_L_I = parameters.extrinsic_R;
            x.offset_T_L_I = parameters.extrinsic_T;
        }
        const auto extrinsic_covariance = estimator.kf.P.block<6, 6>(6, 6).eval();
        estimator.kf.P = result.covariance;
        estimator.kf.P.block<6, 6>(6, 6) = extrinsic_covariance;
        apply_planar_constraint();
        // The supported preserve-tilt constraint only shifts the origin in z.
        // Hard level projection is rejected during parameter loading because
        // it would alter the recovered attitude/gravity/velocity relationship.
        const Eigen::Vector3d map_translation = x.position - result.position;
        for (const auto &point : result.map) {
            estimator.add_map_point((point.cast<double>() + map_translation).cast<float>(), bool(estimator.moment_map));
        }
        if (estimator.moment_map) { estimator.moment_map->flush(); }
        time_current = result.time;
        estimator.kf.init_timestamp(time_current);
        estimator.diagnostics = {};
        estimator.diagnostics.start = time_current;
        estimator.diagnostics.position_start = x.position;
        // Retain measurements after the terminal knot for normal processing;
        // in particular, do not silently lose the newest IMU or following scan.
        while (!points.empty() && points.front().timestamp <= time_current) { points.pop_front(); }
        while (!dense.empty() && dense.front().timestamp <= time_current) { dense.pop_front(); }
        while (!imus.empty() && imus.front().timestamp <= time_current) { imus.pop_front(); }
        imu_time_guard = pending_guard;
        is_init = true;
        selected_initialization = result.method;
        record_pose_history(time_current);
        RCLCPP_INFO(logger, "INIT_OK path=%s frames=%zu map_points=%zu plane_rms=%.5f imu_rms=%.3f bg_norm=%.5f ba_norm=%.5f",
                    result.method == initialization::Method::Static ? "static" : "motion", result.frame_count, result.map.size(), result.plane_rms, result.imu_rms, x.bg.norm(), x.ba.norm());
        return true;
    }

    void SmallPointLio::record_pose_history(double timestamp) {
        auto odometry = make_lidar_odometry(timestamp);
        constexpr double timestamp_epsilon = 1e-9;
        if (!pose_history.empty() && timestamp < pose_history.back().timestamp - timestamp_epsilon) {
            return;
        }
        if (!pose_history.empty() && std::abs(timestamp - pose_history.back().timestamp) <= timestamp_epsilon) {
            pose_history.back() = odometry;
        } else {
            pose_history.push_back(odometry);
        }
        while (pose_history.size() > 2 &&
               pose_history.back().timestamp - pose_history.front().timestamp > parameters.deskew_pose_history_duration) {
            pose_history.pop_front();
        }
        if (pose_history_callback) {
            pose_history_callback(odometry);
        }
    }

    common::Odometry SmallPointLio::make_lidar_odometry(double timestamp) const {
        const auto &x = estimator.kf.x;
        Eigen::Matrix<state::value_type, 3, 1> lidar_T_wrt_IMU;
        Eigen::Matrix<state::value_type, 3, 3> lidar_R_wrt_IMU;
        if (parameters.extrinsic_est_en) {
            lidar_T_wrt_IMU = x.offset_T_L_I;
            lidar_R_wrt_IMU = x.offset_R_L_I;
        } else {
            lidar_T_wrt_IMU = estimator.Lidar_T_wrt_IMU;
            lidar_R_wrt_IMU = estimator.Lidar_R_wrt_IMU;
        }

        common::Odometry odometry;
        odometry.timestamp = timestamp;
        // The filter state is expressed at the IMU origin.  The public LIO
        // pose is expressed at the lidar (or compensated virtual-sensor)
        // origin, so the lever-arm terms must be applied to velocity too.
        // lidar_R_wrt_IMU maps lidar vectors into the IMU frame.
        const Eigen::Matrix<state::value_type, 3, 3> imu_from_lidar = lidar_R_wrt_IMU;
        const Eigen::Matrix<state::value_type, 3, 3> lidar_from_imu = imu_from_lidar.transpose();
        const Eigen::Vector3d omega_imu = x.omg.cast<double>();
        const Eigen::Vector3d omega_world = x.rotation.cast<double>() * omega_imu;
        const Eigen::Vector3d lidar_offset_world =
                x.rotation.cast<double>() * lidar_T_wrt_IMU.cast<double>();

        odometry.position = (x.rotation * lidar_T_wrt_IMU + x.position).cast<double>();
        odometry.orientation = Eigen::Quaterniond(
                (x.rotation * imu_from_lidar).cast<double>()).normalized();
        odometry.velocity = x.velocity.cast<double>() + omega_world.cross(lidar_offset_world);
        odometry.angular_velocity = lidar_from_imu.cast<double>() * omega_imu;
        return odometry;
    }

    void SmallPointLio::publish_ready_deskew_scans() {
        constexpr double timestamp_epsilon = 1e-9;
        while (!preprocess.dense_point_deque.empty()) {
            const auto scan_id = preprocess.dense_point_deque.front().scan_id;
            auto scan_end = preprocess.dense_point_deque.begin();
            while (scan_end != preprocess.dense_point_deque.end() && scan_end->scan_id == scan_id) {
                ++scan_end;
            }
            if (scan_end == preprocess.dense_point_deque.begin()) {
                preprocess.dense_point_deque.pop_front();
                continue;
            }

            const double scan_start_timestamp = preprocess.dense_point_deque.front().timestamp;
            const double scan_end_timestamp = std::prev(scan_end)->timestamp;
            if (pose_history.empty() ||
                pose_history.back().timestamp + timestamp_epsilon < scan_end_timestamp) {
                return;
            }
            if (pose_history.front().timestamp - timestamp_epsilon > scan_start_timestamp) {
                RCLCPP_WARN(
                        logger,
                        "Dropping scan %llu: LIO pose history no longer covers %.6f..%.6f",
                        static_cast<unsigned long long>(scan_id),
                        scan_start_timestamp,
                        scan_end_timestamp);
                preprocess.dense_point_deque.erase(preprocess.dense_point_deque.begin(), scan_end);
                continue;
            }

            std::vector<common::Point> scan;
            scan.reserve(static_cast<size_t>(std::distance(preprocess.dense_point_deque.begin(), scan_end)));
            scan.insert(scan.end(), preprocess.dense_point_deque.begin(), scan_end);
            preprocess.dense_point_deque.erase(preprocess.dense_point_deque.begin(), scan_end);
            if (deskew_scan_callback) {
                deskew_scan_callback(scan);
            }
        }
    }

    void SmallPointLio::predict_state_with_diagnostics(double timestamp) {
        if (!parameters.motion_diagnostics_en) {
            estimator.kf.predict_state(timestamp);
            return;
        }
        const Eigen::Vector3d p = estimator.kf.x.position;
        const Eigen::Vector3d v = estimator.kf.x.velocity;
        estimator.kf.predict_state(timestamp);
        estimator.diagnostics.predict_dp += estimator.kf.x.position - p;
        estimator.diagnostics.predict_dv += estimator.kf.x.velocity - v;
    }

    void SmallPointLio::log_motion_diagnostics() {
        auto &d = estimator.diagnostics;
        if (!parameters.motion_diagnostics_en || time_current - d.start < 1.0) { return; }
        const auto &x = estimator.kf.x;
        const Eigen::Vector3d net = x.position - d.position_start;
        const Eigen::Vector3d balance = net - d.predict_dp - d.point_dp - d.imu_dp - d.constraint_dp;
        Eigen::Vector2d eigenvalues = Eigen::Vector2d::Zero();
        Eigen::Matrix2d eigenvectors = Eigen::Matrix2d::Identity();
        if (d.accepted > 0) {
            const Eigen::Matrix2d mean = d.normal_xy_sum / static_cast<double>(d.accepted);
            const Eigen::SelfAdjointEigenSolver<Eigen::Matrix2d> solver(mean);
            eigenvalues = solver.eigenvalues();
            eigenvectors = solver.eigenvectors();
        }
        const auto roll = std::atan2(x.rotation(2, 1), x.rotation(2, 2));
        const auto pitch = std::asin(std::clamp(-x.rotation(2, 0), static_cast<state::value_type>(-1.0), static_cast<state::value_type>(1.0)));
        const auto yaw = std::atan2(x.rotation(1, 0), x.rotation(0, 0));
        const auto measured_specific_force = estimator.linear_acceleration * estimator.imu_acceleration_scale;
        const auto world_acceleration = x.rotation * x.acceleration + x.gravity;
        const double rms = d.accepted ? std::sqrt(d.residual_squared_sum / d.accepted) : 0.0;
        RCLCPP_INFO(logger,
                "LIO_DIAG t=%.6f dt=%.6f p_xy=[%.6f,%.6f] v_xy=[%.6f,%.6f] "
                "net_dp_xy=[%.6f,%.6f] predict_dp_xy=[%.6f,%.6f] point_dp_xy=[%.6f,%.6f] "
                "imu_dp_xy=[%.6f,%.6f] balance_norm=%.3e "
                "predict_dv_xy=[%.6f,%.6f] point_dv_xy=[%.6f,%.6f] imu_dv_xy=[%.6f,%.6f]",
                time_current, time_current - d.start, x.position.x(), x.position.y(), x.velocity.x(), x.velocity.y(),
                net.x(), net.y(), d.predict_dp.x(), d.predict_dp.y(), d.point_dp.x(), d.point_dp.y(),
                d.imu_dp.x(), d.imu_dp.y(), balance.norm(),
                d.predict_dv.x(), d.predict_dv.y(), d.point_dv.x(), d.point_dv.y(), d.imu_dv.x(), d.imu_dv.y());
        RCLCPP_INFO(logger,
                "LIO_MATCH t=%.6f attempted=%zu accepted=%zu ratio=%.4f "
                "no_neighbors=%zu nonplanar=%zu residual_rejected=%zu residual_rms=%.6f "
                "normal_xy_eigen=[%.6f,%.6f] imu_updates=%zu imu_failed=%zu late_points=%zu late_imus=%zu",
                time_current, d.attempted, d.accepted,
                d.attempted ? static_cast<double>(d.accepted) / d.attempted : 0.0,
                d.no_neighbors, d.nonplanar, d.residual_rejected, rms,
                eigenvalues.x(), eigenvalues.y(), d.imu_updates, d.imu_failed, d.late_points, d.late_imus);
        RCLCPP_INFO(logger,
                "LIO_PROP t=%.6f rpy_deg=[%.3f,%.3f,%.3f] "
                "acc_meas_body=[%.6f,%.6f,%.6f] acc_est_body=[%.6f,%.6f,%.6f] "
                "acc_world=[%.6f,%.6f,%.6f] normal_xy_weak_dir=[%.6f,%.6f] "
                "normal_xy_strong_dir=[%.6f,%.6f] normal_dirs_valid=%d",
                time_current,
                roll * 180.0 / M_PI, pitch * 180.0 / M_PI, yaw * 180.0 / M_PI,
                measured_specific_force.x(), measured_specific_force.y(), measured_specific_force.z(),
                x.acceleration.x(), x.acceleration.y(), x.acceleration.z(),
                world_acceleration.x(), world_acceleration.y(), world_acceleration.z(),
                eigenvectors(0, 0), eigenvectors(1, 0),
                eigenvectors(0, 1), eigenvectors(1, 1),
                d.accepted > 0 ? 1 : 0);
        d = {};
        d.start = time_current;
        d.position_start = x.position;
    }

    void SmallPointLio::apply_planar_constraint() {
        if (!parameters.planar_constraint_en) {
            return;
        }

        auto &x = estimator.kf.x;
        const auto yaw = std::atan2(x.rotation(1, 0), x.rotation(0, 0));
        const auto cos_yaw = std::cos(yaw);
        const auto sin_yaw = std::sin(yaw);
        Eigen::Matrix<state::value_type, 3, 3> yaw_rotation;
        yaw_rotation << cos_yaw, -sin_yaw, 0.0,
                        sin_yaw,  cos_yaw, 0.0,
                        0.0,      0.0,     1.0;

        if (parameters.motion_diagnostics_en) {
            estimator.diagnostics.constraint_dp.z() += parameters.planar_z - x.position.z();
        }
        x.position.z() = static_cast<state::value_type>(parameters.planar_z);
        x.velocity.z() = 0.0;
        if (parameters.planar_preserve_initial_tilt) {
            // Legacy raw-sensor mode keeps the fixed mounting tilt.
            if (!planar_reference_initialized) {
                planar_tilt_rotation = yaw_rotation.transpose() * x.rotation;
                planar_reference_initialized = true;
            }
            x.rotation = yaw_rotation * planar_tilt_rotation;
        } else {
            // The compensated virtual body frame is level and planar.
            x.rotation = yaw_rotation;
            x.omg.x() = 0.0;
            x.omg.y() = 0.0;
        }

        auto &P = estimator.kf.P;
        constexpr state::value_type constrained_cov = 1e-6;
        const std::array<int, 4> constrained_indices = {
                state::position_index + 2,
                state::rotation_index + 0,
                state::rotation_index + 1,
                state::velocity_index + 2,
        };
        for (const auto index: constrained_indices) {
            P.row(index).setZero();
            P.col(index).setZero();
            P(index, index) = constrained_cov;
        }
        if (!parameters.planar_preserve_initial_tilt) {
            for (const auto index: {
                    state::omg_index + 0,
                    state::omg_index + 1}) {
                P.row(index).setZero();
                P.col(index).setZero();
                P(index, index) = constrained_cov;
            }
        }
    }

    void SmallPointLio::publish_odometry(double timestamp) {
        if (odometry_callback) {
            odometry_callback(make_lidar_odometry(timestamp));
        }
    }

}// namespace small_point_lio
