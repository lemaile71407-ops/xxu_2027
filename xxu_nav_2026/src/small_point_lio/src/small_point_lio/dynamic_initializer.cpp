#include "dynamic_initializer.h"

#include <Eigen/Cholesky>
#include <Eigen/Eigenvalues>
#include <algorithm>
#include <array>
#include <cmath>
#include <limits>

namespace small_point_lio::initialization {
    namespace {
        using V3 = Eigen::Vector3d;
        using M3 = Eigen::Matrix3d;

        M3 skew(const V3 &v) {
            M3 m;
            m << 0, -v.z(), v.y(), v.z(), 0, -v.x(), -v.y(), v.x(), 0;
            return m;
        }
        M3 exp(const V3 &v) {
            const double a = v.norm();
            return a < 1e-12 ? M3(M3::Identity() + skew(v))
                             : M3(Eigen::AngleAxisd(a, v / a).toRotationMatrix());
        }
        V3 log(const M3 &r) {
            const Eigen::AngleAxisd a(r);
            return a.axis() * a.angle();
        }

        struct Knot {
            M3 r = M3::Identity();
            V3 p = V3::Zero(), v = V3::Zero();
        };
        struct Window {
            std::vector<Knot> knots;
            V3 gravity = V3::Zero(), bg = V3::Zero(), ba = V3::Zero();
        };

        Imu sample(const std::vector<Imu> &imu, double time) {
            auto hi = std::lower_bound(imu.begin(), imu.end(), time,
                                       [](const Imu &a, double t) { return a.time < t; });
            if (hi == imu.begin()) { return *hi; }
            if (hi == imu.end()) { return imu.back(); }
            const auto &lo = *std::prev(hi);
            const double u = (time - lo.time) / (hi->time - lo.time);
            return {time, lo.gyro + u * (hi->gyro - lo.gyro), lo.force + u * (hi->force - lo.force)};
        }

        // Midpoint integration in either time direction. With identity initial pose and
        // zero gravity this is the IMU preintegration (Delta R, Delta p, Delta v).
        Knot integrate(Knot x, const std::vector<Imu> &imu, double from, double to,
                       const V3 &g, const V3 &bg, const V3 &ba) {
            double t = from;
            const bool forward = to >= from;
            auto next = forward
                                ? std::upper_bound(imu.begin(), imu.end(), from,
                                                   [](double a, const Imu &b) { return a < b.time; })
                                : std::lower_bound(imu.begin(), imu.end(), from,
                                                   [](const Imu &a, double b) { return a.time < b; });
            while (std::abs(to - t) > 1e-10) {
                double end = to;
                if (forward && next != imu.end()) { end = std::min(to, next->time); }
                if (!forward && next != imu.begin()) { end = std::max(to, std::prev(next)->time); }
                const double dt = end - t;
                const auto a = sample(imu, t), b = sample(imu, end);
                const V3 w = 0.5 * (a.gyro + b.gyro) - bg;
                const V3 f = 0.5 * (a.force + b.force) - ba;
                const V3 acceleration = x.r * exp(w * (0.5 * dt)) * f + g;
                x.p += x.v * dt + 0.5 * acceleration * dt * dt;
                x.v += acceleration * dt;
                x.r *= exp(w * dt);
                t = end;
                if (forward && next != imu.end()) { ++next; }
                if (!forward && next != imu.begin()) { --next; }
            }
            return x;
        }

        using Clouds = std::vector<std::vector<V3>>;
        Clouds deskew(const Window &w, const std::vector<Scan> &scans, const std::vector<Imu> &imu) {
            Clouds clouds(scans.size());
            for (std::size_t i = 0; i < scans.size(); ++i) {
                for (const auto &point: scans[i].points) {
                    const auto pose = integrate(w.knots[i], imu, scans[i].time, point.time,
                                                w.gravity, w.bg, w.ba);
                    clouds[i].push_back(w.knots[i].r.transpose() *
                                        (pose.r * point.body + pose.p - w.knots[i].p));
                }
            }
            return clouds;
        }

        struct Plane {
            V3 n, center;
        };
        bool plane_at(const std::vector<V3> &cloud, const V3 &q, const Options &o, Plane &plane) {
            std::array<double, 5> distances;
            distances.fill(o.match_distance * o.match_distance);
            std::array<const V3 *, 5> nearest{};
            for (const auto &p: cloud) {
                const double d = (q - p).squaredNorm();
                if (d >= distances.back()) { continue; }
                int k = 4;
                while (k > 0 && d < distances[k - 1]) {
                    distances[k] = distances[k - 1];
                    nearest[k] = nearest[k - 1];
                    --k;
                }
                distances[k] = d;
                nearest[k] = &p;
            }
            if (!nearest.back()) { return false; }
            plane.center.setZero();
            for (const auto *p: nearest) { plane.center += *p / 5.0; }
            M3 cov = M3::Zero();
            for (const auto *p: nearest) {
                const V3 d = *p - plane.center;
                cov.noalias() += d * d.transpose();
            }
            const Eigen::SelfAdjointEigenSolver<M3> eig(cov);
            if (eig.info() != Eigen::Success || eig.eigenvalues()[1] < 1e-5 ||
                eig.eigenvalues()[0] > 0.08 * eig.eigenvalues()[1]) { return false; }
            plane.n = eig.eigenvectors().col(0);
            for (const auto *p: nearest) {
                if (std::abs(plane.n.dot(*p - plane.center)) > 2.0 * o.plane_sigma) { return false; }
            }
            return std::abs(plane.n.dot(q - plane.center)) < 0.3;
        }

        bool coarse_align(Knot &pose, const std::vector<V3> &source,
                          const Knot &reference, const std::vector<V3> &target, const Options &o,
                          double min_information_ratio = 1e-6) {
            for (int iteration = 0; iteration < 12; ++iteration) {
                Eigen::Matrix<double, 6, 6> h = Eigen::Matrix<double, 6, 6>::Zero();
                Eigen::Matrix<double, 6, 1> b = Eigen::Matrix<double, 6, 1>::Zero();
                int count = 0;
                for (const auto &p: source) {
                    const V3 q = reference.r.transpose() * (pose.r * p + pose.p - reference.p);
                    Plane plane;
                    if (!plane_at(target, q, o, plane)) { continue; }
                    const V3 normal = reference.r * plane.n;
                    const double residual = plane.n.dot(q - plane.center);
                    Eigen::Matrix<double, 1, 6> j;
                    const V3 angular = skew(p) * pose.r.transpose() * normal;
                    for (int axis = 0; axis < 3; ++axis) {
                        j[axis] = angular[axis];
                        j[axis + 3] = normal[axis];
                    }
                    const double weight = std::min(1.0, 0.1 / std::max(1e-12, std::abs(residual)));
                    h.noalias() += weight * j.transpose() * j;
                    b.noalias() += weight * j.transpose() * residual;
                    ++count;
                }
                if (count < o.min_matches) { return false; }
                const Eigen::SelfAdjointEigenSolver<Eigen::Matrix<double, 6, 6>> eig(h);
                if (eig.info() != Eigen::Success ||
                    eig.eigenvalues()[0] < min_information_ratio * eig.eigenvalues()[5]) { return false; }
                Eigen::Matrix<double, 6, 1> dx = -h.ldlt().solve(b);
                if (!dx.allFinite() || dx.head<3>().norm() > 0.5 || dx.tail<3>().norm() > 1.0) { return false; }
                pose.r *= exp(dx.head<3>());
                pose.p += dx.tail<3>();
                if (dx.norm() < 1e-5) { break; }
            }
            return true;
        }

        struct Match {
            std::size_t source, target;
            V3 point, normal, center;
        };
        std::vector<Match> match(const Window &w, const Clouds &clouds, const Options &o) {
            std::vector<Match> matches;
            for (std::size_t i = 1; i < clouds.size(); ++i) {
                // Each plane belongs to an earlier frame, never the query itself.
                // Its pose remains a variable in the joint objective below.
                for (int reference = 0; reference < (i == 1 ? 1 : 2); ++reference) {
                    const std::size_t j = reference == 0 ? i - 1 : 0;
                    for (const auto &point: clouds[i]) {
                        const V3 q = w.knots[j].r.transpose() *
                                     (w.knots[i].r * point + w.knots[i].p - w.knots[j].p);
                        Plane plane;
                        if (plane_at(clouds[j], q, o, plane)) {
                            matches.push_back({i, j, point, plane.n, plane.center});
                        }
                    }
                }
            }
            return matches;
        }

        int velocity_offset(int n) { return 6 * (n - 1); }
        int shared_offset(int n) { return velocity_offset(n) + 3 * n; }
        int dimension(int n) { return shared_offset(n) + 9; }

        Window increment(Window w, const Eigen::VectorXd &dx) {
            const int n = static_cast<int>(w.knots.size());
            for (int i = 1; i < n; ++i) {
                w.knots[i].r *= exp(dx.segment<3>(6 * (i - 1)));
                w.knots[i].p += dx.segment<3>(6 * (i - 1) + 3);
            }
            for (int i = 0; i < n; ++i) { w.knots[i].v += dx.segment<3>(velocity_offset(n) + 3 * i); }
            w.gravity += dx.segment<3>(shared_offset(n));
            w.bg += dx.segment<3>(shared_offset(n) + 3);
            w.ba += dx.segment<3>(shared_offset(n) + 6);
            return w;
        }

        Eigen::VectorXd residuals(const Window &w, const std::vector<Scan> &scans,
                                  const std::vector<Imu> &imu, const std::vector<Match> &matches,
                                  const Options &o) {
            const int n = static_cast<int>(scans.size());
            Eigen::VectorXd r(matches.size() + 9 * (n - 1) + 1);
            int row = 0;
            for (const auto &m: matches) {
                const auto &a = w.knots[m.source], &b = w.knots[m.target];
                r[row++] = m.normal.dot(b.r.transpose() * (a.r * m.point + a.p - b.p) - m.center) / o.plane_sigma;
            }
            for (int i = 0; i < n - 1; ++i) {
                const double dt = scans[i + 1].time - scans[i].time;
                const auto d = integrate(Knot{}, imu, scans[i].time, scans[i + 1].time,
                                         V3::Zero(), w.bg, w.ba);
                const auto &a = w.knots[i], &b = w.knots[i + 1];
                r.segment<3>(row) = log(d.r.transpose() * a.r.transpose() * b.r) /
                                    (o.gyro_noise_density * std::sqrt(dt));
                r.segment<3>(row + 3) = (a.r.transpose() * (b.v - a.v - w.gravity * dt) - d.v) /
                                        (o.accel_noise_density * std::sqrt(dt));
                r.segment<3>(row + 6) = (a.r.transpose() *
                                                 (b.p - a.p - a.v * dt - 0.5 * w.gravity * dt * dt) -
                                         d.p) /
                                        (o.accel_noise_density * std::sqrt(dt * dt * dt / 3.0));
                row += 9;
            }
            r[row] = (w.gravity.norm() - o.gravity) / 0.01;
            return r;
        }

        Eigen::MatrixXd jacobian(const Window &w, const std::vector<Scan> &scans,
                                 const std::vector<Imu> &imu, const std::vector<Match> &matches,
                                 const Options &o, const Eigen::VectorXd &r) {
            const int dim = dimension(static_cast<int>(scans.size()));
            Eigen::MatrixXd j(r.size(), dim);
            for (int col = 0; col < dim; ++col) {
                Eigen::VectorXd dx = Eigen::VectorXd::Zero(dim);
                dx[col] = 1e-6;
                j.col(col) = (residuals(increment(w, dx), scans, imu, matches, o) - r) / dx[col];
            }
            return j;
        }

        bool optimize(Window &w, const std::vector<Scan> &scans, const std::vector<Imu> &imu,
                      const std::vector<Match> &matches, const Options &o) {
            double damping = 1e-3;
            for (int iteration = 0; iteration < 15; ++iteration) {
                const auto r = residuals(w, scans, imu, matches, o);
                const auto j = jacobian(w, scans, imu, matches, o, r);
                if (!r.allFinite() || !j.allFinite()) { return false; }
                const Eigen::MatrixXd h = j.transpose() * j;
                const Eigen::VectorXd gradient = j.transpose() * r;
                if (gradient.lpNorm<Eigen::Infinity>() < 1e-5) { return true; }
                bool accepted = false;
                for (int trial = 0; trial < 8; ++trial) {
                    Eigen::MatrixXd regularized = h;
                    regularized.diagonal().array() += damping * h.diagonal().array().max(1.0);
                    const Eigen::VectorXd dx = -regularized.ldlt().solve(gradient);
                    if (!dx.allFinite()) { return false; }
                    auto candidate = increment(w, dx);
                    const auto updated = residuals(candidate, scans, imu, matches, o);
                    if (updated.allFinite() && updated.squaredNorm() < r.squaredNorm()) {
                        w = std::move(candidate);
                        damping = std::max(1e-9, damping * 0.3);
                        accepted = true;
                        if (dx.norm() < 1e-5) { return true; }
                        break;
                    }
                    damping *= 10.0;
                }
                if (!accepted) { return gradient.norm() < 1e-3; }
            }
            return false;
        }

        bool validate(const std::vector<Scan> &scans, const std::vector<Imu> &imu,
                      const Options &o, const V3 &target, std::string &reason, std::size_t min_scans = 5) {
            if (scans.size() < min_scans || scans.size() > 16 || imu.size() < 10 || imu.size() > 20000) { return false; }
            const std::array positive = {o.min_duration, o.max_imu_gap, o.gravity, o.plane_sigma,
                                         o.gyro_noise_density, o.accel_noise_density, o.max_gyro, o.max_force,
                                         o.match_distance, o.gyro_measurement_variance, o.accel_measurement_variance};
            if (std::any_of(positive.begin(), positive.end(), [](double x) { return !std::isfinite(x) || x <= 0; }) ||
                o.min_matches < 10 || o.min_matches > 256 || !target.allFinite() || target.norm() < 1e-3) {
                reason = "invalid_options";
                return false;
            }
            for (std::size_t i = 0; i < imu.size(); ++i) {
                const auto &m = imu[i];
                if (!std::isfinite(m.time) || !m.gyro.allFinite() || !m.force.allFinite()) {
                    reason = "nonfinite_imu";
                    return false;
                }
                if (i && (m.time <= imu[i - 1].time || m.time - imu[i - 1].time > o.max_imu_gap)) {
                    reason = "imu_time_discontinuity";
                    return false;
                }
                if (m.gyro.cwiseAbs().maxCoeff() >= o.max_gyro || m.force.cwiseAbs().maxCoeff() >= o.max_force) {
                    reason = "imu_saturated";
                    return false;
                }
            }
            for (std::size_t i = 0; i < scans.size(); ++i) {
                const auto &scan = scans[i];
                if (!std::isfinite(scan.time) || scan.points.size() < static_cast<std::size_t>(o.min_matches) ||
                    scan.points.size() > 256 || scan.time > imu.back().time ||
                    (i && scan.time <= scans[i - 1].time)) {
                    reason = "invalid_scan_window";
                    return false;
                }
                for (const auto &p: scan.points) {
                    if (!std::isfinite(p.time) || !p.body.allFinite() || p.time > scan.time + 1e-9 || p.time < imu.front().time - 1e-9) {
                        reason = "unbracketed_or_invalid_point";
                        return false;
                    }
                }
            }
            if (scans.back().time - scans.front().time + 1e-9 < o.min_duration) { return false; }
            return true;
        }
    }// namespace

    Result initialize_stationary(const std::vector<Scan> &scans, const std::vector<Imu> &imu,
                                 const Options &options, const V3 &target_gravity) {
        Result out;
        out.method = Method::Static;
        out.frame_count = scans.size();
        Options o = options;
        o.min_duration = 0.6;
        if (!validate(scans, imu, o, target_gravity, out.reason, 3)) { return out; }
        V3 mean_force = V3::Zero();
        double max_gyro = 0;
        for (const auto &m: imu) {
            mean_force += m.force;
            max_gyro = std::max(max_gyro, m.gyro.norm());
        }
        mean_force /= double(imu.size());
        double variance = 0;
        for (const auto &m: imu) { variance += (m.force - mean_force).squaredNorm(); }
        if (max_gyro > 0.05 || std::sqrt(variance / imu.size()) > 0.15 || mean_force.norm() < 1e-3) {
            out.reason = "imu_not_stationary";
            return out;
        }
        std::vector<V3> reference;
        for (const auto &p: scans.front().points) { reference.push_back(p.body); }
        Knot pose;
        double squared_error = 0;
        int matches = 0;
        for (std::size_t i = 1; i < scans.size(); ++i) {
            std::vector<V3> cloud;
            for (const auto &p: scans[i].points) { cloud.push_back(p.body); }
            // Stronger geometry is required to certify absence of motion:
            // tiny normals produced by noise on one plane are not evidence.
            if (!coarse_align(pose, cloud, Knot{}, reference, o, 1e-3)) {
                out.reason = "stationarity_geometry_unobservable";
                return out;
            }
            if (pose.p.norm() > 0.02 || log(pose.r).norm() > 0.01) {
                out.reason = "lidar_motion_detected";
                return out;
            }
            for (const auto &p: cloud) {
                Plane plane;
                const V3 q = pose.r * p + pose.p;
                if (plane_at(reference, q, o, plane)) {
                    squared_error += std::pow(plane.n.dot(q - plane.center), 2);
                    ++matches;
                }
            }
        }
        out.plane_rms = matches ? std::sqrt(squared_error / matches) : INFINITY;
        if (out.plane_rms > 0.02) {
            out.reason = "stationarity_residual_limit";
            return out;
        }
        out.rotation = Eigen::Quaterniond::FromTwoVectors(mean_force, -target_gravity).toRotationMatrix();
        out.gravity = target_gravity;
        out.force = -out.rotation.transpose() * out.gravity;
        out.time = scans.back().time;
        // Same stationary-state assumptions as the legacy path: zero initial
        // velocity/omega/bias, with uncertainty retained for subsequent updates.
        out.covariance.setIdentity();
        out.covariance *= 0.01;
        out.covariance.block<3, 3>(21, 21).diagonal().setConstant(0.0001);
        out.covariance.block<3, 3>(24, 24).diagonal().setConstant(0.001);
        out.covariance.block<3, 3>(27, 27).diagonal().setConstant(0.001);
        for (const auto &scan: scans) {
            for (const auto &p: scan.points) { out.map.push_back((out.rotation * p.body).cast<float>()); }
        }
        out.success = true;
        out.reason = "stationary_initialized";
        return out;
    }

    Result initialize(const std::vector<Scan> &scans, const std::vector<Imu> &imu,
                      const Options &o, const V3 &target_gravity) {
        Result out;
        out.frame_count = scans.size();
        if (!validate(scans, imu, o, target_gravity, out.reason)) { return out; }
        const int n = static_cast<int>(scans.size());
        Window w;
        w.knots.resize(n);
        V3 mean = V3::Zero();
        int count = 0;
        for (const auto &m: imu) {
            if (m.time < scans[0].time) { continue; }
            mean += m.force;
            ++count;
            if (count == 20) { break; }
        }
        if (!count || mean.norm() < 1e-3) {
            out.reason = "invalid_gravity_seed";
            return out;
        }
        // This average is ONLY a seed; it is never installed as the final gravity.
        w.gravity = -mean.normalized() * o.gravity;
        for (int i = 1; i < n; ++i) {
            w.knots[i] = integrate(w.knots[i - 1], imu, scans[i - 1].time, scans[i].time,
                                   w.gravity, w.bg, w.ba);
            const auto clouds = deskew(w, std::vector<Scan>(scans.begin(), scans.begin() + i + 1), imu);
            if (!coarse_align(w.knots[i], clouds[i], w.knots[i - 1], clouds[i - 1], o)) {
                out.reason = "insufficient_plane_geometry";
                return out;
            }
            const double dt = scans[i].time - scans[i - 1].time;
            w.knots[i].v = (w.knots[i].p - w.knots[i - 1].p) / dt;
        }
        for (int i = 0; i < n; ++i) {
            const int lo = std::max(0, i - 1), hi = std::min(n - 1, i + 1);
            w.knots[i].v = (w.knots[hi].p - w.knots[lo].p) / (scans[hi].time - scans[lo].time);
        }

        Clouds clouds;
        std::vector<Match> matches;
        bool converged = false;
        for (int outer = 0; outer < 6; ++outer) {
            clouds = deskew(w, scans, imu);
            matches = match(w, clouds, o);
            std::vector<int> counts(n, 0);
            for (const auto &m: matches) { ++counts[m.source]; }
            if (std::any_of(counts.begin() + 1, counts.end(),
                            [&](int c) { return c < o.min_matches; })) {
                out.reason = "insufficient_matches";
                return out;
            }
            const auto previous = w;
            if (!optimize(w, scans, imu, matches, o)) {
                out.reason = "optimization_not_converged";
                return out;
            }
            double pose_change = 0;
            for (int i = 0; i < n; ++i) {
                pose_change = std::max(pose_change, (w.knots[i].p - previous.knots[i].p).norm());
                pose_change = std::max(pose_change, log(previous.knots[i].r.transpose() * w.knots[i].r).norm());
            }
            if (outer > 0 && pose_change < 0.005 &&
                (w.gravity - previous.gravity).norm() < 0.02 && (w.ba - previous.ba).norm() < 0.02) {
                converged = true;
                break;
            }
        }
        if (!converged) {
            out.reason = "deskew_not_converged";
            return out;
        }
        clouds = deskew(w, scans, imu);
        matches = match(w, clouds, o);
        const auto r = residuals(w, scans, imu, matches, o);
        const auto j = jacobian(w, scans, imu, matches, o, r);
        const Eigen::MatrixXd h = j.transpose() * j;
        if (!r.allFinite() || !h.allFinite() || matches.empty()) {
            out.reason = "nonfinite_solution";
            return out;
        }
        const Eigen::VectorXd scaling = h.diagonal().array().max(1e-12).sqrt().inverse();
        const Eigen::MatrixXd normalized = scaling.asDiagonal() * h * scaling.asDiagonal();
        const Eigen::SelfAdjointEigenSolver<Eigen::MatrixXd> eig(normalized);
        if (eig.info() != Eigen::Success) {
            out.reason = "invalid_information";
            return out;
        }
        out.information_ratio = eig.eigenvalues()[0] / eig.eigenvalues().tail(1)[0];
        if (out.information_ratio < 1e-7) {
            out.reason = "insufficient_excitation";
            return out;
        }
        out.plane_rms = r.head(matches.size()).norm() * o.plane_sigma / std::sqrt(matches.size());
        out.imu_rms = r.segment(matches.size(), 9 * (n - 1)).norm() / std::sqrt(9 * (n - 1));
        if (out.plane_rms > 0.08 || out.imu_rms > 3.0 ||
            std::abs(w.gravity.norm() - o.gravity) > 0.05 || w.bg.norm() > 0.3 || w.ba.norm() > 2.0) {
            out.reason = "residual_or_bias_limit";
            return out;
        }
        const auto factor = h.ldlt();
        const Eigen::MatrixXd covariance = factor.solve(Eigen::MatrixXd::Identity(h.rows(), h.cols())) *
                                           std::max(1.0, r.squaredNorm() / std::max(1.0, double(r.size() - h.rows())));
        const int shared = shared_offset(n);
        if (!covariance.allFinite() || covariance.block<3, 3>(shared, shared).trace() > std::pow(o.gravity * 0.08, 2) ||
            covariance.block<3, 3>(shared + 3, shared + 3).trace() > 0.05 * 0.05 ||
            covariance.block<3, 3>(shared + 6, shared + 6).trace() > 0.5 * 0.5) {
            out.reason = "initial_state_uncertain";
            return out;
        }
        const M3 align = Eigen::Quaterniond::FromTwoVectors(w.gravity, target_gravity).toRotationMatrix();
        const auto &last = w.knots.back();
        out.rotation = align * last.r;
        out.position = align * last.p;
        out.velocity = align * last.v;
        out.gravity = target_gravity.normalized() * o.gravity;
        out.gyro_bias = w.bg;
        out.accel_bias = w.ba;
        const auto measurement = sample(imu, scans.back().time);
        out.omega = measurement.gyro - w.bg;
        out.force = measurement.force - w.ba;
        out.time = scans.back().time;
        // Gravity alignment is a gauge choice. Propagate its sensitivity as well as
        // the solved terminal state; otherwise tilt uncertainty would be lost.
        const auto output_vector = [&](const Window &x) {
            const M3 a = Eigen::Quaterniond::FromTwoVectors(x.gravity, target_gravity).toRotationMatrix();
            Eigen::Matrix<double, 30, 1> delta = Eigen::Matrix<double, 30, 1>::Zero();
            delta.segment<3>(0) = a * x.knots.back().p - out.position;
            delta.segment<3>(3) = log(out.rotation.transpose() * a * x.knots.back().r);
            delta.segment<3>(12) = a * x.knots.back().v - out.velocity;
            delta.segment<3>(15) = w.bg - x.bg;
            delta.segment<3>(18) = w.ba - x.ba;
            delta.segment<3>(21) = target_gravity.normalized() * (x.gravity.norm() - o.gravity);
            delta.segment<3>(24) = x.bg - w.bg;
            delta.segment<3>(27) = x.ba - w.ba;
            return delta;
        };
        Eigen::MatrixXd lift(30, h.rows());
        const auto base = output_vector(w);
        for (int col = 0; col < h.rows(); ++col) {
            Eigen::VectorXd dx = Eigen::VectorXd::Zero(h.rows());
            dx[col] = 1e-6;
            lift.col(col) = (output_vector(increment(w, dx)) - base) / dx[col];
        }
        out.covariance = lift * covariance * lift.transpose();
        out.covariance.block<3, 3>(15, 15).diagonal().array() += o.gyro_measurement_variance;
        out.covariance.block<3, 3>(18, 18).diagonal().array() += o.accel_measurement_variance;
        out.covariance = (0.5 * (out.covariance + out.covariance.transpose())).eval();
        for (int i = 0; i < n; ++i) {
            for (const auto &point: clouds[i]) {
                out.map.push_back((align * (w.knots[i].r * point + w.knots[i].p)).cast<float>());
            }
        }
        out.success = true;
        out.reason = "initialized";
        return out;
    }
}// namespace small_point_lio::initialization
