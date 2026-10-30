#pragma once
#include <Eigen/Core>
#include <memory>
#include <optional>

namespace small_point_lio {
    // Bridge to the unmodified, optional z-localization map. Coordinates and
    // covariances use its world-frame left pose error [translation, rotation].
    class MomentMap {
    public:
        using Mat6 = Eigen::Matrix<double, 6, 6>;
        struct Match {
            Eigen::Vector3d normal;
            double residual, variance;
        };
        explicit MomentMap(double resolution, std::size_t capacity);
        ~MomentMap();
        static bool available();
        void insert(const Eigen::Vector3d &world, const Eigen::Matrix3d &sensor_cov,
                    const Mat6 &pose_cov, bool defer = false);
        void flush();
        std::optional<Match> match(const Eigen::Vector3d &world, const Eigen::Matrix3d &covariance) const;
        std::size_t size() const;
    private:
        struct Impl;
        std::unique_ptr<Impl> impl;
    };
}
