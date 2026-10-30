#include "moment_map.h"
#include <stdexcept>
#ifdef SMALL_POINT_LIO_HAS_MOMENT_MAP
#include "zlio/map.hpp"
#endif

namespace small_point_lio {
#ifdef SMALL_POINT_LIO_HAS_MOMENT_MAP
    struct MomentMap::Impl {
        zlio::VoxelMap map;
        std::vector<zlio::MapPoint> pending;
        Mat6 last_pose_covariance;
        zlio::MapPoseCovariance shared_pose;
        explicit Impl(double resolution, std::size_t capacity)
            : map([&] { zlio::MapConfig c; c.voxel_size = resolution; c.maximum_roots = capacity; return c; }()) {}
    };
    bool MomentMap::available() { return true; }
    MomentMap::MomentMap(double resolution, std::size_t capacity) : impl(std::make_unique<Impl>(resolution, capacity)) {}
    void MomentMap::insert(const Eigen::Vector3d &world, const Eigen::Matrix3d &sensor_cov,
                           const Mat6 &pose_cov, bool defer) {
        if (!impl->shared_pose || !(impl->last_pose_covariance.array() == pose_cov.array()).all()) {
            impl->last_pose_covariance = pose_cov;
            impl->shared_pose = zlio::MapPoseCovariance(pose_cov);
        }
        zlio::MapPoint point{world, sensor_cov, impl->shared_pose};
        if (defer) { impl->pending.push_back(std::move(point)); }
        else { impl->map.insert(std::vector<zlio::MapPoint>{std::move(point)}); }
    }
    void MomentMap::flush() {
        impl->map.insert(impl->pending);
        impl->pending.clear();
    }
    std::optional<MomentMap::Match> MomentMap::match(const Eigen::Vector3d &world, const Eigen::Matrix3d &covariance) const {
        auto found = impl->map.match(world, covariance);
        if (!found) { return std::nullopt; }
        return Match{found->normal, found->residual, found->plane_variance + found->surface_variance};
    }
    std::size_t MomentMap::size() const { return impl->map.size(); }
#else
    struct MomentMap::Impl {};
    bool MomentMap::available() { return false; }
    MomentMap::MomentMap(double, std::size_t) {
        throw std::invalid_argument("map_backend=moment requires building with SMALL_POINT_LIO_ZLIO_MAP=ON");
    }
    void MomentMap::insert(const Eigen::Vector3d &, const Eigen::Matrix3d &, const Mat6 &, bool) {}
    void MomentMap::flush() {}
    std::optional<MomentMap::Match> MomentMap::match(const Eigen::Vector3d &, const Eigen::Matrix3d &) const { return std::nullopt; }
    std::size_t MomentMap::size() const { return 0; }
#endif
    MomentMap::~MomentMap() = default;
}
