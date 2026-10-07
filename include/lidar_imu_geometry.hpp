#ifndef LIDAR_IMU_GEOMETRY_HPP
#define LIDAR_IMU_GEOMETRY_HPP

#include <Eigen/Core>
#include <Eigen/Geometry>
#include <Eigen/SVD>
#include <stdexcept>

namespace voxel_map_plus {

inline Eigen::Matrix3d skew(const Eigen::Vector3d &p) {
    Eigen::Matrix3d result;
    result << 0, -p.z(), p.y(), p.z(), 0, -p.x(), -p.y(), p.x(), 0;
    return result;
}

// p_I = rotation * p_L + translation. State poses are IMU -> world;
// deskewed clouds remain in the LiDAR frame at the end of the scan.
struct LidarImuGeometry {
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    Eigen::Matrix3d rotation = Eigen::Matrix3d::Identity();
    Eigen::Vector3d translation = Eigen::Vector3d::Zero();

    // Keep calibration files unchanged, but use a proper rotation consistently
    // in forward/inverse transforms. Reject grossly invalid calibrations.
    double set(const Eigen::Vector3d &t, const Eigen::Matrix3d &r) {
        if (!t.allFinite() || !r.allFinite() || r.determinant() <= 0 ||
            (r.transpose() * r - Eigen::Matrix3d::Identity()).norm() > 0.1) {
            throw std::invalid_argument("extrinsic_R must be close to a proper rotation and extrinsic_T must be finite");
        }
        Eigen::JacobiSVD<Eigen::Matrix3d> svd(r, Eigen::ComputeFullU | Eigen::ComputeFullV);
        rotation = svd.matrixU() * svd.matrixV().transpose();
        translation = t;
        return (rotation - r).norm();
    }

    Eigen::Vector3d toImu(const Eigen::Vector3d &p) const {
        return rotation * p + translation;
    }

    Eigen::Vector3d toWorld(const Eigen::Vector3d &p, const Eigen::Matrix3d &r_wi,
                            const Eigen::Vector3d &t_wi) const {
        return r_wi * toImu(p) + t_wi;
    }

    Eigen::Matrix3d lidarRotation(const Eigen::Matrix3d &r_wi) const {
        return r_wi * rotation;
    }

    Eigen::Vector3d lidarPosition(const Eigen::Matrix3d &r_wi,
                                 const Eigen::Vector3d &t_wi) const {
        return r_wi * translation + t_wi;
    }

    Eigen::Vector3d deskew(const Eigen::Vector3d &p, const Eigen::Matrix3d &r_wi,
                          const Eigen::Vector3d &t_wi, const Eigen::Matrix3d &r_end,
                          const Eigen::Vector3d &t_end) const {
        return rotation.transpose() *
               (r_end.transpose() * (toWorld(p, r_wi, t_wi) - t_end) - translation);
    }

    Eigen::Matrix3d covarianceToImu(const Eigen::Matrix3d &cov_lidar) const {
        return rotation * cov_lidar * rotation.transpose();
    }
};

// Right perturbation for the IMU attitude, world-frame position error.
inline Eigen::Matrix<double, 3, 6> pointPoseJacobian(
        const Eigen::Matrix3d &r_wi, const Eigen::Matrix3d &point_imu_skew) {
    Eigen::Matrix<double, 3, 6> j;
    j.leftCols<3>() = -r_wi * point_imu_skew;
    j.rightCols<3>().setIdentity();
    return j;
}

inline Eigen::Matrix3d pointCovarianceWorld(
        const Eigen::Matrix3d &r_wi, const Eigen::Matrix3d &point_imu_skew,
        const Eigen::Matrix3d &cov_imu, const Eigen::Matrix<double, 6, 6> &pose_cov) {
    const auto j = pointPoseJacobian(r_wi, point_imu_skew);
    return r_wi * cov_imu * r_wi.transpose() + j * pose_cov * j.transpose();
}

} // namespace voxel_map_plus
#endif
