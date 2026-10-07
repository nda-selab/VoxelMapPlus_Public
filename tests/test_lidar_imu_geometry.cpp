#include <gtest/gtest.h>
#include "../src/IMU_Processing.hpp"
#include <limits>

using voxel_map_plus::LidarImuGeometry;
using voxel_map_plus::pointPoseJacobian;
using voxel_map_plus::pointCovarianceWorld;
using voxel_map_plus::skew;

namespace {
M3D rotation(double angle, const V3D &axis = V3D(0, 0, 1)) {
    return Eigen::AngleAxisd(angle, axis.normalized()).toRotationMatrix();
}

LidarImuGeometry calibration() {
    LidarImuGeometry ext;
    ext.set(V3D(0.8, -0.3, 0.2), rotation(0.7, V3D(1, 2, 3)));
    return ext;
}

sensor_msgs::Imu::Ptr imu(double time, double yaw_rate = 0) {
    sensor_msgs::Imu::Ptr msg(new sensor_msgs::Imu());
    msg->header.stamp.fromSec(time);
    msg->linear_acceleration.z = G_m_s2;
    msg->angular_velocity.z = yaw_rate;
    return msg;
}
}

TEST(LidarImuGeometry, IdentityAndKnownTransform) {
    LidarImuGeometry ext;
    const V3D point(2, 3, 4), translation(5, 6, 7);
    const M3D r = rotation(0.4);
    EXPECT_TRUE(ext.toWorld(point, r, translation).isApprox(r * point + translation, 1e-12));
    ext.set(V3D(1, 2, 3), rotation(M_PI / 2));
    EXPECT_TRUE(ext.toImu(V3D(1, 0, 0)).isApprox(V3D(1, 3, 3), 1e-12));
    // Published LiDAR pose must map the same cloud as the mapping path.
    EXPECT_TRUE(ext.toWorld(point, r, translation).isApprox(
            ext.lidarRotation(r) * point + ext.lidarPosition(r, translation), 1e-12));
}

TEST(LidarImuGeometry, DeskewPreservesWorldPointAndStaticScan) {
    const auto ext = calibration();
    const V3D point(3, -2, 1), start(1, 2, 3), end(2, 4, 6);
    const M3D r_start = rotation(0.2), r_end = rotation(0.9, V3D(1, 1, 2));
    const V3D compensated = ext.deskew(point, r_start, start, r_end, end);
    EXPECT_TRUE(ext.toWorld(compensated, r_end, end).isApprox(
            ext.toWorld(point, r_start, start), 1e-12));
    EXPECT_TRUE(ext.deskew(point, r_start, start, r_start, start).isApprox(point, 1e-12));
}

TEST(LidarImuGeometry, PoseAndPlaneJacobiansMatchFiniteDifferences) {
    const auto ext = calibration();
    const V3D point(3, -2, 1), position(1, 2, 3), normal = V3D(1, -2, 3).normalized();
    const M3D r = rotation(0.6, V3D(1, 3, 2));
    const auto analytic = pointPoseJacobian(r, skew(ext.toImu(point)));
    Eigen::Matrix<double, 3, 6> numerical;
    const double step = 1e-6;
    for (int axis = 0; axis < 3; ++axis) {
        const V3D direction = V3D::Unit(axis);
        numerical.col(axis) = (ext.toWorld(point, r * rotation(step, direction), position) -
                               ext.toWorld(point, r * rotation(-step, direction), position)) / (2 * step);
        numerical.col(axis + 3) = (ext.toWorld(point, r, position + step * direction) -
                                   ext.toWorld(point, r, position - step * direction)) / (2 * step);
    }
    EXPECT_LT((analytic - numerical).norm(), 1e-8);
    EXPECT_LT((normal.transpose() * analytic - normal.transpose() * numerical).norm(), 1e-8);
}

TEST(LidarImuGeometry, CovarianceMatchesNumericalPropagationIncludingCrossTerms) {
    const auto ext = calibration();
    const V3D point(3, 2, 1), position(1, 0, 2);
    const M3D r = rotation(0.5, V3D(1, 2, 3));
    Eigen::Matrix<double, 9, 9> joint = Eigen::Matrix<double, 9, 9>::Identity() * 0.01;
    joint(0, 0) = 0.02;
    joint(1, 1) = 0.03;
    joint(3, 6) = joint(6, 3) = 0.002;
    joint(4, 8) = joint(8, 4) = -0.003;
    Eigen::Matrix<double, 3, 9> numerical;
    const double step = 1e-6;
    for (int axis = 0; axis < 3; ++axis) {
        const V3D d = V3D::Unit(axis);
        numerical.col(axis) = (ext.toWorld(point + step * d, r, position) -
                               ext.toWorld(point - step * d, r, position)) / (2 * step);
        numerical.col(axis + 3) = (ext.toWorld(point, r * rotation(step, d), position) -
                                   ext.toWorld(point, r * rotation(-step, d), position)) / (2 * step);
        numerical.col(axis + 6) = (ext.toWorld(point, r, position + step * d) -
                                   ext.toWorld(point, r, position - step * d)) / (2 * step);
    }
    const M3D expected = numerical * joint * numerical.transpose();
    const M3D actual = pointCovarianceWorld(r, skew(ext.toImu(point)),
            ext.covarianceToImu(joint.topLeftCorner<3, 3>()), joint.bottomRightCorner<6, 6>());
    EXPECT_LT((actual - expected).norm(), 1e-8);
    EXPECT_LT((actual - actual.transpose()).norm(), 1e-12);
}

TEST(LidarImuGeometry, LongCorridorCalibrationBecomesProperRotation) {
    M3D r;
    r << 0.999212900, -0.000519121, 0.004000000,
         0.000516111, 0.999218492, -0.000939132,
        -0.004000000, 0.000802565, 0.999993652;
    LidarImuGeometry ext;
    EXPECT_GT(ext.set(V3D(0.08, 0.029, 0.03), r), 1e-6);
    EXPECT_NEAR(ext.rotation.determinant(), 1.0, 1e-12);
    EXPECT_LT((ext.rotation.transpose() * ext.rotation - M3D::Identity()).norm(), 1e-12);
    const V3D point(1, 2, 3);
    EXPECT_TRUE(ext.deskew(point, M3D::Identity(), V3D::Zero(),
                          M3D::Identity(), V3D::Zero()).isApprox(point, 1e-12));
}

TEST(LidarImuGeometry, RejectsInvalidCalibration) {
    LidarImuGeometry ext;
    EXPECT_THROW(ext.set(V3D::Zero(), M3D::Zero()), std::invalid_argument);
    EXPECT_THROW(ext.set(V3D::Zero(), -M3D::Identity()), std::invalid_argument);
    EXPECT_THROW(ext.set(V3D::Zero(), 2 * M3D::Identity()), std::invalid_argument);
    EXPECT_THROW(ext.set(V3D(std::numeric_limits<double>::quiet_NaN(), 0, 0), M3D::Identity()),
                 std::invalid_argument);
}

TEST(ImuProcessing, MovingScanIncludesTimeZeroAndRepeatedPointTimes) {
    // Exercise the actual IMU integration and backwards point loop, not just
    // the geometry helper. All observations refer to one stationary landmark.
    for (bool non_identity : {false, true}) {
        ImuProcess processor;
        const auto ext = non_identity ? calibration() : LidarImuGeometry();
        processor.set_extrinsic(ext.translation, ext.rotation);
        StatesGroup state;
        PointCloudXYZI::Ptr output(new PointCloudXYZI());
        MeasureGroup init;
        init.lidar_beg_time = 0;
        for (int i = 0; i <= 200; ++i) {
            init.imu.push_back(imu(i * 0.005));
        }
        processor.Process(init, state, output);
        const V3D landmark(5, 3, 2), velocity(0.2, -0.1, 0);
        state.vel_end = velocity;
        MeasureGroup scan;
        scan.lidar_beg_time = 1.0;
        for (int i = 0; i <= 5; ++i) {
            scan.imu.push_back(imu(1.0 + i * 0.02, 1.0));
        }
        for (double t : {0.05, 0.0, 0.02, 0.0, 0.1}) {
            const V3D observed = ext.rotation.transpose() *
                    (rotation(t).transpose() * (landmark - velocity * t) - ext.translation);
            PointType p;
            p.x = observed.x(); p.y = observed.y(); p.z = observed.z();
            p.curvature = t * 1000;
            scan.surf_lidar->push_back(p);
        }
        processor.Process(scan, state, output);
        ASSERT_EQ(output->size(), scan.surf_lidar->size());
        EXPECT_TRUE(state.rot_end.isApprox(rotation(0.1), 1e-10));
        EXPECT_TRUE(state.pos_end.isApprox(velocity * 0.1, 1e-10));
        for (const auto &p : *output) {
            const V3D world = ext.toWorld(V3D(p.x, p.y, p.z), state.rot_end, state.pos_end);
            EXPECT_LT((world - landmark).norm(), 2e-6) << "point time=" << p.curvature;
        }
    }
}

int main(int argc, char **argv) {
    testing::InitGoogleTest(&argc, argv);
    ros::Time::init();
    return RUN_ALL_TESTS();
}
