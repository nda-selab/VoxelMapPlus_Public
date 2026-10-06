#include <thread>
#include "preprocess.h"
#include <cmath>

Preprocess::Preprocess()
        : lidar_type(AVIA), blind(0.01), point_filter_num(1) {
    N_SCANS = 6;
}

Preprocess::~Preprocess() {}

// Avia input
void Preprocess::process(const livox_ros_driver::CustomMsg::ConstPtr &msg,
                         PointCloudXYZI::Ptr &pcl_surf_out) {
    avia_handler(msg);
    *pcl_surf_out = pl_surf;
}

// Other input
void Preprocess::process(const sensor_msgs::PointCloud2::ConstPtr &msg,
                         PointCloudXYZI::Ptr &pcl_out) {
    switch (lidar_type) {
        case L515:
            l515_handler(msg);
            break;
        case VELO16:
            velodyne_handler(msg);
            break;
        default:
            printf("Error LiDAR Type");
            break;
    }
    *pcl_out = pl_surf;
}

// 无特征提取，把所有点都当作了面特征
void Preprocess::avia_handler(
        const livox_ros_driver::CustomMsg::ConstPtr &msg) {
    auto t_feature_start = std::chrono::high_resolution_clock::now();
    pl_surf.clear();
    pl_corn.clear();
    pl_full.clear();
    int plsize = msg->point_num;
    std::vector<bool> is_valid_pt(plsize, false);

    pl_corn.reserve(plsize);
    pl_surf.reserve(plsize);
    pl_full.resize(plsize);

    for (int i = 0; i < N_SCANS; i++) {
        pl_buff[i].clear();
        pl_buff[i].reserve(plsize);
    }
    uint valid_num = 0;

    for (uint i = 1; i < plsize; i++) {
        if ((msg->points[i].line < N_SCANS) &&
            ((msg->points[i].tag & 0x30) == 0x10 ||
             (msg->points[i].tag & 0x30) == 0x00)) {
            if (i % point_filter_num == 0) {
                pl_full[i].x = msg->points[i].x;
                pl_full[i].y = msg->points[i].y;
                pl_full[i].z = msg->points[i].z;
                pl_full[i].intensity = msg->points[i].reflectivity;
                pl_full[i].curvature =
                        msg->points[i].offset_time /
                        float(1000000); // use curvature as time of each laser points

                if (pl_full[i].x * pl_full[i].x + pl_full[i].y * pl_full[i].y +
                    pl_full[i].z + pl_full[i].z < blind * blind) {
                    continue;
                }
                if(isinf(pl_full[i].x) || isinf(pl_full[i].y) || isinf(pl_full[i].z)){
                    continue;
                }

                if ((abs(pl_full[i].x - pl_full[i - 1].x) > 1e-7) ||
                    (abs(pl_full[i].y - pl_full[i - 1].y) > 1e-7) ||
                    (abs(pl_full[i].z - pl_full[i - 1].z) > 1e-7)) {
                    is_valid_pt[i] = true;
                    valid_num++;
                }
            }
        }
    }

    for (uint i = 1; i < plsize; i++) {
        if (is_valid_pt[i]) {
            pl_surf.points.push_back(pl_full[i]);
        }
    }
}

void Preprocess::l515_handler(const sensor_msgs::PointCloud2::ConstPtr &msg) {
    pl_surf.clear();
    pcl::PointCloud<velodyne_ros::Point> pl_orig;
    pcl::fromROSMsg(*msg, pl_orig);
    int plsize = pl_orig.points.size();
    for (int i = 0; i < plsize; i++) {
        PointType added_pt;
        added_pt.x = pl_orig.points[i].x;
        added_pt.y = pl_orig.points[i].y;
        added_pt.z = pl_orig.points[i].z;
        added_pt.intensity = pl_orig.points[i].intensity;
        if (i % point_filter_num == 0) {
            pl_surf.push_back(added_pt);
        }
    }
}

#define MAX_LINE_NUM 64

void Preprocess::velodyne_handler(
        const sensor_msgs::PointCloud2::ConstPtr &msg) {
    pl_surf.clear();
    pl_corn.clear();
    pl_full.clear();

    pcl::PointCloud<velodyne_ros::Point> pl_orig;
    pcl::fromROSMsg(*msg, pl_orig);

    const int plsize = pl_orig.points.size();
    pl_surf.reserve(plsize);

    for (int i = 0; i < plsize; i++) {
        if (i % point_filter_num != 0) {
            continue;
        }

        const auto &src = pl_orig.points[i];

        if (src.ring >= N_SCANS) {
            continue;
        }

        if (!std::isfinite(src.x) ||
            !std::isfinite(src.y) ||
            !std::isfinite(src.z)) {
            continue;
        }

        const double range2 =
            src.x * src.x +
            src.y * src.y +
            src.z * src.z;

        if (range2 < blind * blind) {
            continue;
        }

        PointType added_pt;
        added_pt.x = src.x;
        added_pt.y = src.y;
        added_pt.z = src.z;
        added_pt.intensity = src.intensity;

        // time [s] -> curvature [ms]
        added_pt.curvature = src.time * 1000.0f;

        pl_surf.push_back(added_pt);
    }
}
