#include <gtest/gtest.h>
#include <m-detector/DynObjCluster.h>

namespace {
PointType point(float x, float y, float z) {
    PointType p{};
    p.x = x; p.y = y; p.z = z;
    return p;
}

TEST(SparseCluster, KeepsLabelsAndCleansOccupiedCellsAcrossFrames) {
    ros::Time::init();
    DynObjCluster cluster;
    cluster.configured_xyz_origin = Eigen::Vector3f::Constant(-2.f);
    cluster.configured_maprange = Eigen::Vector3f::Constant(4.f);
    cluster.Voxel_revolusion = .25f;
    cluster.cluster_min_pixel_number = 1;
    cluster.cluster_extend_pixel = 4;
    cluster.odom_rot = Eigen::Matrix3d::Identity();
    cluster.Init();
    pcl::PointCloud<PointType>::Ptr events(new pcl::PointCloud<PointType>());
    for (float x : {-.3f, .3f})
        for (float y : {-.3f, .3f})
            for (float z : {.8f, 1.4f}) events->push_back(point(x, y, z));
    auto raw = *events;
    raw.push_back(point(0, 0, 1.1f));
    raw.push_back(point(0, 0, 0));
    raw.push_back(point(.01f, .01f, 1.1f));
    for (int frame = 0; frame < 2; ++frame) {
        std::vector<int> tags(raw.size(), 0);
        tags.back() = -1;
        std::unordered_set<int> used;
        std::vector<std::vector<int>> clusters;
        bbox_t boxes;
        cluster.cluster_begin = ros::Time::now();
        cluster.GetClusterResult_voxel(events, cluster.umap, clusters, used);
        cluster.PubClusterResult_voxel(tags, std_msgs::Header(), boxes, .05, clusters, raw, used);
        ASSERT_EQ(boxes.Center.size(), 1u);
        for (size_t i = 0; i <= events->size(); ++i) EXPECT_EQ(tags[i], 1);
        EXPECT_EQ(tags[events->size()+1], 0);
        EXPECT_EQ(tags.back(), -1);
        EXPECT_LE(used.size(), raw.size());
        for (int voxel : used) {
            EXPECT_EQ(cluster.umap[voxel].points_num, 0);
            EXPECT_EQ(cluster.umap_ground[voxel].bbox_index, -1);
            EXPECT_EQ(cluster.umap_insidebox[voxel].bbox_index, -1);
        }
    }
}
} // namespace
