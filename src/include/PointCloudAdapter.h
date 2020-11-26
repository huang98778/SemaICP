/*
 * @Author: your name
 * @Date: 2020-10-30 10:24:47
 * @LastEditTime: 2020-11-01 15:21:45
 * @LastEditors: Please set LastEditors
 * @Description: In User Settings Edit
 * @FilePath: /semICP/PointCloudAdapter.h
 */

#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_cloud.h>
// #include <memory>

#include "SemanticPointCloud.h"
#include "Def.h"
// #include "PointType.hpp"

class PointCloudAdapter
{
private:
    /* data */
public:
    PointCloudAdapter(/* args */);
    ~PointCloudAdapter();

    // template <typename PointT>
    static void decentralize(pcl::PointCloud<pcl::PointXYZRGBL>::Ptr &pc);

    // static void interpolate(std::vector<std::pair<std::string, pcl::PointCloud<pcl::PointXYZ>::Ptr>> &pcs);
    static void decentralize(pcl::PointCloud<pcl::PointXYZL>::Ptr &pc,
                             pcl::PointCloud<pcl::PointXYZL>::Ptr &output);

    template <typename PointT>
    static bool loadPointCloudT(const std::string &dir, typename pcl::PointCloud<PointT>::Ptr &pc);

    static bool loadPointCloud(const std::string &dir, pcl::PointCloud<pcl::PointXYZRGBL>::Ptr &pc);

    static void pcl2object(const pcl::PointCloud<pcl::PointXYZRGBL>::Ptr pclCloud,
                           std::map<uint8_t, pcl::PointCloud<pcl::PointXYZL>::Ptr> &objcloud);

    static bool getSameObjPointCloud(const pcl::PointCloud<pcl::PointXYZL>::Ptr &cloud,
                                     const std::shared_ptr<SemanticICP::SemanticPointCloud<pcl::PointXYZRGB, uint32_t>> target,
                                         pcl::PointCloud<pcl::PointXYZL>::Ptr &resCloud);
};
