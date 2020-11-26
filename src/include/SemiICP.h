/*
 * @Author: your name
 * @Date: 2020-10-28 10:13:53
 * @LastEditTime: 2020-11-03 11:50:53
 * @LastEditors: Please set LastEditors
 * @Description: In User Settings Edit
 * @FilePath: /semICP/SemiICP.h
 */
#include <vector>
#include <map>

#include <pcl/point_types.h>
#include <pcl/point_cloud.h>

#include "SemanticPointCloud.h"
#include "sophus/se3.hpp"

// #include "PointType.hpp"

namespace SemanticICP
{

    template <typename PointT, typename SemanticT>
    class SemiICP
    {
    public:
        typedef SemanticPointCloud<PointT, SemanticT> SemanticCloud;
        typedef typename std::shared_ptr<SemanticCloud> SemanticCloudPtr;
        // typedef typename std::shared_ptr< const SemanticCloud> SemanticCloudConstPtr;

        // typedef std::vector< Eigen::Matrix3d, Eigen::aligned_allocator<Eigen::Matrix3d> > MatricesVector;
        typedef std::vector<Eigen::Matrix<double, 6, 6>,
                            Eigen::aligned_allocator<Eigen::Matrix<double, 6, 6>>>
            CovarianceVector;

        // typedef std::shared_ptr< MatricesVector > MatricesVectorPtr;
        // typedef std::shared_ptr< const MatricesVector > MatricesVectorConstPtr;

        typedef pcl::KdTreeFLANN<PointT> KdTree;
        typedef typename KdTree::Ptr KdTreePtr;

        // typedef Eigen::Matrix<double, 6, 1> Vector6d;

        SemiICP(/* args */)
        {
            Eigen::Matrix4d mat = Eigen::Matrix4d::Identity();
            baseTransformation_ = Sophus::SE3d(mat);
        }
        ~SemiICP() {}

        // private:
        void pcl_2_semantic(const pcl::PointCloud<pcl::PointXYZL>::Ptr pclCloud,
                            std::shared_ptr<SemanticPointCloud<pcl::PointXYZ, uint32_t>> &semanticCloud);

        void pcl2semantic(const pcl::PointCloud<pcl::PointXYZRGBL>::Ptr pclCloud,
                          std::shared_ptr<SemanticPointCloud<pcl::PointXYZRGB, uint32_t>> &semanticCloud);

        inline void setInputSource(const SemanticCloudPtr &cloud)
        {
            sourceCloud_ = cloud;
        };

        inline void setInputTarget(const SemanticCloudPtr &cloud)
        {
            targetCloud_ = cloud;
        };

        void align(SemanticCloudPtr finalCloud);

        void align(SemanticCloudPtr finalCloud, Sophus::SE3d &initTransform);

        Sophus::SE3d getFinalTransFormation()
        {
            Sophus::SE3d temp = finalTransformation_;
            return temp;
        };

    public:
        Sophus::SE3d baseTransformation_;
        Sophus::SE3d finalTransformation_;

        SemanticCloudPtr sourceCloud_;
        SemanticCloudPtr targetCloud_;
    };

} // namespace SemanticICP
