/*
 * @Author: your name
 * @Date: 2020-10-28 10:14:11
 * @LastEditTime: 2020-11-03 13:58:35
 * @LastEditors: Please set LastEditors
 * @Description: In User Settings Edit
 * @FilePath: /semICP/SemiICP.cpp
 */
#include <glog/logging.h>
#include <ceres/ceres.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/common/transforms.h>

#include "SemiICP.h"
#include "SemanticPointCloud.h"

// #include "gicp_cost_function.h"
#include "GicpCostFunctionDef.h"

using namespace SemanticICP;

template <typename PointT, typename SemanticT>
void SemiICP<PointT, SemanticT>::pcl_2_semantic(const pcl::PointCloud<pcl::PointXYZL>::Ptr pclCloud,
                                                std::shared_ptr<SemanticPointCloud<pcl::PointXYZ, uint32_t>> &semanticCloud)
{

    typedef pcl::PointCloud<pcl::PointXYZ> PointCloud;
    typedef typename PointCloud::Ptr PointCloudPtr;

    std::vector<uint32_t> labels;
    std::map<uint32_t, PointCloudPtr> map;

    for (pcl::PointXYZL p : pclCloud->points)
    {
        if (map.find(p.label) == map.end())
        {
            PointCloudPtr cloud(new PointCloud());
            cloud->push_back(pcl::PointXYZ(p.x, p.y, p.z));
            map[p.label] = cloud;
            labels.push_back(p.label);
        }
        else
        {
            PointCloudPtr cloud = map[p.label];
            cloud->push_back(pcl::PointXYZ(p.x, p.y, p.z));
        }
        // LOG(INFO) << "point xyz: " << p.x << p.y << p.z;
    }

    for (uint32_t l : labels)
    {
        PointCloudPtr cloud = map[l];
        // LOG(INFO) << " map pc size: " << l << " " << cloud->size();
        semanticCloud->addSemanticCloud(l, cloud);
    }
    // semanticCloud->sumad(1, 2);
}

template <typename PointT, typename SemanticT>
void SemiICP<PointT, SemanticT>::pcl2semantic(const pcl::PointCloud<pcl::PointXYZRGBL>::Ptr pclCloud,
                                              std::shared_ptr<SemanticPointCloud<pcl::PointXYZRGB, uint32_t>> &semanticCloud)
{
    typedef pcl::PointXYZRGB PointPR;
    typedef pcl::PointCloud<PointPR> PointCloud;
    typedef typename PointCloud::Ptr PointCloudPtr;

    std::vector<uint32_t> labels;
    std::map<uint32_t, PointCloudPtr> map;

    for (pcl::PointXYZRGBL p : pclCloud->points)
    {
        if (map.find(p.label) == map.end())
        {
            PointCloudPtr cloud(new PointCloud());
            PointPR pt;
            pt.x = p.x;
            pt.y = p.y;
            pt.z = p.z;
            pt.r = pt.g = pt.b = p.r;

            cloud->push_back(pt);
            map[p.label] = cloud;
            labels.push_back(p.label);
        }
        else
        {
            PointCloudPtr cloud = map[p.label];
            PointPR pt;
            pt.x = p.x;
            pt.y = p.y;
            pt.z = p.z;
            pt.r = pt.g = pt.b = p.r;
            cloud->push_back(pt);
        }
        // LOG(INFO) << "point xyz: " << p.x << p.y << p.z;
    }

    for (uint32_t l : labels)
    {
        PointCloudPtr cloud = map[l];
        // LOG(INFO) << "====: " << cloud->size();
        semanticCloud->addSemanticCloud(l, cloud);
    }
}


template <typename PointT, typename SemanticT>
void SemiICP<PointT, SemanticT>::align(SemanticCloudPtr finalCloud)
{
    Eigen::Matrix4d mat = Eigen::Matrix4d::Identity();
    Sophus::SE3d init(mat);
    align(finalCloud, init);
};

template <typename PointT, typename SemanticT>
void SemiICP<PointT, SemanticT>::align(SemanticCloudPtr finalCloud, Sophus::SE3d &initTransform)
{

    // sourceCloud_->sumad(1, 5);
    for (size_t i = 0; i < sourceCloud_->semanticLabels.size(); i++)
    {
        LOG(INFO) << "***source labels : ***" << sourceCloud_->semanticLabels[i];
    }
    for (size_t i = 0; i < targetCloud_->semanticLabels.size(); i++)
    {
        LOG(INFO) << "***target labels : ******" << targetCloud_->semanticLabels[i];
    }

    //TODO:
    Sophus::SE3d currentTransform(initTransform);
    bool converged = false;
    size_t count = 0;

    while (!converged)
    {
        std::vector<Sophus::SE3d> transformsVec;
        CovarianceVector covVec;

        // problem
        ceres::Problem problem;

        Sophus::SE3d estTransform(currentTransform);
        double mesHigh = 0;
        count++;

        for (SemanticT s : sourceCloud_->semanticLabels)
        {
            if (targetCloud_->labeledPointClouds.find(s) != targetCloud_->labeledPointClouds.end())
            {
                LOG(INFO) << "have same label cloud in target cloud..." << s;
                if (targetCloud_->labeledPointClouds[s]->size() > 5) // TODO:  size threshold
                {
                    typename pcl::PointCloud<PointT>::Ptr transformedSource(new pcl::PointCloud<PointT>());
                    Sophus::SE3d transform = currentTransform;
                    Eigen::Matrix4d transMat = transform.matrix();
                    pcl::transformPointCloud(*(sourceCloud_->labeledPointClouds[s]),
                                             *transformedSource,
                                             transMat);
                    KdTreePtr tree = targetCloud_->labeledKdTrees[s];
                    std::vector<int> targetIndx;
                    std::vector<float> distSq;
                    LOG(INFO) << "Num Points int transformedSource: " << transformedSource->size();

                    for (int sourceIndx = 0; sourceIndx < transformedSource->size(); sourceIndx++)
                    {
                        const PointT &transfromedSourcePoint = transformedSource->points[sourceIndx];
                        tree->nearestKSearch(transfromedSourcePoint, 1, targetIndx, distSq);
                        LOG(INFO) << "transformedSource nearest size in target cloud: " << targetIndx.size();

                        for (int i = 0; i < targetIndx.size(); i++)
                        {
                            LOG(INFO) << "kd search : " << targetIndx[i] << "   " << distSq[i];
                        }
                        if (distSq[0] < 5000) // TODO:  dis calcu
                        {
                            const PointT sourcePoint = (sourceCloud_->labeledPointClouds[s])->points[sourceIndx];
                            const Eigen::Matrix3d sourceCov = (sourceCloud_->labeledCovariances[s])->at(sourceIndx);

                            const PointT targetPoint = (targetCloud_->labeledPointClouds[s])->points[targetIndx[0]];
                            const Eigen::Matrix3d targetCov = (targetCloud_->labeledCovariances[s])->at(targetIndx[0]);

                            ceres::CostFunction *cost_function = new GICPCostFunctionT(sourcePoint,
                                                                                       targetPoint,
                                                                                       sourceCov,
                                                                                       targetCov,
                                                                                       baseTransformation_);
                            problem.AddResidualBlock(cost_function, new ceres::CauchyLoss(0.1), estTransform.data()); // TODO: CauchyLoss threshold
                        }
                        // TODO:  // Gradient Check

                    } // For loop over points
                }
            }
        }

        // Solve options
        ceres::Solver::Options option;
        // option.gradient_tolerance = 0.1 * Sophus::Constants<double>::epsilon();
        // option.function_tolerance = 0.1 * Sophus::Constants<double>::epsilon();
        option.linear_solver_type = ceres::DENSE_QR;
        // option.num_threads = 4;
        option.max_num_iterations = 100;
        // option.gradient_check_numeric_derivative_relative_step_size = 1e-8;
        // option.gradient_check_relative_precision = 1e-6;

        // Solve
        ceres::Solver::Summary summary;
        ceres::Solve(option, &problem, &summary);
        std::cout << summary.BriefReport() << std::endl;

        double mse = (currentTransform.inverse() * estTransform).log().squaredNorm();
        if (mse < 0.0000000001 || count > 55)
            converged = true;

        LOG(INFO) << "MSE: " << mse << std::endl;
        LOG(INFO) << "Transform: " << std::endl;
        LOG(INFO) << "\n"
                  << estTransform.matrix() << std::endl;
        LOG(INFO) << "Itteration: " << count << std::endl;
        currentTransform = estTransform;
    }

    finalTransformation_ = currentTransform;

    Sophus::SE3d trans = finalTransformation_ * baseTransformation_;
    Eigen::Matrix4f mat = (trans.matrix()).cast<float>();
    finalCloud->transform(mat);
}

template class SemiICP<pcl::PointXYZRGB, uint32_t>;
