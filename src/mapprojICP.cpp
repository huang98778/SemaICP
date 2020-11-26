/*
 * @Author: your name
 * @Date: 2020-10-22 14:31:19
 * @LastEditTime: 2020-11-04 10:51:20
 * @LastEditors: Please set LastEditors
 * @Description: In User Settings Edit
 * @FilePath: /semICP/main.cpp
 */

#include <string>
#include <chrono>

#include "PerceptionParser.h"

#include <pcl/point_types.h>
#include <pcl/filters/filter.h>

#include <pcl/visualization/cloud_viewer.h>
#include <pcl/io/pcd_io.h>

#include "EMICP.h"
#include "SemiICP.h"
// #include <boost/filesystem.hpp>
#include <glog/logging.h>
#include "GlogInit.h"
#include "PointCloudAdapter.h"

// #include "PointType.hpp"

int main()
{
    logInit("semiIcp"); // google logging

    LOG(INFO) << "loading data ...";
    std::string projDir, dir1, dir2;
    projDir = "/home/hy/Ecarx_ws/HY520/SemiICP/semICP/data/xyzrgbl";

    typedef pcl::PointXYZRGBL PointT;

    pcl::PointCloud<PointT>::Ptr cloudS(new pcl::PointCloud<PointT>);
    pcl::PointCloud<PointT>::Ptr cloudT(new pcl::PointCloud<PointT>);

    dir1 = projDir + "/pcpImg.pcd";
    dir2 = projDir + "/projMap.pcd";
    // pcl::PointCloud<PointT>::Ptr pcS(new pcl::PointCloud<PointT>);
    // pcl::PointCloud<PointT>::Ptr pcT(new pcl::PointCloud<PointT>);
    if (!PointCloudAdapter::loadPointCloud(dir1, cloudS))
    {
        return -1;
    }

    if (!PointCloudAdapter::loadPointCloud(dir2, cloudT))
    {
        return -1;
    }

    std::vector<int> mapping_in;
    std::vector<int> mapping_out;
    pcl::removeNaNFromPointCloud(*cloudS, *cloudS, mapping_in);
    pcl::removeNaNFromPointCloud(*cloudT, *cloudT, mapping_out); //skipping NaNs

    LOG(INFO) << "***** source pc: ";
    PointCloudAdapter::decentralize(cloudS);
    LOG(INFO) << "***** target pc: ";
    PointCloudAdapter::decentralize(cloudT);

    pcl::PointCloud<PointT>::Ptr finalPc(new pcl::PointCloud<PointT>);
    pcl::PointCloud<PointT>::Ptr labeledCloudem(new pcl::PointCloud<PointT>);

    bool success = false;
    // int bicp = 2;
    //     // semi  icp
    LOG(INFO) << "semi icp start ...";
    std::shared_ptr<SemanticICP::SemanticPointCloud<pcl::PointXYZRGB, uint32_t>>
        semiS(new SemanticICP::SemanticPointCloud<pcl::PointXYZRGB, uint32_t>());
    std::shared_ptr<SemanticICP::SemanticPointCloud<pcl::PointXYZRGB, uint32_t>>
        semiT(new SemanticICP::SemanticPointCloud<pcl::PointXYZRGB, uint32_t>());

    SemanticICP::SemiICP<pcl::PointXYZRGB, uint32_t>  sicp;
    LOG(INFO) << "----------------------------------";
    LOG(INFO) << "---------     cloudS   -----------";
    LOG(INFO) << "--------cloudS size :" << cloudS->size() << "---------";
    sicp.pcl2semantic(cloudS, semiS);
    LOG(INFO) << "----------------------------------";
    LOG(INFO) << "---------     cloudT   -----------";
    LOG(INFO) << "--------cloudT size :" << cloudT->size() << "---------";
    sicp.pcl2semantic(cloudT, semiT);


    Eigen::Matrix4d temp = Eigen::Matrix4d::Identity();
    temp(1, 1) = 1;
    temp(0, 0) = 1;
    temp(2, 2) = 1;
    Eigen::Vector3d t(0, 10, 0);
    Sophus::SE3d initTransform(temp);
    sicp.setInputSource(semiS);
    sicp.setInputTarget(semiT);

    std::chrono::steady_clock::time_point t1 = std::chrono::steady_clock::now();
    sicp.align(semiS, initTransform);
    std::chrono::steady_clock::time_point t2 = std::chrono::steady_clock::now();
    std::chrono::duration<double> timeused = std::chrono::duration_cast<std::chrono::duration<double>>(t2 - t1);
    Sophus::SE3d sicpTranform = sicp.getFinalTransFormation();
    LOG(INFO) << "Final SICP Transform\n"
              << sicpTranform.matrix() << "\ntime cost: " << timeused.count() << std::endl;

    pcl::transformPointCloud(*cloudS, *finalPc, (sicpTranform.matrix()).cast<float>());

    //     // std::cout << "SICP Accuracy "
    //   << semanticICPMetrics.evaluate(finalPc, cloudT, numSource) << std::endl;
    // }
    std::map<uint8_t, pcl::PointCloud<pcl::PointXYZL>::Ptr> objectCloud;
    pcl::PointCloud<pcl::PointXYZL>::Ptr resCloud; //(new pcl::PointCloud<pcl::PointXYZL>);
    PointCloudAdapter::pcl2object(finalPc, objectCloud);
    LOG(INFO) << "re size: " << objectCloud.size();

    std::string name = "name: ";
    boost::shared_ptr<pcl::visualization::PCLVisualizer> viewerT(new pcl::visualization::PCLVisualizer(name));
    viewerT->addCoordinateSystem (1.0);
    viewerT->initCameraParameters ();
    int indx = 0;
    for (auto pct : objectCloud)
    {
        pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZL> rbs(pct.second, 0, 0, 255);
        pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZL> rgt(resCloud, 255, 0, 0);
        // pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZL> rr(finalPc, 0, 255, 0);

        LOG(INFO) << " source obj " << int(pct.first) << "  obj size: " << pct.second->size();

        resCloud.reset(new pcl::PointCloud<pcl::PointXYZL>);
        PointCloudAdapter::getSameObjPointCloud(pct.second, semiT, resCloud);
        LOG(INFO) << "taget obj: " << resCloud->size();
        viewerT->addPointCloud<pcl::PointXYZL>(pct.second, rbs, "cloud10-" + std::to_string(indx));
        viewerT->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 2, "cloud10-" + std::to_string(indx));
        viewerT->addPointCloud<pcl::PointXYZL>(resCloud, rgt, "cloud12-" + std::to_string(indx));
        viewerT->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 5, "cloud12-" + std::to_string(indx));

        indx++;

        viewerT->spin();
    }
    // viewerT->spin();

    LOG(INFO) << "align result: " << finalPc->points.size();

    success = true;

    bool bview = false;
    if (bview)
    {
        boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer1(new pcl::visualization::PCLVisualizer("init windows"));
        boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer2(new pcl::visualization::PCLVisualizer("semi icp result windows"));
        boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer3(new pcl::visualization::PCLVisualizer("result windows"));

        pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZRGBL> rb(cloudS, 0, 0, 255);
        pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZRGBL> rg(cloudT, 255, 0, 0);
        pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZRGBL> rr(finalPc, 0, 255, 0);

        viewer1->removePointCloud("cloud1");
        viewer1->addPointCloud<pcl::PointXYZRGBL>(cloudS, rg, "cloud1");
        viewer1->addPointCloud<pcl::PointXYZRGBL>(cloudT, rr, "cloud2");
        viewer1->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 2, "cloud1");

        viewer2->removePointCloud("cloud3");
        viewer2->addPointCloud<pcl::PointXYZRGBL>(cloudT, rr, "cloud3");
        viewer2->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 2, "cloud3");

        if (success)
        {
            viewer2->addPointCloud<pcl::PointXYZRGBL>(finalPc, rg, "cloud4");
            viewer2->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 2, "cloud4");
        }

        viewer3->removePointCloud("cloud3");
        viewer3->addPointCloud<pcl::PointXYZRGBL>(finalPc, rr, "cloud3");
        viewer3->addPointCloud<pcl::PointXYZRGBL>(cloudS, rg, "cloud3");
        viewer3->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 2, "cloud3");

        viewer1->spin();
        viewer2->spin();
        viewer3->spin();
    }
// */
    google::ShutdownGoogleLogging();

    return 0;
}