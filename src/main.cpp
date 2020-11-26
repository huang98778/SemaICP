/*
 * @Author: your name
 * @Date: 2020-10-22 14:31:19
 * @LastEditTime: 2020-10-31 14:14:28
 * @LastEditors: Please set LastEditors
 * @Description: In User Settings Edit
 * @FilePath: /semICP/main.cpp
 */

#include <string>

#include "PerceptionParser.h"

#include <pcl/point_types.h>
#include <pcl/visualization/cloud_viewer.h>
#include <pcl/io/pcd_io.h>


#include "EMICP.h"
#include "SemiICP.h"
// #include <boost/filesystem.hpp>
#include <glog/logging.h>
#include "GlogInit.h"
#include "PointCloudAdapter.h"

#include "PointType.hpp"

int main()
{
    logInit("semiIcp"); // google logging
    test();

    LOG(INFO) << "loading data ...";
    std::string projDir, dir1, dir2;
    projDir = "/home/hy/Ecarx_ws/HY520/semICP/data";
    // dir1 = projDir + "/data/20200928123816383.json";
    // dir2 = projDir + "/data/20200928123816583.json";

    // std::vector<std::pair<std::string, pcl::PointCloud<pcl::PointXYZ>::ConstPtr>> pcs1, pcs2;
    // PerceptionParser parser1, parser2;
    // pcs1.resize(0);
    // parser1.getPointCloud(dir1, pcs1);
    // pcs2.resize(0);
    // parser2.getPointCloud(dir2, pcs2);

    pcl::PointCloud<pcl::PointXYZL>::Ptr cloudS(new pcl::PointCloud<pcl::PointXYZL>);
    pcl::PointCloud<pcl::PointXYZL>::Ptr cloudT(new pcl::PointCloud<pcl::PointXYZL>);
    // parser1.convertVec2Pc(pcs1, cloudS);
    // parser2.convertVec2Pc(pcs2, cloudT);

    dir1 = projDir + "/pcpImg.pcd";
    dir2 = projDir + "/projMap.pcd";
    pcl::PointCloud<pcl::PointXYZL>::Ptr pcS(new pcl::PointCloud<pcl::PointXYZL>);
    pcl::PointCloud<pcl::PointXYZL>::Ptr pcT(new pcl::PointCloud<pcl::PointXYZL>);

    if (pcl::io::loadPCDFile<pcl::PointXYZL> (dir1, *pcS) == -1) //* load the file
    {
        PCL_ERROR ("Couldn't read source file\n");
        return (-1);
    }
    if (pcl::io::loadPCDFile<pcl::PointXYZL> (dir2, *pcT) == -1) //* load the file
    {
        PCL_ERROR ("Couldn't read source file\n");
        return (-1);
    }

    LOG(INFO) << "***** source pc: ";
    PointCloudAdapter::decentralize(pcS, cloudS);
    LOG(INFO) << "***** target pc: ";
    PointCloudAdapter::decentralize(pcT, cloudT);

    pcl::PointCloud<pcl::PointXYZL>::Ptr finalPc(new pcl::PointCloud<pcl::PointXYZL>);
    pcl::PointCloud<pcl::PointXYZL>::Ptr labeledCloudem(new pcl::PointCloud<pcl::PointXYZL>);

    bool success = false;
    int bicp = 2;
    if (bicp == 1)
    {
        // em icp
        LOG(INFO) << "em icp start ...";

        Eigen::Matrix<double, 11, 11> cm = Eigen::Matrix<double, 11, 11>::Identity();
        SemanticICP::EmIterativeClosestPoint emicp(11);
        emicp.setSourceCloud(cloudS);
        emicp.setTargetCloud(cloudT);
        emicp.setConfusionMatrix(cm);

        Eigen::Matrix4d T = Eigen::Matrix4d::Identity();
        Eigen::Matrix3d R = Eigen::Matrix3d::Identity();
        R(1, 1) = 1;
        R(0, 0) = 1;
        R(2, 2) = 0.95;
        Eigen::Vector3d t(10, 5, 0);
        Sophus::SE3d initT(T);
        emicp.align(finalPc, initT);

        Sophus::SE3d sicpTranform = emicp.getFinalTransFormation();
        // std::cout << "SICP MSE: " << semanticICPMetrics.evaluate(sicpTranform, indxTarget, indxSource, timeSICP, emicp.getOuterIter())
        //           << std::endl;
        // pcl::PointCloud<pcl::PointXYZL>::Ptr labeledCloudem(new pcl::PointCloud<pcl::PointXYZL>);
        emicp.getFusedLabels(labeledCloudem, sicpTranform);

        LOG(INFO) << "em icp done ...";
    }
    else if (bicp == 2)
    {
        // semi  icp
        LOG(INFO) << "semi icp start ...";
        std::shared_ptr<SemanticICP::SemanticPointCloud<pcl::PointXYZ, uint32_t>>
            semiS(new SemanticICP::SemanticPointCloud<pcl::PointXYZ, uint32_t>());
        std::shared_ptr<SemanticICP::SemanticPointCloud<pcl::PointXYZ, uint32_t>>
            semiT(new SemanticICP::SemanticPointCloud<pcl::PointXYZ, uint32_t>());

        SemanticICP::SemiICP<pcl::PointXYZ, uint32_t> sicp;
        LOG(INFO) << "----------------------------------";
        LOG(INFO) << "---------     cloudS   -----------";
        LOG(INFO) << "--------cloudS size :"<< cloudS->size()<<"---------";
        sicp.pcl_2_semantic(cloudS, semiS);
        LOG(INFO) << "----------------------------------";
        LOG(INFO) << "---------     cloudT   -----------";
        LOG(INFO) << "--------cloudT size :"<< cloudT->size()<<"---------";
        sicp.pcl_2_semantic(cloudT, semiT);

        Eigen::Matrix4d temp = Eigen::Matrix4d::Identity();
        temp(1, 1) = 1;
        temp(0, 0) = 1;
        temp(2, 2) = 1;
        Eigen::Vector3d t(0, 10, 0);
        //Bootstrap boot(cloudA, cloudB);
        //Eigen::Matrix4d temp = (boot.align()).cast<double>();
        Sophus::SE3d initTransform(temp);
        // semanticicp::SemanticIterativeClosestPoint<pcl::PointXYZ, uint32_t> sicp;
        sicp.setInputSource(semiS);
        sicp.setInputTarget(semiT);

        LOG(INFO) << "*********************************";
        LOG(INFO) << "---------     cloudS   ----------";
        LOG(INFO) << "*********************************";
        sicp.align(semiS, initTransform);
        Sophus::SE3d sicpTranform = sicp.getFinalTransFormation();
        LOG(INFO) << "Final SICP Transform\n"
                  << sicpTranform.matrix() << std::endl;

        // pcl::PointCloud<pcl::PointXYZL>::Ptr cloudASICP(new pcl::PointCloud<pcl::PointXYZL>());

        pcl::transformPointCloud(*cloudS, *finalPc, (sicpTranform.matrix()).cast<float>());

        // std::cout << "SICP Accuracy "
        //   << semanticICPMetrics.evaluate(finalPc, cloudT, numSource) << std::endl;
    }
    else if (bicp == 3)
    {
        // em icp
        LOG(INFO) << "icp start ...";

        pcl::PointCloud<pcl::PointXYZ>::Ptr cloudSnoL (new pcl::PointCloud<pcl::PointXYZ>());
        pcl::PointCloud<pcl::PointXYZ>::Ptr cloudTnoL (new pcl::PointCloud<pcl::PointXYZ>());
        pcl::PointCloud<pcl::PointXYZ>::Ptr cloudRes (new pcl::PointCloud<pcl::PointXYZ>());

        for(size_t t = 0; t< cloudS->points.size(); t++){
            pcl::PointXYZL p = cloudS->points[t];
            // p.label=0;
            pcl::PointXYZ pt;
            pt.x = p.x;
            pt.y = p.y;
            pt.z = p.z;
            cloudSnoL->push_back(pt);
        }
        for(size_t t = 0; t< cloudT->points.size(); t++){
            pcl::PointXYZL p = cloudT->points[t];
            // p.label=0;
            pcl::PointXYZ pt;
            pt.x = p.x;
            pt.y = p.y;
            pt.z = p.z;
            cloudTnoL->push_back(pt);
        }
        // cloudS.reset();

        pcl::IterativeClosestPoint<pcl::PointXYZL, pcl::PointXYZL> icp2; //创建ICP对象，用于ICP配准
        icp2.setMaximumIterations(200);
        icp2.setInputCloud(cloudS);   //设置输入点云
        icp2.setInputTarget(cloudT); //设置目标点云（输入点云进行仿射变换，得到目标点云）
        icp2.align(*finalPc);       //匹配后源点云
    }
    else
    {
        LOG(WARNING) << "warning switch ...";
    }

    if (finalPc != nullptr)
    {
        LOG(INFO) << "align result: " << finalPc->size();
    }
    else
    {
        return 0;
    }

    // // em icp done
    success = true;

    bool bview = true;
    if (bview)
    {
        boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer1(new pcl::visualization::PCLVisualizer("init windows"));
        boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer2(new pcl::visualization::PCLVisualizer("semi icp result windows"));
        boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer3(new pcl::visualization::PCLVisualizer("result windows"));

        pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZL> rb(cloudS, 0, 0, 255);
        pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZL> rg(cloudT, 255, 0, 0);
        pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZL> rr(finalPc, 0, 255, 0);

        viewer1->removePointCloud("cloud1");
        viewer1->addPointCloud<pcl::PointXYZL>(cloudS, rg, "cloud1");
        viewer1->addPointCloud<pcl::PointXYZL>(cloudT, rr, "cloud2");
        viewer1->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 2, "cloud1");

        viewer2->removePointCloud("cloud3");
        viewer2->addPointCloud<pcl::PointXYZL>(cloudT, rr, "cloud3");
        viewer2->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 2, "cloud3");

        if (success)
        {
            viewer2->addPointCloud<pcl::PointXYZL>(finalPc, rg, "cloud4");
            viewer2->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 2, "cloud4");
        }

        viewer3->removePointCloud("cloud3");
        viewer3->addPointCloud<pcl::PointXYZL>(finalPc, rr, "cloud3");
        viewer3->addPointCloud<pcl::PointXYZL>(cloudS, rg, "cloud3");
        viewer3->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 2, "cloud3");


        viewer1->spin();
        viewer2->spin();
        // viewer3->spin();
    }

    google::ShutdownGoogleLogging();

    return 0;
}