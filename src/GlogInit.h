/*
 * @Author: your name
 * @Date: 2020-10-24 10:46:17
 * @LastEditTime: 2020-10-31 12:21:54
 * @LastEditors: Please set LastEditors
 * @Description: In User Settings Edit
 * @FilePath: /semICP/GlogInit.cpp
 */
#include <glog/logging.h>
#include <boost/filesystem.hpp>

#include "PointType.hpp"
#include <pcl/io/pcd_io.h>


void logInit(const std::string &project)
{
    google::InitGoogleLogging(project.c_str());
    std::string logMainDir = project;
    LOG(INFO) << "log dir :" << logMainDir;
    boost::filesystem::path logPath(logMainDir);
    if ( !boost::filesystem::exists(logMainDir))
    {
            boost::filesystem::create_directory(logMainDir);
    }
    google::SetLogDestination(google::GLOG_INFO, (logMainDir + "/INFO").c_str());
    google::SetLogDestination(google::GLOG_WARNING, (logMainDir + "/WARNING").c_str());
    google::SetLogDestination(google::GLOG_ERROR, (logMainDir + "/ERROR").c_str());

    google::SetStderrLogging(google::INFO);
}

void test()
{
    std::string dir1 = "/home/hy/Ecarx_ws/HY520/semICP/data/xyzlc/pcpImg.pcd";
    std::string dir2 = "/home/hy/Ecarx_ws/HY520/semICP/data/xyzlc/projMap.pcd";
    pcl::PointCloud<PointXYZLC>::Ptr pcS(new pcl::PointCloud<PointXYZLC>);
    pcl::PointCloud<PointXYZLC>::Ptr pcT(new pcl::PointCloud<PointXYZLC>);

    if (pcl::io::loadPCDFile<PointXYZLC>(dir1, *pcS) == -1) //* load the file
    {
        PCL_ERROR ("Couldn't read source file\n");
    }
    if (pcl::io::loadPCDFile<PointXYZLC> (dir2, *pcT) == -1) //* load the file
    {
        PCL_ERROR ("Couldn't read source file\n");
    }

    for(size_t i = 0; i < pcS->points.size(); i++)
    {
        PointXYZLC pt = pcS->points[i];
        LOG(INFO) << "pcp: " << pt.x << " " << pt.y << " " << pt.z << " " << int(pt.label) << " " << pt.owner;
    }

    for(size_t i = 0; i < pcT->points.size(); i++)
    {
        PointXYZLC pt = pcT->points[i];
        LOG(INFO) << "map: " << pt.x << " " << pt.y << " " << pt.z << " " << int(pt.label) << " " << pt.owner;
    }
}