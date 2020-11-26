/*
 * @Author: your name
 * @Date: 2020-11-03 15:07:20
 * @LastEditTime: 2020-11-03 15:07:21
 * @LastEditors: your name
 * @Description: In User Settings Edit
 * @FilePath: /semantic_location_self/src/semantic_loc/include/PointCloudHelper.hpp
 */
#pragma once

#include <map>
#include <pcl/point_types.h>
#include <pcl/visualization/pcl_visualizer.h>

// namespace ns_helper
// {



template<typename PointT>
class PointCloudHelper
{
public:
    typedef typename pcl::PointCloud<PointT> CloudT;
    typedef typename CloudT::Ptr CloudTPtr;
    typedef typename CloudT::ConstPtr CloudTConstPtr;


public:
    static void showPointCloud(CloudTConstPtr cloud, const std::string& title = "3D viewer")
    {
        pcl::visualization::PCLVisualizer::Ptr viewer(new pcl::visualization::PCLVisualizer(title));
        pcl::visualization::PointCloudColorHandlerCustom<PointT> cloud_rgb(cloud, 255, 0, 0);
        viewer->addPointCloud<PointT>(cloud, cloud_rgb, "cloud");

        viewer->spin();
    }

    static void showPointCloudPair(CloudTConstPtr src, CloudTConstPtr tgt, const std::string& title = "3D viewer")
    {
        pcl::visualization::PCLVisualizer::Ptr viewer(new pcl::visualization::PCLVisualizer(title));
        pcl::visualization::PointCloudColorHandlerCustom<PointT> src_rgb(src, 255, 0, 0);
        pcl::visualization::PointCloudColorHandlerCustom<PointT> tgt_rgb(tgt, 0, 255, 0);
        viewer->addPointCloud<PointT>(src, src_rgb, "src");
        viewer->addPointCloud<PointT>(tgt, tgt_rgb, "tgt");

        viewer->spin();
    }

    static void ShowPointCloudVector(std::vector<CloudTConstPtr>& pcs, bool colorful = true, const std::string& title = "3D viewer")
    {
        pcl::visualization::PCLVisualizer::Ptr viewer(new pcl::visualization::PCLVisualizer(title));
        double r = std::rand()%256 / 255.0;
        double g = std::rand()%256 / 255.0;
        double b = std::rand()%256 / 255.0;

        for(size_t i = 0; i < pcs.size(); ++i)
        {            
            if(colorful)
            {
                r = std::rand()%256 / 255.0;
                g = std::rand()%256 / 255.0;
                b = std::rand()%256 / 255.0;
            }
            
            std::string label = "pc-" + std::to_string(i);
            pcl::visualization::PointCloudColorHandlerCustom<PointT> src_rgb(pcs[i], r, g, b);
            viewer->addPointCloud<PointT>(pcs[i], src_rgb, label);
        }

        viewer->spin();
    }

    static void ShowPointCloudLines(std::vector<CloudTConstPtr>& lines, bool colorful = true, const std::string& title = "3D viewer")
    {
        pcl::visualization::PCLVisualizer::Ptr viewer(new pcl::visualization::PCLVisualizer(title));
        double r = std::rand()%256 / 255.0;
        double g = std::rand()%256 / 255.0;
        double b = std::rand()%256 / 255.0;

        for(size_t i = 0; i < lines.size(); ++i)
        {            
            if(colorful)
            {
                r = std::rand()%256 / 255.0;
                g = std::rand()%256 / 255.0;
                b = std::rand()%256 / 255.0;
            }
            
            CloudTConstPtr a_line = lines[i];
            for(size_t j = 1; j < a_line->points.size(); j=j+2)
            {
                std::string label = "line-" + std::to_string(i) + "-" + std::to_string(j);
                viewer->addLine<PointT>(a_line->points[j-1], a_line->points[j], r, g, b, label);
                viewer->setShapeRenderingProperties(pcl::visualization::PCL_VISUALIZER_LINE_WIDTH, 2, label);
                viewer->setShapeRenderingProperties(pcl::visualization::PCL_VISUALIZER_OPACITY, 0.9, label);
            }
        }

        // viewer->spinOnce();
        viewer->spin();
    }

};


// }   // namespace