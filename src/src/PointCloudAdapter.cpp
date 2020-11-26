/*
 * @Author: your name
 * @Date: 2020-10-30 10:25:25
 * @LastEditTime: 2020-11-04 10:37:49
 * @LastEditors: Please set LastEditors
 * @Description: In User Settings Edit
 * @FilePath: /semICP/PointCloudAdapter.cpp
 */
#include "PointCloudAdapter.h"
#include <glog/logging.h>

#include <pcl/kdtree/kdtree_flann.h>

PointCloudAdapter::PointCloudAdapter(/* args */)
{
}

PointCloudAdapter::~PointCloudAdapter()
{
}

void PointCloudAdapter::pcl2object(const pcl::PointCloud<pcl::PointXYZRGBL>::Ptr pclCloud,
                                   std::map<uint8_t, pcl::PointCloud<pcl::PointXYZL>::Ptr> &objcloud)
{
    typedef pcl::PointXYZL PointT;
    typedef pcl::PointCloud<pcl::PointXYZL> PointCloud;
    typedef typename PointCloud::Ptr PointCloudPtr;

    std::vector<uint8_t> objs;
    std::map<uint8_t, PointCloudPtr> map;

    for (pcl::PointXYZRGBL p : pclCloud->points)
    {
        if (map.find(p.r) == map.end())
        {
            PointCloudPtr cloud(new PointCloud());
            pcl::PointXYZL pt;
            pt.x = p.x;
            pt.y = p.y;
            pt.label = p.label;
            cloud->push_back(pt);

            map[p.r] = cloud;
            objs.push_back(p.r);
        }
        else
        {
            PointCloudPtr cloud = map[p.r];
            pcl::PointXYZL pt;
            pt.x = p.x;
            pt.y = p.y;
            pt.label = p.label;
            cloud->push_back(pt);
        }
        // LOG(INFO) << "point xyz: " << p.x << p.y << p.z;
    }

    // objcloud = map;
    swap(objcloud, map);
}

// template <typename PointT>
void PointCloudAdapter::decentralize(typename pcl::PointCloud<pcl::PointXYZRGBL>::Ptr &pc)
{
    double sumx, sumy, sumz;
    for (int i = 0; i < pc->points.size(); i++)
    {
        sumx += pc->points[i].x;
        sumy += pc->points[i].y;
        // sumz += pc->points[i].z;
    }
    // LOG(INFO) << "PC  " << sumx << " " << sumy << " " << sumz;

    for (pcl::PointXYZRGBL &pt : pc->points)
    {
        pt.x = pt.x -  sumx / pc->points.size(); // 960; //
        pt.y = pt.y -  sumy / pc->points.size(); // 540; //
        pt.z = 0;
    }
}

void PointCloudAdapter::decentralize(pcl::PointCloud<pcl::PointXYZL>::Ptr &pc,
                                     pcl::PointCloud<pcl::PointXYZL>::Ptr &output)
{
    double sumx, sumy, sumz;
    for (int i = 0; i < pc->points.size(); i++)
    {
        sumx += pc->points[i].x;
        sumy += pc->points[i].y;
        // sumz += pc->points[i].z;
    }
    // LOG(INFO) << "PC  " << sumx << " " << sumy << " " << sumz;

    for (int i = 0; i < pc->points.size(); i++)
    {
        pcl::PointXYZL pt;
        pt.x = pc->points[i].x - 960; // sumx / pc->points.size();
        pt.y = pc->points[i].y - 540; // sumy / pc->points.size();
        pt.z = 0;
        pt.label = pc->points[i].label;

        output->push_back(pt);

        LOG(INFO) << "PC befor: "
                  << pc->points[i].label << " "
                  << pc->points[i].x << " "
                  << pc->points[i].y << " "
                  << pc->points[i].z;
    }
}

template <typename PointT>
bool PointCloudAdapter::loadPointCloudT(const std::string &dir, typename pcl::PointCloud<PointT>::Ptr &pc)
{
    if (pcl::io::loadPCDFile<PointT>(dir, *pc) == -1) //* load the file
    {
        PCL_ERROR("Couldn't read source file\n");
        return false;
    }

    // for(size_t i = 0; i < pcT->points.size(); i++)
    // {
    //     PointXYZLC pt = pcT->points[i];
    //     LOG(INFO) << "map: " << pt.x << " " << pt.y << " " << pt.z << " " << int(pt.label) << " " << pt.owner;
    // }

    return true;
}

bool PointCloudAdapter::loadPointCloud(const std::string &dir, pcl::PointCloud<pcl::PointXYZRGBL>::Ptr &pc)
{
    if (pcl::io::loadPCDFile<pcl::PointXYZRGBL>(dir, *pc) == -1) //* load the file
    {
        PCL_ERROR("Couldn't read source file\n");
        return false;
    }

    // for(size_t i = 0; i < pcT->points.size(); i++)
    // {
    //     PointXYZLC pt = pcT->points[i];
    //     LOG(INFO) << "map: " << pt.x << " " << pt.y << " " << pt.z << " " << int(pt.label) << " " << pt.owner;
    // }

    return true;
}

bool PointCloudAdapter::getSameObjPointCloud(const pcl::PointCloud<pcl::PointXYZL>::Ptr &cloud,
                                             const std::shared_ptr<SemanticICP::SemanticPointCloud<pcl::PointXYZRGB, uint32_t>> target,
                                             pcl::PointCloud<pcl::PointXYZL>::Ptr &resCloud)
{
    typedef pcl::KdTreeFLANN<pcl::PointXYZRGB> KdTree;
    typedef typename KdTree::Ptr KdTreePtr;

    for (size_t i = 0; i < cloud->size(); ++i)
    {
        pcl::PointXYZL pt = cloud->points[i];
        pcl::PointXYZRGB ptt;
        ptt.x = pt.x;
        ptt.y = pt.y;
        ptt.z = pt.z;
        ptt.r = ptt.g = ptt.b = 0;

        if (target->labeledPointClouds.find(pt.label) != target->labeledPointClouds.end())
        {
            KdTreePtr tree = target->labeledKdTrees[pt.label];

            std::vector<int> targetIndx;
            std::vector<float> distSq;
            tree->nearestKSearch(ptt, 1, targetIndx, distSq);

            std::cout << "label: " << pt.label << " indx: " << targetIndx[0] << " dis: " << distSq[0]<<std::endl;

            const pcl::PointXYZRGB  targetPoint = (target->labeledPointClouds[pt.label])->points[targetIndx[0]];

            pcl::PointXYZL tpoint;
            tpoint.x = targetPoint.x;
            tpoint.y = targetPoint.y;
            tpoint.z = targetPoint.z;
            tpoint.label = pt.label;
            resCloud->points.push_back(tpoint);                    
        }                

    }

}

// void PointCloudAdapter::interpolate(std::vector<std::pair<std::string, pcl::PointCloud<pcl::PointXYZ>::Ptr>> &pcs)
// {
//     for (size_t i = 0; i < pcs.size(); i++)
//     {
//         std::pair<std::string, pcl::PointCloud<pcl::PointXYZ>::Ptr> ptrPc = pcs[i];
//         pcl::PointXYZL pt;
//         if (ptrPc.first == "AD_TrafficLight")
//         {
//             for (auto p : ptrPc.second->points)
//             {
//                 pt.x = p.x;
//                 pt.y = p.y;
//                 pt.z = p.z;
//                 pt.label = _TrafficLight;
//                 cloud.points.push_back(pt);
//             }

//         }
//         if (ptrPc.first == "AD_TrafficSign")
//         {
//             for (auto p :  ptrPc.second->points)
//             {
//                 pt.x = p.x;
//                 pt.y = p.y;
//                 pt.z = p.z;
//                 pt.label = _TrafficSign;
//                 cloud.points.push_back(pt);
//             }

//         }
//         // if (ptrPc.first == "AD_DashLine")
//         // {
//         //     for (auto p :  ptrPc.second->points)
//         //     {
//         //         pt.x = p.x;
//         //         pt.y = p.y;
//         //         pt.z = p.z;
//         //         pt.label = _DashLine;
//         //         cloud.points.push_back(pt);
//         //     }
//         // }
//         // if (ptrPc.first == "AD_LaneMark")
//         // {
//         //     for (auto p :  ptrPc.second->points)
//         //     {
//         //         pt.x = p.x;
//         //         pt.y = p.y;
//         //         pt.z = p.z;
//         //         pt.label = _LaneMark;
//         //         cloud.points.push_back(pt);
//         //     }
//         // }
//         // if (ptrPc.first == "AD_LaneDivider")
//         // {
//         //     for (auto p :  ptrPc.second->points)
//         //     {
//         //         pt.x = p.x;
//         //         pt.y = p.y;
//         //         pt.z = p.z;
//         //         pt.label = _LaneDivider;
//         //         cloud.points.push_back(pt);
//         //     }
//         // }

//     }
// }
