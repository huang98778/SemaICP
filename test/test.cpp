/*
 * @Author: your name
 * @Date: 2020-10-31 18:21:47
 * @LastEditTime: 2020-10-31 19:28:26
 * @LastEditors: Please set LastEditors
 * @Description: In User Settings EditP
 * @FilePath: /semICP/test/test.cpp
 */

#include "../PointType.hpp"
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/point_types.h>
#include <iostream>
#include <vector>
// #include <algorithm>
#include <memory>
// #include <map>

#include <pcl/point_cloud.h>


int main()
{
    typedef pcl::PointXYZRGB PointT;
    typedef pcl::KdTreeFLANN<PointT> KdTree;
    typedef typename KdTree::Ptr KdTreePtr;

    pcl::PointCloud<PointT>::Ptr pc(new pcl::PointCloud<PointT>);
    for (int i = 0; i < 100; i++)
    {
        PointT pt;

        pt.x = i;
        pt.y = 2*i;
        pt.z = 0;

        pt.r = 1002;
        // pt.owner = 1000;
        // pt.label = 19;

        pc->push_back(pt);
    }

    KdTreePtr tree(new KdTree());
    tree->setInputCloud(pc);
    PointT pt;
    pt.x = 20;
    pt.y = 40;
    pt.z = 0;

    std::vector<int> ind;
    std::vector<float> dis;

    tree->nearestKSearch(pt, 1, ind, dis);

    std::cout << " size: " << ind.size() <<"dis: "<<dis[0] << std::endl;

}