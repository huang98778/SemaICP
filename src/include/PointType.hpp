/*
 * @Author: your name
 * @Date: 2020-10-31 11:10:56
 * @LastEditTime: 2020-10-31 15:52:57
 * @LastEditors: Please set LastEditors
 * @Description: In User Settings Edit
 * @FilePath: /semICP/PointType.hpp
 */
#ifndef _POINT_TYPES_
#define _POINT_TYPES_

// #define PCL_NO_PRECOMPILE

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <boost/graph/graph_concepts.hpp>


struct PointXYZLC
{
    PCL_ADD_POINT4D;
    uint8_t label;
    uint16_t owner;
    inline PointXYZLC(float x_, float y_, float z_, uint8_t label_, uint16_t owner_)
    {
        x = x_; y = y_; z = z_; data[3] = 1.0f;
        
        label = label_;
        owner = owner_;
    }

    inline PointXYZLC ()
    {
        x = y = z = 0.0f;
        data[3] = 1.0f;

        label = 0;
        owner = 0;
    }

    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
}EIGEN_ALIGN16;

struct PointXYZC
{
    PCL_ADD_POINT4D;
    // uint8_t label;
    uint16_t owner;
    inline PointXYZC(float x_, float y_, float z_,  uint16_t owner_)
    {
        x = x_; y = y_; z = z_; data[3] = 1.0f;
        
        // label = label_;
        owner = owner_;
    }

    inline PointXYZC ()
    {
        x = y = z = 0.0f;
        data[3] = 1.0f;

        // label = 0;
        owner = 0;
    }

    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
}EIGEN_ALIGN16;

POINT_CLOUD_REGISTER_POINT_STRUCT (PointXYZLC,
                                    (float, x, x)
                                    (float, y, y)
                                    (float, z, z)
                                    (uint8_t, label, label)
                                    (uint16_t, owner, owner)
)

POINT_CLOUD_REGISTER_POINT_STRUCT (PointXYZC,
                                    (float, x, x)
                                    (float, y, y)
                                    (float, z, z)
                                    // (uint8_t, label, label)
                                    (uint16_t, owner, owner)
)
#endif