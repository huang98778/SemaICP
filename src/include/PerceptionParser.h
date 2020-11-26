/*
 * @Author: your name
 * @Date: 2020-10-22 14:50:28
 * @LastEditTime: 2020-10-30 18:38:43
 * @LastEditors: Please set LastEditors
 * @Description: In User Settings Edit
 * @FilePath: /semICP/PerceptionParser.h
 */
#pragma once
#include <vector>
#include <string>
#include <iostream>
#include <fstream>
#include <glog/logging.h>
#include <eigen3/Eigen/Core>
#include "nlohmann/json.hpp"
#include "dr_utility.h"

#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl/common/transforms.h>
#include <pcl/visualization/pcl_visualizer.h>

typedef struct tag_SemPoint
{
    double x;
    double y;
    double z;
} SemPoint;

typedef struct tag_SingleSemData
{
    std::string obj_type;
    long long obj_id;
    int type;
    double confidence;
    int measure_state;
    std::vector<SemPoint> points;
} SingleSemData;

typedef struct tag_FusionSemData
{

} FusionSemData;

typedef struct tag_FR60
{
    std::vector<SingleSemData> single_sem_data;
    std::vector<FusionSemData> fusion_sem_data;
} FR60;

typedef struct tag_PerceptionData
{
    double time_stamp_cam_right;
    FR60 fr_60;

} PerceptionData;

class PerceptionParser
{
public:
    PerceptionParser();
    ~PerceptionParser();

    inline void setTransformation(Eigen::Matrix4f &transformation)
    {
        m_transform = transformation;
    }

    bool load(std::string &fn);
    PerceptionData getData();
    bool getPointCloud(std::string jsonFile, std::vector<std::pair<std::string, pcl::PointCloud<pcl::PointXYZ>::ConstPtr>> &pcs);

    void convertVec2Pc(std::vector<std::pair<std::string, pcl::PointCloud<pcl::PointXYZ>::ConstPtr>> &pcs, pcl::PointCloud<pcl::PointXYZL> &cloud);

private:
    nlohmann::json input_json_;
    PerceptionData data_;
    Eigen::Matrix4f m_transform;
};