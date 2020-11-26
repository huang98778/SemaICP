#include "TrajectoryFusion.h"
#include <algorithm>

TrajectoryFusion::TrajectoryFusion(OdomParser &refParser, std::vector<std::string> &jsonFiles)
{
    if (!initialLocalFrame(refParser))
        return;

    extractKeyFramesFromDataParser(refParser, m_referenceFrames);
    assignJsonFileToKeyFrames(m_referenceFrames, jsonFiles);
    // showOdometry(m_referenceFrames);
}

TrajectoryFusion::~TrajectoryFusion(){};

bool TrajectoryFusion::initialLocalFrame(OdomParser &refParser)
{
    if (refParser.odom_out_.empty())
    {
        LOG(ERROR) << "initialLocalFrame: odomParser is empty!";
        return false;
    }

    double lon = refParser.odom_out_[0].lon * DEG2RAD_;
    double lat = refParser.odom_out_[0].lat * DEG2RAD_;
    double height = refParser.odom_out_[0].height;
    m_localFrame.SetRefPoint(lon, lat, height);

    return true;
}

void TrajectoryFusion::extractKeyFramesFromDataParser(OdomParser &odomParser, KeyFramePtrs &keyFrames)
{
    keyFrames.clear();
    for (int i = 0; i < odomParser.odom_out_.size(); ++i)
    {
        KeyFrame kf;
        kf.timestamp = odomParser.odom_out_[i].timestamp;
        kf.pose = odomParser.odom_out_[i].pose;

        keyFrames.push_back(std::make_shared<KeyFrame>(kf));
    }
}

void TrajectoryFusion::extractTimestampsFromJsonFiles(std::vector<std::string> &jsonFiles, std::vector<double> &timestamps)
{
    timestamps.clear();
    timestamps.resize(jsonFiles.size());
    for (int i = 0; i < jsonFiles.size(); ++i)
    {
        std::string basename = fs::basename(jsonFiles[i]);
        timestamps[i] = Utility::strDayTimeToStamp(basename);
    }
}

int TrajectoryFusion::getCorrespondingIdx(double queryTimestamp, std::vector<double> &timestamps, int startId)
{
    for (int i = startId; i < timestamps.size(); ++i)
    {
        if (fabs(timestamps[i] - queryTimestamp) <= 0.005)
            return i;
    }
    return -1;
}

void TrajectoryFusion::assignJsonFileToKeyFrames(KeyFramePtrs &keyFrames, std::vector<std::string> &jsonFiles)
{
    std::vector<double> timestamps;
    extractTimestampsFromJsonFiles(jsonFiles, timestamps);

    for (int i = 0; i < keyFrames.size(); ++i)
    {
        int idx = getCorrespondingIdx(keyFrames[i]->timestamp, timestamps);
        if (idx < 0)
            continue;
        keyFrames[i]->jsonFile = jsonFiles[idx];
    }
}


bool TrajectoryFusion::getSemanticSubMap(int refId, double minDist, 
                                        std::vector<std::pair<std::string, pcl::PointCloud<pcl::PointXYZ>::ConstPtr>> &pcs)
{
    

    for (int i = refId; i < m_referenceFrames.size(); ++i)
    {
        if (m_referenceFrames[i]->jsonFile.empty())
            continue;
        double distance = (m_referenceFrames[i]->pose.translation() - m_referenceFrames[refId]->pose.translation()).norm();
        if (distance < minDist)
        {
            Eigen::Isometry3d transform = m_referenceFrames[refId]->pose.inverse() * m_referenceFrames[i]->pose;
            Eigen::Matrix4f trans = transform.matrix().cast<float>();
            PerceptionParser pp;
            pp.setTransformation(trans);
            pp.getPointCloud(m_referenceFrames[i]->jsonFile, pcs);
        }
        else
        {
            break;
        }
    }

    for (int i = refId - 1; i >= 0; --i)
    {
        if (m_referenceFrames[i]->jsonFile.empty())
            continue;
        double distance = (m_referenceFrames[i]->pose.translation() - m_referenceFrames[refId]->pose.translation()).norm();
        if (distance < minDist)
        {
            Eigen::Isometry3d transform = m_referenceFrames[refId]->pose.inverse() * m_referenceFrames[i]->pose;
            Eigen::Matrix4f trans = transform.matrix().cast<float>();
            PerceptionParser pp;
            pp.setTransformation(trans);
            pp.getPointCloud(m_referenceFrames[i]->jsonFile, pcs);
        }
        else
        {
            break;
        }
    }

    std::cout << "pcs size: " << pcs.size() << std::endl;

    // showPointCloud(pcs, true, "submap", true);

    return true;
}


void TrajectoryFusion::setActiveFramesInfo(OdomParser &activeParser, std::vector<std::string> &jsonFiles)
{
    m_activeOdomParser = activeParser;

    extractKeyFramesFromDataParser(activeParser, m_activeFrames);
    assignJsonFileToKeyFrames(m_activeFrames, jsonFiles);
    // showOdometry(m_activeFrames);
}

bool TrajectoryFusion::runTrajectoryFusionOptimization()
{
    if (m_referenceFrames.empty())
    {
        LOG(ERROR) << "runTrajectoryFusionOptimization: m_referenceFrames is empty!";
        return false;
    }

    if (m_activeFrames.empty())
    {
        LOG(ERROR) << "runTrajectoryFusionOptimization: m_activeFrames is empty!";
        return false;
    }

    buildGeometryConstraints();

    return true;
}

bool TrajectoryFusion::buildGeometryConstraints()
{
    return true;
}

void TrajectoryFusion::showOdometry(KeyFramePtrs &keyframes, size_t interval, double scale)
{
    pcl::PointCloud<pcl::PointXYZ>::Ptr orginal(new pcl::PointCloud<pcl::PointXYZ>);
    orginal->points.resize(keyframes.size());
    for (int i = 0; i < keyframes.size(); ++i)
    {
        orginal->points[i].x = keyframes[i]->pose.translation()[0];
        orginal->points[i].y = keyframes[i]->pose.translation()[1];
        orginal->points[i].z = keyframes[i]->pose.translation()[2];
    }

    pcl::visualization::PCLVisualizer odomViewer("odometry viewer");
    odomViewer.addPointCloud(orginal, "orginal");

    orginal->width = orginal->points.size();
    orginal->height = 1;
    // pcl::io::savePCDFileASCII("odometry.pcd", *orginal);

    for (int i = 0; i < keyframes.size(); i += interval)
    {
        Eigen::Vector3d x = keyframes[i]->pose * (scale * Eigen::Vector3d(1, 0, 0));
        Eigen::Vector3d y = keyframes[i]->pose * (scale * Eigen::Vector3d(0, 1, 0));
        Eigen::Vector3d z = keyframes[i]->pose * (scale * Eigen::Vector3d(0, 0, 1));

        pcl::PointXYZ px, py, pz;
        px.x = x(0);
        px.y = x(1);
        px.z = x(2);
        py.x = y(0);
        py.y = y(1);
        py.z = y(2);
        pz.x = z(0);
        pz.y = z(1);
        pz.z = z(2);

        std::stringstream ss;
        ss << "line_" << i;
        std::string l1(ss.str()), l2(ss.str()), l3(ss.str());
        l1.push_back('x');
        l2.push_back('y');
        l3.push_back('z');
        odomViewer.addLine(orginal->points[i], px, 1, 0, 0, l1);
        odomViewer.addLine(orginal->points[i], py, 0, 1, 0, l2);
        odomViewer.addLine(orginal->points[i], pz, 0, 0, 1, l3);
    }

    odomViewer.spin();
}

void TrajectoryFusion::showPointCloud(std::vector<std::pair<std::string, pcl::PointCloud<pcl::PointXYZ>::ConstPtr>> &pcs,
                                      bool colorful, const std::string &title, bool isStop)
{
    pcl::visualization::PCLVisualizer::Ptr viewer(new pcl::visualization::PCLVisualizer(title));

    Eigen::Vector3i rgb(255, 255, 255);
    for (size_t i = 0; i < pcs.size(); ++i)
    {
        if (colorful)
        {
            if (pcs[i].first == "AD_LaneDivider")
                rgb << 255, 0, 255; //深红
            else if (pcs[i].first == "AD_DashLine")
                rgb << 0, 255, 0; //绿色
            else if (pcs[i].first == "AD_LaneMark")
                rgb << 255, 255, 0; //黄色
            else if (pcs[i].first == "AD_Arrow")
                rgb << 255, 0, 0; //红色
        }
        // std::cout << "object type: " << pcs[i].first << std::endl;
        std::string label = "pc-" + std::to_string(i);
        pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> src_rgb(pcs[i].second, rgb(0), rgb(1), rgb(2));
        viewer->addPointCloud<pcl::PointXYZ>(pcs[i].second, src_rgb, label);
        viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 2, label);
    }
    if (isStop)
        viewer->spin();
    else
        viewer->spinOnce();
}
