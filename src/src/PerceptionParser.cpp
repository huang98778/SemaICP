#include "PerceptionParser.h"
#include "Def.h"

PerceptionParser::PerceptionParser()
{
    m_transform = Eigen::Matrix4f::Identity();
}

PerceptionParser::~PerceptionParser()
{

}

bool PerceptionParser::load(std::string &fn)
{
    std::ifstream fs(fn);
    if(!fs)
    {
        std::cout << "fail to load: " << fn;  
        return false;
    }

    fs >> input_json_;
    data_.time_stamp_cam_right = input_json_["time_stamp_cam_right"].get<double>();
    // std::cout << "time_stamp_cam_right" << data_.time_stamp_cam_right << std::endl;

    for(auto it = input_json_["fr_60"]["single_sem_data"].begin(); it != input_json_["fr_60"]["single_sem_data"].end(); ++it)
    {

        SingleSemData single_sem;
        single_sem.obj_type = (*it)["obj_type"].get<std::string>();
        // std::cout << "single_sem.obj_type " << single_sem.obj_type << std::endl;
        // single_sem.type = (*it)["type"].get<int>();
        // std::cout << "single_sem.type " << single_sem.type << std::endl;
        single_sem.confidence = (*it)["confidence"].get<double>();
        // std::cout << "single_sem.confidence " << single_sem.confidence << std::endl;
        // single_sem.obj_id = (*it)["obj_id"].get<std::string>();

        if(single_sem.obj_type != "AD_LaneDivider")
        {
            single_sem.measure_state = (*it)["measure_state"].get<int>();
            if(single_sem.measure_state != 2)
                continue;
        }

        std::string str_pts = (*it)["points"].get<std::string>();
        std::string str_pts2 = str_pts.substr(1, str_pts.size()-2);

        std::vector<std::string> v_str;
        Utility::splitString(v_str, str_pts2, ",");
        for(auto st : v_str)
        {
            // std::cout << st << std::endl;

            if(single_sem.obj_type != "AD_LaneDivider")
            {
                st = st.substr(1, st.size()-2);
            }
            std::vector<std::string> v_str2;
            Utility::splitString(v_str2, st, " ");
            SemPoint pt;
            pt.x = atof(v_str2[0].c_str());
            pt.y = atof(v_str2[1].c_str());
            pt.z = atof(v_str2[2].c_str());
            single_sem.points.push_back(pt);
            // std::cout << "xyz " << pt.x << " " << pt.y << " " << pt.z << std::endl;
        } 

        data_.fr_60.single_sem_data.push_back(single_sem);
    }
    return true;
}

PerceptionData PerceptionParser::getData()
{
    return data_;
}


bool PerceptionParser::getPointCloud(std::string jsonFile, 
                                     std::vector<std::pair<std::string, pcl::PointCloud<pcl::PointXYZ>::ConstPtr>> &pcs)
{
    bool ret = load(jsonFile);
    if(!ret)
    {
        LOG(ERROR) << "getPointCloud: load " << jsonFile << " failed";
        return false;
    }
    // LOG(INFO) << jsonFile;

    for(auto sem : data_.fr_60.single_sem_data)
    {
        pcl::PointCloud<pcl::PointXYZ> pc;
        for(auto pt : sem.points)
            pc.points.emplace_back(pt.x, pt.y, pt.z);
        // pcl::PointCloud<pcl::PointXYZ> cloud;
        // pcl::transformPointCloud(pc, cloud, m_transform);    
        pcs.push_back(std::make_pair(sem.obj_type, pc.makeShared()));
    }

    return true;
}

void PerceptionParser::convertVec2Pc(std::vector<std::pair<std::string, pcl::PointCloud<pcl::PointXYZ>::ConstPtr>> &pcs, pcl::PointCloud<pcl::PointXYZL> &cloud)
{
    for (size_t i = 0; i < pcs.size(); i++)
    {
        std::pair<std::string, pcl::PointCloud<pcl::PointXYZ>::ConstPtr> ptrPc = pcs[i];
        pcl::PointXYZL pt;
        if (ptrPc.first == "AD_TrafficLight")
        {
            for (auto p : ptrPc.second->points)
            {                
                pt.x = p.x;
                pt.y = p.y;
                pt.z = p.z;
                pt.label = _TrafficLight;
                cloud.points.push_back(pt);
            }
            
        }
        if (ptrPc.first == "AD_TrafficSign")
        {
            for (auto p :  ptrPc.second->points)
            {
                pt.x = p.x;
                pt.y = p.y;
                pt.z = p.z;
                pt.label = _TrafficSign;
                cloud.points.push_back(pt);
            }
            
        } 
        if (ptrPc.first == "AD_DashLine")
        {
            for (auto p :  ptrPc.second->points)
            {
                pt.x = p.x;
                pt.y = p.y;
                pt.z = p.z;
                pt.label = _DashLine;
                cloud.points.push_back(pt);
            }
        }
        if (ptrPc.first == "AD_LaneMark")
        {
            for (auto p :  ptrPc.second->points)
            {
                pt.x = p.x;
                pt.y = p.y;
                pt.z = p.z;
                pt.label = _LaneMark;
                cloud.points.push_back(pt);
            }
        }
        if (ptrPc.first == "AD_LaneDivider")
        {
            for (auto p :  ptrPc.second->points)
            {
                pt.x = p.x;
                pt.y = p.y;
                pt.z = p.z;
                pt.label = _LaneDivider;
                cloud.points.push_back(pt);
            }
        }
               
    }
}


