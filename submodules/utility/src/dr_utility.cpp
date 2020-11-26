#include "dr_utility.h"


void Utility::splitString(std::vector<std::string>& v, const std::string& s, const std::string& c)
{
	std::string::size_type pos1, pos2;
	pos2 = s.find(c);
	pos1 = 0;
	while (std::string::npos != pos2)
	{
		if (s.substr(pos1, pos2 - pos1) != "")
		{
			v.push_back(s.substr(pos1, pos2 - pos1));
		}

		pos1 = pos2 + c.size();
		pos2 = s.find(c, pos1);
	}
	if (pos1 != s.length())
		v.push_back(s.substr(pos1));
}

double Utility::GpsTime2UnixTime(int wn, double ws)
{
	double ts = (wn*SECOND_IN_A_WEEK + ws) + GPS_UNIX_OFFSET - GPS_LEAP_SECOND;
	return ts;
}

int64_t Utility::GpsTime2UnixTime64(int wn, double ws)
{
	int64_t ts64 = (wn*SECOND_IN_A_WEEK*1e6 + ws*1e6) + GPS_UNIX_OFFSET*1e6 - GPS_LEAP_SECOND*1e6;

	return ts64;
}

int64_t Utility::DateTime2UnixTime64(const std::string& str, const std::string& pattern /* = "%d-%d-%d %d:%d:%d.%d" */, 
									 const int fraction_part /* = 6 */, const int timezone /* = 0 */)
{
	char* cha = (char*)str.data(); 
	tm timeinfo;
	int year, month, day, hour, minute, second, subsecond;
	
#ifdef _WIN32
	sscanf_s(cha, pattern, &year, &month, &day, &hour, &minute, &second, &ns);
#else
	sscanf(cha, pattern.c_str(), &year, &month, &day, &hour, &minute, &second, &subsecond);
#endif

	timeinfo.tm_year = year - 1900;		// year, start from 1900
	timeinfo.tm_mon = month - 1;		// month, range: 0-11
	timeinfo.tm_mday = day;
	timeinfo.tm_hour = hour - timezone;
	timeinfo.tm_min = minute;
	timeinfo.tm_sec = second;
	
	timeinfo.tm_isdst = 0;
	time_t t = mktime(&timeinfo);		// convert form tm ot time_t

	double subsecond_d = double(subsecond) / pow(10, fraction_part);
	int64_t time_stamp = int64_t(t * 1e6) + int64_t(subsecond_d * 1.0e6);

	return time_stamp;
}	

std::string Utility::UnixTime642DateTime(const int64_t timestamp, const std::string& pattern /* = "%04d-%02d-%02d %02d:%02d:%02d.%06d" */, 
										  const int fraction_part /* = 6 */, const int timezone /* = 0 */)
{
	time_t time = timestamp / 1000000;
	int subsecond = timestamp % 1000000;

	int n = 6 - fraction_part;
	subsecond /= pow(10, n);

	struct tm timeinfo;
#ifdef _WIN32
	localtime_s(&timeinfo, &time);
#else
	struct tm * p_timeinfo;
	p_timeinfo = localtime(&time);
	timeinfo = *p_timeinfo;
#endif

	int year, month, day, hour, minute, second;
	year = timeinfo.tm_year + 1900;
	month = timeinfo.tm_mon + 1;
	day = timeinfo.tm_mday;
	hour = timeinfo.tm_hour;
	minute = timeinfo.tm_min;
	second = timeinfo.tm_sec;

	char s[64];
#ifdef _WIN32
	sprintf_s(s, pattern.c_str(), yearStr, monthStr, dayStr, hourStr, minuteStr, secondStr, subsecondStr);
#else
	sprintf(s, pattern.c_str(), year, month, day, hour, minute, second, subsecond);
#endif

	std::string str(s);
	return str;
}

int64_t Utility::GetSystemTimeStamp()
{
	auto now = std::chrono::system_clock::now();
    auto m = now.time_since_epoch();
    int64_t diff = std::chrono::duration_cast<std::chrono::microseconds>(m).count();

	return diff;
}



void Utility::NormlizeAngle(double& angle)
{
	while (angle < 0.0)
	{
		angle += 2.0*PI_;
	}

	while (angle >= 2.0*PI_)
	{
		angle -= 2.0*PI_;
	}
}

Matrix3d Attitude::Euler2DCM(Vector3d euler)
{
	//Matrix3d dcm;
	//dcm = AngleAxisd(euler[2], Vector3d::UnitZ())   // yaw
	//	* AngleAxisd(euler[0], Vector3d::UnitX())   // pitch
	//	* AngleAxisd(euler[1], Vector3d::UnitY());  // roll

	double p = euler(0);
	double r = euler(1);
	double y = euler(2);
	Matrix3d Rz, Ry, Rx, dcm;

	Rz << cos(y), -sin(y), 0.0,
		  sin(y),  cos(y), 0.0,
		  0.0,        0.0, 1.0;

	Rx << 1.0,    0.0,     0.0,
		  0.0, cos(p), -sin(p),
		  0.0, sin(p),  cos(p);

	Ry <<  cos(r), 0.0, sin(r),
		   0.0,    1.0,    0.0,
		  -sin(r), 0.0, cos(r);

	dcm = Rz * Rx * Ry;
	return dcm;
}

Vector3d Attitude::DCM2Euler(Matrix3d dcm)
{
	/*Vector3d euler, tmp;
	tmp = dcm.eulerAngles(2, 0, 1);
	euler << tmp(1), tmp(2), tmp(0);*/

	Eigen::Vector3d euler(Eigen::Vector3d::Zero());

	if (fabs(dcm(2, 1)) < 0.999999)
	{
		euler(0) = asin(dcm(2, 1));					// pitch
		euler(1) = -atan2(dcm(2, 0), dcm(2, 2));	// roll
		euler(2) = -atan2(dcm(0, 1), dcm(1, 1));	// yaw
	}
	else
	{
		//std::cout << "!!!pitch angle is +-90 degree" << std::endl;
		euler(0) = asin(dcm(2, 1));
		euler(1) = atan2(dcm(0, 2), dcm(0, 0)); // 90:roll-yaw; -90:TBD (roll+yaw or roll+yaw-180)
		euler(2) = 0.0;
	}

	return euler;
}

Matrix3d Attitude::Euler2DCM_FLU(Vector3d euler)
{
	double r = euler(0);
	double p = euler(1);
	double y = euler(2);
	Matrix3d Rz, Ry, Rx, dcm;

	Rz << cos(y), -sin(y), 0.0,
		  sin(y),  cos(y), 0.0,
		  0.0,        0.0, 1.0;

	Ry <<  cos(p), 0.0, sin(p),
		   0.0,    1.0,    0.0,
		  -sin(p), 0.0, cos(p);

	Rx << 1.0,    0.0,     0.0,
		  0.0, cos(r), -sin(r),
		  0.0, sin(r),  cos(r);

	dcm = Rz * Ry * Rx;
	return dcm;
}

Vector3d Attitude::DCM2Euler_FLU(Matrix3d dcm)
{
	Eigen::Vector3d euler(Eigen::Vector3d::Zero());

	if (fabs(dcm(2, 0)) < 0.999999)
	{
		euler(1) = -asin(dcm(2, 0));			// pitch
		euler(0) = atan2(dcm(2, 1), dcm(2, 2));	// roll
		euler(2) = atan2(dcm(1, 0), dcm(0, 0));	// yaw
	}
	else
	{
		//std::cout << "!!!pitch angle is +-90 degree" << std::endl;
		euler(0) = -asin(dcm(2, 0));
		euler(1) = atan2(dcm(0, 1), dcm(0, 2)); 
		euler(2) = 0.0;
	}

	return euler;
}

Matrix3d Attitude::Euler2DCM_RDF(Vector3d euler)
{
	double r = euler(0);
	double p = euler(1);
	double y = euler(2);
	Matrix3d Rz, Ry, Rx, dcm;

	Ry <<  cos(y), 0.0, sin(y),
		   0.0,    1.0,    0.0,
		  -sin(y), 0.0, cos(y);

	Rx << 1.0,    0.0,     0.0,
		  0.0, cos(p), -sin(p),
		  0.0, sin(p),  cos(p);

	Rz << cos(r), -sin(r), 0.0,
		  sin(r),  cos(r), 0.0,
		  0.0,        0.0, 1.0;

	dcm = Ry * Rx * Rz;
	return dcm;
}

Vector3d Attitude::DCM2Euler_RDF(Matrix3d dcm)
{
	Eigen::Vector3d euler(Eigen::Vector3d::Zero());

	if (fabs(dcm(1, 2)) < 0.999999)
	{
		euler(1) = -asin(dcm(1, 2));			// pitch
		euler(0) = atan2(dcm(1, 0), dcm(1, 1));	// roll
		euler(2) = atan2(dcm(0, 2), dcm(2, 2));	// yaw
	}
	else
	{
		//std::cout << "!!!pitch angle is +-90 degree" << std::endl;
		euler(0) = -asin(dcm(1, 2));
		euler(1) = atan2(dcm(2, 0), dcm(2, 1)); 
		euler(2) = 0.0;
	}

	return euler;
}


Quaterniond Attitude::DCM2Quat(Matrix3d dcm)
{
	Quaterniond quat;
	quat = dcm;
	return quat;
}

Matrix3d Attitude::Quat2DCM(Quaterniond& quat)
{
	Matrix3d dcm;
	quat.normalize();
	dcm = quat.toRotationMatrix();
	return dcm;
}

Quaterniond Attitude::RotVec2Quat(Vector3d rotVec)
{
	Quaterniond quat;
	double norm;
	double q0, s;
	norm = rotVec.transpose() * rotVec;
	if (norm < 1.0e-8)
	{
		q0 = 1 - norm * (1.0 / 8 - norm / 384);
		s = 1.0 / 2 - norm * (1.0 / 48 - norm / 3840);
	}
	else
	{
		double phi = sqrt(norm);
		q0 = cos(phi / 2);
		s = sin(phi / 2) / phi;
	}
	quat.w() = q0;
	quat.vec() = s * rotVec;

	return quat;
}

Vector3d Attitude::Quat2RotVec(Quaterniond q)
{
	if(q.w() < 0.0)
	{
		q.w() = -q.w();
		q.vec() = -q.vec();
	}

	double half_phi = acos(q.w());
	double b = 0.0;
	if(half_phi > 1e-20)
	{
		b = 2 * half_phi / sin(half_phi);
	}
	else
	{
		b = 2.0;
	}
	
	Vector3d rot = b * q.vec();
	return rot;
}

Quaterniond Attitude::Pos2Quat(Vector3d pos)
{
	Quaterniond quat;
	double s1, c1, s2, c2;

	s1 = sin(pos(0) / 2.0);
	c1 = cos(pos(0) / 2.0);
	s2 = sin(-PI_ / 4.0 - pos(1) / 2.0);
	c2 = cos(-PI_ / 4.0 - pos(1) / 2.0);

	quat.w() = c1 * c2;
	quat.x() = -s1 * s2;
	quat.y() = c1 * s2;
	quat.z() = c2 * s1;

	return quat;
}
Vector3d Attitude::Quat2Pos(Quaterniond & quat)
{
	Vector3d pos;
	pos(0) = -2 * atan(quat.y() / quat.w()) - PI_ / 2;
	pos(1) = 2 * atan2(quat.z(), quat.w());
	pos(2) = 0;
	return pos;
}

Matrix3d Attitude::Skew_Symmetic(Vector3d v)
{
	Matrix3d mat = Matrix3d::Zero(3, 3);
	mat(1, 0) = v(2);
	mat(2, 0) = -v(1);
	mat(0, 1) = -v(2);
	mat(2, 1) = v(0);
	mat(0, 2) = v(1);
	mat(1, 2) = -v(0);
	return mat;
}


