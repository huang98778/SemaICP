#pragma once

#include <Eigen/Dense>
#include <vector>
#include <string>
#include <iostream>
#include <ctime>
#include <chrono>
#include <memory>

// #include "dr_earth.h"

using namespace Eigen;

#define ZERO_EPS			1.0e-6
#define GPS_UNIX_OFFSET 	315964800		// 6.1.1980UTC(GPS) - 1.1.1970UTC(Unix)  
#define GPS_LEAP_SECOND 	18				// leap seconds, GPS's additional seconds from UTC
#define SECOND_IN_A_WEEK 	604800

#define PI_ 3.1415926

class Utility
{
public:
	static void splitString(std::vector<std::string>& v, const std::string& s, const std::string& c);

	static double GpsTime2UnixTime(int wn, double ws);		// in second
	static int64_t GpsTime2UnixTime64(int wn, double ws);	// in micro second

	static int64_t DateTime2UnixTime64(const std::string& str, const std::string& pattern = "%d-%d-%d %d:%d:%d.%d", 
									   const int fraction_part = 6, const int timezone = 0);	// str to us
	static std::string UnixTime642DateTime(const int64_t timestamp, const std::string& pattern = "%04d-%02d-%02d %02d:%02d:%02d.%06d", 
										   const int fraction_part = 6, const int timezone = 0);	// us to str
	
	static int64_t GetSystemTimeStamp();

	static void NormlizeAngle(double& angle);		// [0, 2pi]
};

class Attitude
{
public:
	EIGEN_MAKE_ALIGNED_OPERATOR_NEW;
public:
	static Matrix3d Euler2DCM(Vector3d euler);	// RFU and pry;
	static Vector3d DCM2Euler(Matrix3d dcm);	// RFU and pry;
	static Matrix3d Euler2DCM_FLU(Vector3d euler);	// FLU and rpy;
	static Vector3d DCM2Euler_FLU(Matrix3d dcm);	// FLU and rpy;
	static Matrix3d Euler2DCM_RDF(Vector3d euler);	// RDF and rpy;
	static Vector3d DCM2Euler_RDF(Matrix3d dcm);	// RDF and rpy;

	static Quaterniond DCM2Quat(Matrix3d dcm);
	static Matrix3d Quat2DCM(Quaterniond& quat);

	static Quaterniond RotVec2Quat(Vector3d rotVec);
	static Vector3d Quat2RotVec(Quaterniond q);

	static Quaterniond Pos2Quat(Vector3d pos);
	static Vector3d Quat2Pos(Quaterniond& quat);

	static Matrix3d Skew_Symmetic(Vector3d v);
};

class TicToc
{
public:
	TicToc()
	{
		tic();
	}

	void tic()
	{
		start = std::chrono::system_clock::now();
	}

	double toc()
	{
		end = std::chrono::system_clock::now();
		std::chrono::duration<double> elapsed_seconds = end - start;
		
		return elapsed_seconds.count() * 1000;		// ms
	}

	double toc_tic()
	{
		end = std::chrono::system_clock::now();
		std::chrono::duration<double> elapsed_seconds = end - start;
		start = std::chrono::system_clock::now();

		return elapsed_seconds.count() * 1000;		// ms
	}

private:
	std::chrono::time_point<std::chrono::system_clock> start, end;
};


