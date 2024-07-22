#ifndef DETECT_H
#define DETECT_H

#include <ros/ros.h>
#include <sensor_msgs/LaserScan.h>
#include <tf/transform_listener.h>
#include <math.h>

struct Point
{
	float x;
	float y;
};

struct PointSet
{
	std::vector<Point> points;
	Point first;
	Point middle;
	Point last;
};

struct Circle
{
	Point center;
	float radius;
};

class Detect
{
    protected:
		std::vector<Point> points_;
        std::vector<PointSet> sets_;
		std::vector<Circle> circles_;

		const int min_points = 7;
		const float max_distance = 0.15;
		const float max_radius = 0.3;

	public:	
        
        Detect();

		std::vector<geometry_msgs::Point> detection(const sensor_msgs::LaserScanConstPtr& scan, bool& success);
		void clustering(const sensor_msgs::LaserScanConstPtr& scan);
		void getCircles();
		std::vector<geometry_msgs::Point> transformPositions(const sensor_msgs::LaserScanConstPtr& scan, bool& success);
        
};

#endif
