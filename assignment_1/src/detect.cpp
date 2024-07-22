#include "assignment_1/detect.h"

Detect::Detect()
{
	points_.clear();
	sets_.clear();
	circles_.clear();
}

/*
** Main function in order to detect the cylinder obstacles, from the laser scan data. 
*/
std::vector<geometry_msgs::Point> Detect::detection(const sensor_msgs::LaserScanConstPtr& scan, bool& success)
{
	for (int i = 0; i < scan->ranges.size(); ++i)
	{
		float range = scan->ranges.at(i);
		/* Converting the positions of laser scan into coordinates in robot's frame. */
		if (range > scan->range_min && range < scan->range_max)
		{
			float theta = scan->angle_min + i * scan->angle_increment;
			Point temp;
			temp.x = range * cos(theta);
			temp.y = range * sin(theta);
			points_.push_back(temp);
		}
	}
	/* Grouping the point and saving PointSet variables. */
	clustering(scan);
	/* Checking if the sets of points are actually circles. */
	getCircles();
	/* Publishing the obstacles. */
	return transformPositions(scan, success);
}

/*
** Function to group together close points, but only if there is a 
** sufficient number of points.
*/
void Detect::clustering(const sensor_msgs::LaserScanConstPtr& scan)
{
	int start = 0, end = 0;
	int counter = 1;
	
	for (int i = 1; i < points_.size(); ++i)
	{
		/* Compute distance from current point. */
		float range = scan->ranges.at(i);
		Point p;
		p.x = points_.at(i).x - points_.at(end).x;
		p.y = points_.at(i).y - points_.at(end).y;
		float distance = sqrt(pow(p.x, 2) + pow(p.y, 2));
		/* If the distance is lower, add the point into the current set. */
		if (distance < max_distance + range * scan->angle_increment)
		{
			end = i;
			counter++;
		}
		/* Otherwise check if could be a PointSet. */
		else
		{
			if (counter >= min_points)
			{
				/* Create and save the PointSet. */
				PointSet temp;
				temp.first = points_.at(start);
				temp.last = points_.at(end);
				temp.middle = points_.at(start + (end - start) / 2);
				for (int j = start; j <= end; ++j)
				{
					temp.points.push_back(points_.at(j));
				}
				sets_.push_back(temp);
			}
			/* Resetting variables. */
			start = i;
			end = i;
			counter = 1;
		}
	}
	/* Check the last group of points too. */
	if (counter >= min_points)
	{
		PointSet temp;
		temp.first = points_.at(start);
		temp.middle = points_.at(start + (end - start) / 2);
		temp.last = points_.at(end);
		for (int j = start; j <= end; ++j)
		{
			temp.points.push_back(points_.at(j));
		}
		sets_.push_back(temp);
	}
}

/*
** Function to verify if each set of points could be a circle, then a 
** cylinder obstacle, and save it if satisfy our case's specific 
** conditions (radius).
*/
void Detect::getCircles()
{
	float th_slope = 0.0000001;
	float ths = 0.01;
	for (PointSet set: sets_)
	{
		/* Computing center and radius of a circle given 3 points. */
		Circle circle;
		bool confirmed = false;
		float x1 = set.first.x, y1 = set.first.y;
		float x2 = set.middle.x, y2 = set.middle.y;
		float x3 = set.last.x, y3 = set.last.y;
		float x21 = x2 - x1, y21 = y2 - y1;
		float x32 = x3 - x2, y32 = y3 - y2;
		float s1 = y21 / x21;
		float s2 = y32 / x32;
		if (abs(s1 - s2) < th_slope) continue;
		else {
			circle.center.x = (s1 * s2 * (y1 - y3) + s2 * (x1 + x2) - s1 * (x2 + x3)) / (2 * (s2 - s1));
			circle.center.y = -1 * (circle.center.x - (x1 + x2) / 2) / s1 + (y1 + y2) / 2;
			Point temp;
			temp.x = set.middle.x - circle.center.x;
			temp.y = set.middle.y - circle.center.y;
			circle.radius = sqrt(pow(temp.x, 2) + pow(temp.y, 2));
			/* Check on radius of cylinder (setted by our specific case). */
			if (circle.radius < max_radius)
			{
				for (int i = 0; i < set.points.size(); ++i)
				{
					Point pt;
					pt.x = set.points.at(i).x - circle.center.x;
					pt.y = set.points.at(i).y - circle.center.y;
					float distance = sqrt(pow(pt.x, 2) + pow(pt.y, 2));
					if (distance >= circle.radius - ths && distance <= circle.radius + ths)
					{
						confirmed = true;
					}
				}
				if (confirmed) circles_.push_back(circle);
			}
		} 
	}
}

/*
** Function to computing the transformation from the frame of laser sensor
** to the frame of the robot (base_link), ignoring parts of robot scanned. 
*/
std::vector<geometry_msgs::Point> Detect::transformPositions(const sensor_msgs::LaserScanConstPtr& scan, bool& success)
{
	std::vector<geometry_msgs::Point> positions;
	/* The scanner is mounted on the frontal part of the robot, while the base_link is in the center */
	tf::TransformListener listener;
	tf::StampedTransform transform;
	listener.waitForTransform("base_link", scan->header.frame_id.c_str(), ros::Time(0), ros::Duration(5.0));
	try
	{
		listener.lookupTransform("base_link", scan->header.frame_id.c_str(), ros::Time(0), transform);
	}
	catch(tf::TransformException& exc)
	{
		ROS_ERROR("Exception trying to transform: %s.", exc.what());
		success = false;
		return positions;
	}
	
	for (Circle circle : circles_)
	{	
		geometry_msgs::Point base_link;
		base_link.x = circle.center.x + transform.getOrigin().getX();
		base_link.y = circle.center.y + transform.getOrigin().getY();
		base_link.z = 0.0;
		/* Ignore parts to close to base_link because are the parts of the robot. */
		if (abs(base_link.x) < 0.2 || abs(base_link.y) < 0.1) continue;
		else positions.push_back(base_link);
	}
	success = true;
	return positions;
}
