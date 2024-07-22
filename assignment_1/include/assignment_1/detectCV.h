#ifndef DETECTCV_H
#define DETECTCV_H

#include <ros/ros.h>
#include <sensor_msgs/LaserScan.h>
#include <tf/transform_listener.h>
#include <math.h>

void updateEveryIndexParameters(sensor_msgs::LaserScanConstPtr& scan, Point (&laserDistances)[666], float& max);

std::vector<geometry_msgs::Point> publishObstaclesSimplex(const sensor_msgs::LaserScanConstPtr& scan, std::vector<PointSet>& rawObstacles, bool& success);

std::vector<geometry_msgs::Point> detectionCV(sensor_msgs::LaserScanConstPtr& scan, bool &success);

#endif
