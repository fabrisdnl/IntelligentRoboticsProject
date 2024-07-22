#include "assignment_1/detect.h"
#include "assignment_1/detectCV.h"
#include <cv_bridge/cv_bridge.h>

// OpenCV headers
#include <opencv2/objdetect/objdetect.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>


/* 
**  Function in order to get x y coordinates from polar data and compute max spatial distance from robot.
*/
void updateXYParameters(sensor_msgs::LaserScanConstPtr& scan, Point (&laserDistances)[666], float& max)
{
	//max distance from robot
    max = 0.0;

	//convert each laser data into XY coordinates and update max distance
    for (int i=0; i<666; i++){
        float angle = scan->angle_min + (scan->angle_increment * i);
        laserDistances[i].x = scan->ranges.at(i) * cos(angle);
        laserDistances[i].y = scan->ranges.at(i) * sin(angle);      
        float dist = sqrt(laserDistances[i].x*laserDistances[i].x+laserDistances[i].y*laserDistances[i].y); 
        if ( dist > max) 
        {
            max = dist;
        }
    }
}


/*
	** Function to computing the transformation from the frame of laser sensor
	** to the frame of the robot (base_link). 
*/
std::vector<geometry_msgs::Point> transformPositionsCV(const sensor_msgs::LaserScanConstPtr& scan, std::vector<PointSet>& rawObstacles, bool& success)
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
	
	for (int i = 0; i < rawObstacles.size(); i++)
	{	
		geometry_msgs::Point base_link;
		base_link.x = rawObstacles.at(i).middle.x + transform.getOrigin().getX();
		base_link.y = rawObstacles.at(i).middle.y + transform.getOrigin().getY();
		base_link.z = 0.0;
		/* Ignore parts to close to base_link because are the parts of the robot. */
		if (abs(base_link.x) < 0.2 || abs(base_link.y) < 0.1) continue;
		else positions.push_back(base_link);
	}
	success = true;
	return positions;
}

/*
** Main function in order to detect the obstacles, from the laser scan data. 
** Laser scan data are converted into a "distance map" image and processed to extract obstacles
*/
std::vector<geometry_msgs::Point> detectionCV(sensor_msgs::LaserScanConstPtr& scan, bool &success)
{
	//list containing laser scan in xy coordinates
    Point laserDistances[666];
	//max spatial distance from robot
    float maxDistance;

    //ROS_INFO("SIZE [%f]", scan->ranges.size());

	//get XY coordinates from laser and compute max distance
	updateXYParameters(scan, laserDistances, maxDistance);

    //gray image containing distance map 
    cv::Mat distanceMap(cv::Size(666, 666), CV_8UC1);

	//generate distance map, from right to left to be coherent with laser scan spatial data (from right to left)
    int a = 0;
    for (int i = 665; i >= 0; i--)
    {
        for (int j = 665; j >= 0; j--)
        {
        	distanceMap.at<uchar>(j, i) = ceil(sqrt(laserDistances[a].x*laserDistances[a].x+laserDistances[a].y*laserDistances[a].y)/maxDistance*255);
        }
        a++;
    }
    
	//show distance map
	/*
    const std::string win1 = "Distance Map";
    cv::namedWindow(win1, cv::WINDOW_FREERATIO);
	//cv::imshow(win1, distanceMap);
	*/

    //cv::imwrite("distanceMap.jpg" ,distanceMap);

	ROS_INFO("DETECTION");

	//copmute/generate background image from distance map by applying a gaussian blur
    cv::Mat background;
    cv::GaussianBlur(distanceMap, background, cv::Size(275, 275), 25);
    //cv::imwrite("background.jpg" ,background);
    
	//difference mask in order to get obsatacles
    cv::Mat mask = cv::Mat::zeros(666, 666, CV_8UC1);
    
	//only differences higher than threshold are detected
	int diffThresh = 10;

    for (int i = 0; i<666; i++)
    {
    	for(int j = 0; j<666; j++)
    	{
    		if (background.at<uchar>(i, j)-distanceMap.at<uchar>(i, j)>diffThresh)
    		{
    			mask.at<uchar>(i, j) = 255;
    		}
    	}
    }
    
    //cv::imwrite("mask.jpg" ,mask);
    
	//vector containing detected obstacles
    std::vector<PointSet> rawObstacles;
	//vector containing index of the obstancles middle point
    std::vector<int> rawObstaclesMiddlePoint;
    

	//Detect obstacles from the image, obstacles are white stripes 
	//detection is computed from left to rigth, it needs 655-index to get corrrsponding laser index (that goes from right to left/anti-clockwise)
    bool end = false;
    int index = 0;
	int obstacleThresh = 3;
    
    while (index < 666)
    {
    	//ROS_INFO("value: [%i]", mask.at<uchar>(333, index));
    	if (mask.at<uchar>(333, index) == 255)
    	{
    		PointSet point;
    		point.first = laserDistances[665 - index];
    		int firstIndex = index;
    		bool white = true;
    		bool isObstacle = false;
    		index++;

			//for each white stripe get first, last and compute middle index 
			//get the corresponding laser scan data
			//if white stripes are not larger than 3 px, they are not considered as obstacles
    		while (white)
    		{
    			if (mask.at<uchar>(333, index) == 255 && index < 665) index++;
    			else 
    			{
    				if (index - firstIndex > obstacleThresh)
    				{
					point.last = laserDistances[665 - index];
					int middle = ceil((firstIndex + index) / 2);
					point.middle = laserDistances[665 - middle];
					rawObstaclesMiddlePoint.push_back(middle);
					isObstacle = true;
    				}
    				white = false;
    			}
    		}
    		if (isObstacle) rawObstacles.push_back(point);
    	}
    	else index++;
    }
    
    //debug and show obstacles map (removing first and last obstacles that are part of te robot)
    for (int i = 1; i<rawObstaclesMiddlePoint.size()-1; i++)
    {
    	ROS_INFO("Obstacle at index: [%i]", rawObstaclesMiddlePoint.at(i));
		ROS_INFO("Obstacle at laser frame coordinates: [%f],[%f]", rawObstacles.at(i).middle.x, rawObstacles.at(i).middle.x);
		for(int j = 0; j<666; j++)
		{
			mask.at<uchar>(j, rawObstaclesMiddlePoint.at(i)) = 150;
		}
    }
    
    //cv::imwrite("mask.jpg" ,mask);

	// Reverse the obstacles vector to be coherent with laser scan (from rigth to left/anti-clockwise)
    reverse(rawObstacles.begin(), rawObstacles.end());
    
    return transformPositionsCV(scan, rawObstacles, success);	
}
        
        
