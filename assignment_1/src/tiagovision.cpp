// ROS HEADERS
#include <ros/ros.h>
#include <sensor_msgs/image_encodings.h>
#include <sensor_msgs/Image.h>
#include <image_transport/image_transport.h>

// OpenCV headers
#include <cv_bridge/cv_bridge.h>
#include <opencv2/objdetect/objdetect.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>

//This script allow the user to see what Tiago is seeing accessing to its RBG camera
//image processing and obstacle detection on RBG images are only decleared but not implemented 
//in order to have the possibility to have a future implementation 
//By running this script it will appeara window showing the Tiago RBG camera stream

class TiagoVision
{
public:
	TiagoVision(ros::NodeHandle nh_);
	~TiagoVision();
	image_transport::ImageTransport _imageTransport;
	image_transport::Subscriber image_sub;

protected:
	void imageCB(const sensor_msgs::ImageConstPtr& msg);
	void ImageProcessing();
	void ObstacleDetection(cv::Mat &output_);

private:
	cv::Mat img_bgr, img1, img2, detection_img;
	int i;
};


//live camera feed and obstacle detection result windows
const std::string win1 = "Live Camera Feed";
//const std::string win2 = "Obstacle TiagoVision";



TiagoVision::TiagoVision(ros::NodeHandle nh_): _imageTransport(nh_)
{
    //get raw images data from xtion/rgb/image_raw
	image_sub = _imageTransport.subscribe("xtion/rgb/image_raw", 1, &TiagoVision::imageCB, this, image_transport::TransportHints("compressed"));

    //live camera feed and obstacle detection result windows
	cv::namedWindow(win1, cv::WINDOW_FREERATIO);
	//cv::namedWindow(win2, cv::WINDOW_FREERATIO);
	i=0;
}

TiagoVision::~TiagoVision()
{
	cv::destroyWindow(win1);
	//cv::destroyWindow(win2);
}


void TiagoVision::imageCB(const sensor_msgs::ImageConstPtr& msg)
{
	cv_bridge::CvImagePtr cvPtr;
	try
	{ 
		cvPtr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
	}
	catch (cv_bridge::Exception& e) 
	{
		ROS_ERROR("cv_bridge exception: %s", e.what());
		return;
	}

    //get image in BGR format
	cvPtr->image.copyTo(img_bgr);

    //convert BGR to Gray for processing efficiency 
	cv::cvtColor(cvPtr->image, img1, cv::COLOR_BGR2GRAY);

    //process image
	this->ImageProcessing();

	img1.copyTo(img2);
}

void TiagoVision::ImageProcessing()
{
	if(i!=0)
	{	
        //some image processing

        //obstacle detection
		this->ObstacleDetection(img_bgr);

        //show live image and processed obstacle detection
		cv::imshow(win1, img_bgr);
		//cv::imshow(win2, detection_img);
	}
	++i;
	cv::waitKey(1);
}

void TiagoVision::ObstacleDetection(cv::Mat &output_)
{
	//obstacle detection on RBG image/video
}





int main(int argc, char** argv)
{
	ros::init(argc, argv, "ObstacleDetection");
	ros::NodeHandle nh;
	TiagoVision dt(nh);
	ros::spin();
}