#include <ros/ros.h>
#include <cv_bridge/cv_bridge.h>
#include <image_transport/image_transport.h>
#include <sensor_msgs/image_encodings.h>
#include <opencv2/opencv.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/objdetect/objdetect.hpp>

static const std::string OPENCV_WINDOW = "image_window";
using namespace cv;

class image_circle{
	//First start the nodehandle of the ROS
	ros::NodeHandle nh_;

	//initializing the image_transport for using opencv with ROS
	image_transport::ImageTransport it_;

	//initialising one subscriber A and pubblisher B
	image_transport::Subscriber image_sub_;
	image_transport::Publisher image_pub_;

	public :
	image_circle(): it_(nh_)
	{
		//Now we subscribe to the the topic of video feed and then publish into the ouput video feed.
		image_sub_=it_.subscribe("/usb_cam/image_raw",1,&image_circle::imagecb,this); //first part is the topic to subscribe second is  the size of the outgoing message queue third is the path to direct the subscribed image .
		image_pub_=it_.advertise("/image_converter/output_video",1);
		cv::namedWindow(OPENCV_WINDOW);

	}
	~image_circle(){
		cv::destroyWindow(OPENCV_WINDOW);
	}


	void imagecb(const sensor_msgs::ImageConstPtr& msg)
	{
		cv_bridge::CvImagePtr cv_ptr; //cv_ptr is the place for storing any image.
		try
    	{
      cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8); //HERE WE ARE GETTING THE IMAGE IN ROS MSG FORM 'msg' AND WE ARE COPYING IT TO cv_ptr  WHICH IS A IMAGE TYPE.
    	//Since we're going to draw on the image, we need a mutable copy of it, so we use toCvCopy(). sensor_msgs::image_encodings::BGR8 is simply a constant for "bgr8", but less susceptible to typos.
    	}

    catch (cv_bridge::Exception& e)
    	{
      ROS_ERROR("cv_bridge exception: %s", e.what());
      return;

    	}

    
    circle(cv_ptr->image,cv::Point(50,50),10,CV_RGB(255,0,0));

    	
    imshow(OPENCV_WINDOW,cv_ptr->image);
    waitKey(3);

    image_pub_.publish(cv_ptr->toImageMsg());





	}
};
int main(int argc, char** argv)
{
  ros::init(argc, argv, "imagecircle");
  image_circle ic;
  ros::spin();
  return 0;
}


	

	