#include <ros/ros.h>
#include <cv_bridge/cv_bridge.h>
#include <image_transport/image_transport.h>
#include <sensor_msgs/image_encodings.h>
#include <opencv2/opencv.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/objdetect/objdetect.hpp>
#include <cam_test_pkg/centroid.h>


using namespace cv;
using namespace std; //using "namespace " we have given unique identity to that cv or std . It happens sometimes that we have some , which is similar to the name
                     //in the directories so by using namespaces we have given special importance to the name.


static const std::string OPENCV_WINDOW = "image_window"; //static variable are initialized once only . If a variable is static then the compiler goes to its initialization 
                             //and declaration only once . const makes a variable constant and the complier gives a run time error if its value is tried to change.

static const std::string OPENCV_WINDOW_1= "tracked_image";



class image_circle{
	//First start the nodehandle of the ROS
	ros::NodeHandle nh_;

	//initializing the image_transport for using opencv with ROS
	image_transport::ImageTransport it_;

	//initialising one subscriber A and pubblisher B
	image_transport::Subscriber image_sub_;
	image_transport::Publisher image_pub_;

	//Now we have to create a publisher for publishing the centroid .
	ros::Publisher face_centroid_pub;
	//Now create a oblect of the centroid type .
	cam_test_pkg::centroid face_centroid_var;

	//Now we need some variables to store the data. 
	string input_image_topic, output_image_topic, haar_file_face; 
	int face_tracking, display_original_image, display_tracking_image, center_offset, screenmaxx;




	public :
	image_circle(): it_(nh_)

	{

		haar_file_face = "/home/rishotics/ros_robotics_projects_ws/src/cam_test_pkg/data/face.xml";
  		face_tracking = 1;
 	 	display_original_image = 0;
  		display_tracking_image = 1;
  		screenmaxx = 640;
  		center_offset = 100;
		

		//Now we subscribe to the the topic of video feed and then publish into the ouput video feed.
		image_sub_=it_.subscribe("/usb_cam/image_raw",1,&image_circle::imagecb,this); //first part is the topic to subscribe second is  the size of the outgoing message queue third is the path to direct the subscribed image .
		image_pub_=it_.advertise("/face_detector/raw_image",1);
		

		//we can guide the publisher to have a path to publish the centroid of the face.
		face_centroid_pub=nh_.advertise<cam_test_pkg::centroid>("/face_centroid",10); 



		//cv::namedWindow(OPENCV_WINDOW);

	}
	~image_circle(){
		cv::destroyWindow(OPENCV_WINDOW);
	}


	void imagecb(const sensor_msgs::ImageConstPtr& msg)//now I understood , sensor_msgs we are getting from the sensor and now we go iside the sensor_image to the ImageConstPtr address .
	{
		cv_bridge::CvImagePtr cv_ptr; //cv_ptr is the place for storing any image.
		namespace enc=sensor_msgs::image_encodings;//now we are creating a variable 'enc' to store the encodings of the image.
		try
    	{
      cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8); //HERE WE ARE GETTING THE IMAGE IN ROS MSG FORM 'msg' AND WE ARE COPYING IT TO cv_ptr  WHICH IS A IMAGE TYPE.
    	//Since we're going to draw on the image, we need a mutable copy of it, so we use toCvCopy(). sensor_msgs::image_encodings::BGR8 is simply a constant for "bgr8", but less susceptible to typos.
    	//we know that there are several parts of the sensor_msg namely image , encodings etc
    	}

    catch (cv_bridge::Exception& e)
    	{
      ROS_ERROR("cv_bridge exception: %s", e.what());
      return;

    	}


    	string cascadename=haar_file_face;    	//Setting the location of finding the face file to the cascade name.

    		CascadeClassifier cascade;        //making a object of the classifier.

    		if(!cascade.load(cascadename))
    		{
    				cerr << "ERROR: Could not load classifier cascade" << endl;

    		}

	  //   	if (display_original_image == 1)
	  //  		 {
			// imshow("image_window", cv_ptr->image);
	  //   	}

	    	detectanddraw(cv_ptr->image,cascade);
// circle(cv_ptr->image,cv::Point(50,50),10,CV_RGB(255,0,0));
//waitKey(3);
// (display_tracking_image == 1){

    		imshow( "Face Tracker", cv_ptr->image );
    		waitKey(3);
    		namedWindow("Face Tracker",WINDOW_AUTOSIZE);
//}
    		image_pub_.publish(cv_ptr->toImageMsg());


	}
	void detectanddraw(Mat& img, CascadeClassifier& cascade){
		//we need a Mat dataype variable to storee the image converted
		    double t = 0;
    		double scale = 1;
    		vector<Rect> faces, faces2;
    const static Scalar colors[] =
    {
        Scalar(255,0,0),
        Scalar(255,128,0),
        Scalar(255,255,0),
        Scalar(0,255,0),
        Scalar(0,128,255),
        Scalar(0,255,255),
        Scalar(0,0,255),
        Scalar(255,0,255)
    };
    Mat gray, smallImg;

    cvtColor( img, gray, COLOR_BGR2GRAY );
    double fx = 1 / scale ;
    resize( gray, smallImg, Size(), fx, fx, INTER_LINEAR );
    equalizeHist( smallImg, smallImg );

    t = (double)cvGetTickCount();
    cascade.detectMultiScale( smallImg, faces,
        1.1, 15, 0
        |CASCADE_SCALE_IMAGE,
        Size(30, 30) );
   
    t = (double)cvGetTickCount() - t;

    for ( size_t i = 0; i < faces.size(); i++ )
    {
        Rect r = faces[i];
        Mat smallImgROI;
        vector<Rect> nestedObjects;
        Point center;
        Scalar color = colors[i%8];
        int radius;

        double aspect_ratio = (double)r.width/r.height;
        if( 0.75 < aspect_ratio && aspect_ratio < 1.3 )
        {
            center.x = cvRound((r.x + r.width*0.5)*scale);
            center.y = cvRound((r.y + r.height*0.5)*scale);
            radius = cvRound((r.width + r.height)*0.25*scale);
            circle( img, center, radius, color, 3, 8, 0 );

   	    face_centroid_var.x = center.x;
   	    face_centroid_var.y = center.y;

  
            //Publishing centroid of detected face
  	    face_centroid_pub.publish(face_centroid_var);

        }
        else
            rectangle( img, cvPoint(cvRound(r.x*scale), cvRound(r.y*scale)),
                       cvPoint(cvRound((r.x + r.width-1)*scale), cvRound((r.y + r.height-1)*scale)),
                       color, 3, 8, 0);

    }

    //Adding lines and left | right sections 

    Point pt1, pt2,pt3,pt4,pt5,pt6;

    //Center line
    pt1.x = screenmaxx / 2;
    pt1.y = 0;
 
    pt2.x = screenmaxx / 2;
    pt2.y = 480;


    //Left center threshold
    pt3.x = (screenmaxx / 2) - center_offset;
    pt3.y = 0;

    pt4.x = (screenmaxx / 2) - center_offset;
    pt4.y = 480;

    //Right center threshold
    pt5.x = (screenmaxx / 2) + center_offset;
    pt5.y = 0;

    pt6.x = (screenmaxx / 2) + center_offset;
    pt6.y = 480;


    line(img,  pt1,  pt2, Scalar(0, 0, 255),0.2);
    line(img,  pt3,  pt4, Scalar(0, 255, 0),0.2);
    line(img,  pt5,  pt6, Scalar(0, 255, 0),0.2);


    putText(img, "Left", cvPoint(50,240), FONT_HERSHEY_SIMPLEX, 1, cvScalar(255,0,0), 2, CV_AA);
    putText(img, "Center", cvPoint(280,240), FONT_HERSHEY_SIMPLEX, 1, cvScalar(0,0,255), 2, CV_AA);
    putText(img, "Right", cvPoint(480,240), FONT_HERSHEY_SIMPLEX, 1, cvScalar(255,0,0), 2, CV_AA);

   

}

};
int main(int argc, char** argv)
{
  ros::init(argc, argv, "imagecircle");
  image_circle ic;
  ros::spin();
  return 0;
}


	

	