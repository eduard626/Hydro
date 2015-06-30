#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>

static const std::string OPENCV_WINDOW = "Image window";
int move_dir,servo;

class ImageConverter
{
	ros::NodeHandle nh_;
	image_transport::ImageTransport it_;
	image_transport::Subscriber image_sub_;
	image_transport::Publisher image_pub_;	
  
	public:
		ImageConverter()
		: it_(nh_)
		{
			// Subscrive to input video feed and publish output video feed
			image_sub_ = it_.subscribe("/image_raw", 1,&ImageConverter::imageCb, this);
			image_pub_ = it_.advertise("/image_converter/output_video", 1);
			cv::namedWindow(OPENCV_WINDOW,CV_WINDOW_NORMAL);
			cv::setWindowProperty(OPENCV_WINDOW,CV_WND_PROP_FULLSCREEN, 	CV_WINDOW_FULLSCREEN);
			cv::setMouseCallback(OPENCV_WINDOW,onMouse,0);
			move_dir=servo=0;
		}
	
		~ImageConverter()
		{
			cv::destroyWindow(OPENCV_WINDOW);
		}

	static void onMouse( int event, int x, int y, int, void* )
	{
	
		if( event == cv::EVENT_MBUTTONDOWN )
		{
			if (servo==0)	servo=1;
			else	servo=0;
		}
		if( event == cv::EVENT_LBUTTONDOWN )
		{
			move_dir=1;
		}
		if( event == cv::EVENT_RBUTTONDOWN )
		{
			move_dir=-1;
		}
		if (event == cv::EVENT_RBUTTONUP || event==cv::EVENT_LBUTTONUP)
		{
			move_dir=0;
		}
	}

	void imageCb(const sensor_msgs::ImageConstPtr& msg)
		{
			cv_bridge::CvImagePtr cv_ptr;
			try
			{
				cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
			}
			catch (cv_bridge::Exception& e)
			{
				ROS_ERROR("cv_bridge exception: %s", e.what());
				return;
			}
	
			// Draw an example circle on the video stream
			if (cv_ptr->image.rows > 60 && cv_ptr->image.cols > 60)
				cv::circle(cv_ptr->image, cv::Point(50, 50), 10, CV_RGB(255,0,0));
			// Update GUI Window
			cv::imshow(OPENCV_WINDOW, cv_ptr->image);
			cv::waitKey(3);
			ROS_INFO("Servo: %d, Direction: %d",servo,move_dir);
	    		// Output modified video stream
	   		image_pub_.publish(cv_ptr->toImageMsg());
		}
	};

int main(int argc, char** argv)
{
	ros::init(argc, argv, "image_converter");
	ImageConverter ic;
	ros::spin();
	return 0;
}
